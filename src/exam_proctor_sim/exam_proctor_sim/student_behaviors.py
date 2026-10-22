#!/usr/bin/env python3
"""Animate the classroom students and publish ground truth of what each one is doing.

Each student plays back a sequence of behaviour episodes. Most are ordinary exam
behaviour (writing, reading, thinking, glancing around, stretching); with a small
per-student probability an episode is a cheating behaviour (peeking at the
neighbour's paper, a phone or cheat sheet on the lap, passing a note,
whispering, turning around).

An episode is a list of keyframes. This node interpolates joint values between
them, runs the rig's forward kinematics and streams every student link pose to
Gazebo, where the StudentAnimator system applies them. Ground truth for every student is published on
/classroom/ground_truth.

A behaviour can be forced by publishing '<student_id> <behavior>' on
/classroom/trigger, e.g.  ros2 topic pub --once /classroom/trigger std_msgs/msg/String "{data: 'r1_c2 phone'}"
"""
import math
import random
from collections import namedtuple
from functools import lru_cache

from exam_proctor_msgs.msg import ClassroomState, StudentState
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String

from exam_proctor_sim import classroom_layout as layout
from exam_proctor_sim import student_rig as rig

# Move to `pose` over `move` s, then hold for `hold` s while `wiggle`
# ({joint: (amplitude, frequency_hz)}) adds small periodic motion.
Key = namedtuple('Key', 'phase pose move hold wiggle cheating_visible')


def key(phase, pose, move, hold, wiggle=None, cheating_visible=False):
    return Key(phase, pose, move, hold, wiggle or {}, cheating_visible)


# ------------------------------------------------------------------ poses
# Joints missing from a pose go to 0: upright, arms down, props hidden. Arm angles that
# must put a hand somewhere (on the desk, on the phone, at the neighbour) are solved with
# the rig's IK, so hands stay on the desk surface whatever the torso is doing.
TORSO = ('torso_pitch', 'torso_roll', 'torso_yaw')
DESK_TOP = layout.DESK_HEIGHT
HAND_ON_DESK_Z = DESK_TOP + 0.055            # hand sphere (r=4.5 cm) resting on the desk
DESK_HANDS = {'l': (0.40, 0.17, HAND_ON_DESK_Z), 'r': (0.40, -0.17, HAND_ON_DESK_Z)}
PEN_HAND = (0.46, -0.06, HAND_ON_DESK_Z)     # right hand on the exam paper
ELBOWS = {'l': (0.20, 0.27, 0.80), 'r': (0.20, -0.27, 0.80)}   # loose guide: elbows out, near desk height
# Desk top in the chair frame (x0, x1, y0, y1, top); IK keeps arms above it.
DESK_BOX = (layout.CHAIR_OFFSET - layout.DESK_DEPTH / 2, layout.CHAIR_OFFSET + layout.DESK_DEPTH / 2,
            -layout.DESK_WIDTH / 2, layout.DESK_WIDTH / 2, DESK_TOP)
LAP_ARM = dict(shoulder_pitch=-0.25, shoulder_roll=-0.4, elbow=1.65)   # hand resting on the lap


@lru_cache(maxsize=None)
def _solve_arm(torso, side, hand, elbow):
    return rig.solve_arm(dict(zip(TORSO, torso)), side, hand, elbow, avoid=DESK_BOX)


def arm_to(pose, side, hand, elbow=None):
    """`pose` with the `side` arm solved so its hand reaches `hand` (chair frame)."""
    torso = tuple(round(pose.get(j, 0.0), 3) for j in TORSO)
    return dict(pose, **_solve_arm(torso, side, tuple(hand), tuple(elbow) if elbow else None))


def hands_on_desk(pose, writing=True, sides='lr'):
    for side in sides:
        hand = PEN_HAND if writing and side == 'r' else DESK_HANDS[side]
        pose = arm_to(pose, side, hand, ELBOWS[side])
    return pose


def side_of(direction):
    return 'l' if direction > 0 else 'r'


def with_arm(pose, side, **arm):
    return dict(pose, **{f'{side}_{joint}': value for joint, value in arm.items()})


WRITE = hands_on_desk(dict(torso_pitch=0.15, head_pitch=0.55))
READ = hands_on_desk(dict(torso_pitch=0.1, head_pitch=0.45), writing=False)
THINK = hands_on_desk(dict(torso_pitch=0.12, head_pitch=0.1, head_yaw=0.3,     # chin on the left hand
                           l_shoulder_pitch=1.2, l_shoulder_roll=-0.26, l_shoulder_yaw=1.06, l_elbow=2.38),
                      writing=False, sides='r')
STRETCH = dict(torso_pitch=-0.15, head_pitch=-0.3, l_shoulder_pitch=2.9, r_shoulder_pitch=2.9,
               l_shoulder_roll=0.2, r_shoulder_roll=0.2, l_elbow=0.3, r_elbow=0.3)
SCRATCH = hands_on_desk(dict(head_pitch=0.1, head_yaw=-0.1,
                             r_shoulder_pitch=2.3, r_shoulder_roll=0.6, r_elbow=2.0), writing=False, sides='l')
WRITING_WIGGLE = {'r_shoulder_yaw': (0.05, 3.0), 'r_elbow': (0.02, 2.2), 'head_pitch': (0.02, 0.3)}


def look_up(yaw):
    return hands_on_desk(dict(torso_pitch=-0.05, head_pitch=-0.2, head_yaw=yaw), writing=False)


def glance(yaw):
    return hands_on_desk(dict(head_pitch=0.2, head_yaw=yaw), writing=False)


def peek(d):
    return hands_on_desk(dict(torso_pitch=0.1, torso_roll=0.18 * d, torso_yaw=0.2 * d,
                              head_yaw=0.95 * d, head_pitch=0.3))


def phone():
    # Held flat in both hands just above the desk edge, head down looking at it.
    pose = dict(torso_pitch=0.1, head_pitch=0.95, phone_slide=1.0)
    pose = arm_to(pose, 'l', (0.24, 0.08, 0.83), (0.08, 0.24, 0.78))
    return arm_to(pose, 'r', (0.24, -0.08, 0.83), (0.08, -0.24, 0.78))


def cheat_sheet(looking):
    # Sheet on the desk left of the exam paper, left hand beside it, right hand writing.
    pose = dict(torso_pitch=0.2, head_pitch=0.75, head_yaw=0.55) if looking else \
        dict(torso_pitch=0.15, head_pitch=0.55)
    pose = arm_to(dict(pose, sheet_slide=1.0), 'r', PEN_HAND, ELBOWS['r'])
    return arm_to(pose, 'l', (0.40, 0.34, HAND_ON_DESK_Z), ELBOWS['l'])


def whisper(d):
    return hands_on_desk(dict(torso_roll=0.1 * d, torso_yaw=0.3 * d, head_yaw=1.0 * d, head_pitch=0.05),
                         writing=False)


def turn_back(s):
    # Arm on the turning side hooks over the chair back; the other hand stays on the desk.
    pose = dict(torso_yaw=0.9 * s, torso_roll=-0.05 * s, head_yaw=1.3 * s, head_pitch=-0.05)
    pose = with_arm(pose, side_of(s), shoulder_pitch=-0.4, shoulder_roll=0.25, elbow=0.4)
    return hands_on_desk(pose, writing=False, sides=side_of(-s))


def reach(d, note_shown):
    # Lean towards the neighbour; hands meet low between the two seats, behind the desks.
    side = side_of(d)
    pose = dict(torso_roll=0.3 * d, torso_yaw=0.15 * d, head_yaw=0.6 * d, head_pitch=0.2,
                **{f'{side}_note_slide': note_shown})
    pose = arm_to(pose, side, (0.12, 0.56 * d, 0.74))
    return hands_on_desk(pose, writing=False, sides=side_of(-d))


def stow_note(side):
    return with_arm(dict(WRITE, head_pitch=0.5, **{f'{side}_note_slide': 1.0}), side, **LAP_ARM)


NORMAL = {'writing': 0.42, 'reading': 0.2, 'thinking': 0.1, 'looking_up': 0.1,
          'glancing_around': 0.1, 'stretching': 0.05, 'scratching_head': 0.03}
CHEATING = {'peeking': 0.28, 'phone': 0.2, 'cheat_sheet': 0.2, 'passing_note': 0.12,
            'whispering': 0.12, 'turning_around': 0.08}


def smoothstep(a):
    a = min(max(a, 0.0), 1.0)
    return a * a * (3.0 - 2.0 * a)


class Student:

    def __init__(self, row, col, propensity, phase_offset):
        self.id = layout.student_id(row, col)
        self.row, self.col = row, col
        self.seat = layout.seat_position(row, col)
        self.neighbour = layout.student_id(row, layout.neighbour_col(col))
        self.towards_neighbour = 1 if col % 2 == 0 else -1   # +y if the neighbour is on the left
        self.propensity = propensity
        self.phase_offset = phase_offset
        self.pose = {j: 0.0 for j in rig.JOINT_NAMES}
        self.from_pose = dict(self.pose)
        self.keys = []
        self.key_start = 0.0
        self.behavior = ''
        self.cheating = False
        self.partner = ''
        self.behavior_start = 0.0


class StudentBehaviors(Node):

    def __init__(self):
        super().__init__('student_behaviors')
        self.cheat_probability = self.declare_parameter('cheat_probability', 0.1).value
        rate = self.declare_parameter('rate_hz', 30.0).value
        seed = self.declare_parameter('seed', -1).value
        self.rng = random.Random(None if seed < 0 else seed)

        # A few students cheat a lot, most rarely: lognormal propensity with mean 1.
        self.students = [
            Student(r, c, self.rng.lognormvariate(0.0, 0.8) / math.exp(0.32), self.rng.uniform(0, 10))
            for r in range(layout.ROWS) for c in range(layout.COLS)]
        self.by_id = {s.id: s for s in self.students}
        # Link poses go straight to Gazebo; ROS has no message for a list of named poses.
        self.gz_node = GzNode()
        self.link_pub = self.gz_node.advertise(
            self.declare_parameter('link_pose_topic', '/classroom/student_link_poses').value, Pose_V)
        self.truth_pub = self.create_publisher(ClassroomState, '/classroom/ground_truth', 10)
        self.triggers = []
        self.create_subscription(String, '/classroom/trigger', self.on_trigger, 100)   # room for a whole class
        self.create_timer(1.0 / rate, self.step)
        self.get_logger().info(
            f'animating {len(self.students)} students, cheat_probability={self.cheat_probability}')

    def on_trigger(self, msg):
        parts = msg.data.split()
        if len(parts) != 2 or parts[0] not in self.by_id or parts[1] not in {**NORMAL, **CHEATING}:
            self.get_logger().warn(
                f"ignoring trigger '{msg.data}': expected '<student_id> <behavior>' with behavior one of "
                f"{', '.join([*NORMAL, *CHEATING])}")
            return
        self.triggers.append(parts)

    # ------------------------------------------------------------ episodes
    def assign(self, student, behavior, keys, now, cheating=False, partner=''):
        student.behavior, student.keys, student.cheating, student.partner = behavior, keys, cheating, partner
        student.from_pose = dict(student.pose)
        student.key_start = student.behavior_start = now
        if cheating:
            self.get_logger().info(
                f'{student.id}: {behavior}' + (f' (with {partner})' if partner else ''))

    def normal_keys(self, name):
        u = self.rng.uniform
        return {
            'writing': [key('writing', WRITE, 0.7, u(4, 12), WRITING_WIGGLE)],
            'reading': [key('reading', READ, 0.7, u(3, 8), {'head_pitch': (0.03, 0.2)})],
            'thinking': [key('thinking', THINK, 0.9, u(2, 6))],
            'looking_up': [key('looking_up', look_up(u(-0.3, 0.3)), 0.6, u(1.5, 4))],
            'glancing_around': [key('glancing_around', glance(self.rng.choice((-1, 1)) * u(0.4, 0.9)),
                                    0.35, u(0.5, 1.2))],
            'stretching': [key('stretching', STRETCH, 1.0, u(1.2, 2.5))],
            'scratching_head': [key('scratching_head', SCRATCH, 0.6, u(1, 2), {'r_elbow': (0.15, 3.0)})],
        }[name]

    def start_cheating(self, s, now, name=None):
        u, d = self.rng.uniform, s.towards_neighbour
        mate = self.by_id[s.neighbour]
        if name is None:
            name = self.rng.choices(list(CHEATING), list(CHEATING.values()))[0]
            if name in ('passing_note', 'whispering') and mate.cheating:
                name = 'peeking'
        if name == 'turning_around' and s.row == layout.ROWS - 1:
            name = 'phone'

        if name == 'peeking':
            keys = []
            for i in range(self.rng.randint(2, 4)):
                if i:
                    keys.append(key('writing', WRITE, 0.45, u(0.8, 2.0), WRITING_WIGGLE))
                keys.append(key('peeking', peek(d), 0.45, u(1.0, 2.5), cheating_visible=True))
            self.assign(s, name, keys, now, True, s.neighbour)
        elif name == 'phone':
            keys = [key('using_phone', phone(), 0.8, u(3, 8),
                        {'head_pitch': (0.03, 0.5), 'r_elbow': (0.04, 4.0)}, True)]
            self.assign(s, name, keys, now, True)
        elif name == 'cheat_sheet':
            keys = []
            for _ in range(self.rng.randint(2, 3)):
                keys.append(key('reading_sheet', cheat_sheet(True), 0.7, u(1.0, 2.5), cheating_visible=True))
                keys.append(key('writing', cheat_sheet(False), 0.5, u(1.0, 2.5), WRITING_WIGGLE, True))
            self.assign(s, name, keys, now, True)
        elif name == 'passing_note':
            giver, receiver = (s, mate) if self.rng.random() < 0.5 else (mate, s)
            gd = giver.towards_neighbour
            self.assign(giver, name, [
                key('reaching', reach(gd, 1.0), 1.0, 0.6, cheating_visible=True),
                key('handing_over', reach(gd, 0.0), 0.3, 0.5, cheating_visible=True),
                key('returning', WRITE, 0.9, 0.8, cheating_visible=True),
            ], now, True, receiver.id)
            self.assign(receiver, name, [
                key('reaching', reach(-gd, 0.0), 1.0, 0.6, cheating_visible=True),
                key('handing_over', reach(-gd, 1.0), 0.3, 0.5, cheating_visible=True),
                key('hiding_note', stow_note(side_of(-gd)), 0.9, 0.8, cheating_visible=True),
            ], now, True, giver.id)
        elif name == 'whispering':
            hold = u(2, 5)
            chatter = {'head_pitch': (0.07, 2.5)}
            self.assign(s, name, [key('whispering', whisper(d), 0.5, hold, chatter, True)], now, True, mate.id)
            self.assign(mate, name, [key('whispering', whisper(-d), 0.5, hold, chatter, True)],
                        now, True, s.id)
        else:  # turning_around, towards the student in the row behind
            behind = layout.student_id(s.row + 1, s.col)
            keys = [key('turning_around', turn_back(self.rng.choice((-1, 1))), 0.8, u(1.5, 3),
                        {'head_pitch': (0.05, 2.0)}, True)]
            self.assign(s, name, keys, now, True, behind)

    # ------------------------------------------------------------ loop
    def step(self):
        if not rclpy.ok():
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        for s in self.students:
            while s.keys and now >= s.key_start + s.keys[0].move + s.keys[0].hold:
                done = s.keys.pop(0)
                s.key_start += done.move + done.hold
                s.from_pose = dict(s.pose)
            if not s.keys:
                s.key_start = now
        for student_id, name in self.triggers:
            s = self.by_id[student_id]
            if name in CHEATING:
                self.start_cheating(s, now, name)
            else:
                self.assign(s, name, self.normal_keys(name), now)
        self.triggers.clear()
        for s in self.students:
            if s.keys:
                continue
            if self.rng.random() < self.cheat_probability * s.propensity:
                self.start_cheating(s, now)
            else:
                name = self.rng.choices(list(NORMAL), list(NORMAL.values()))[0]
                self.assign(s, name, self.normal_keys(name), now)

        links = Pose_V()
        truth = ClassroomState()
        truth.header.stamp = self.get_clock().now().to_msg()
        truth.header.frame_id = 'odom'
        for s in self.students:
            k = s.keys[0]
            t = now - s.key_start
            a = smoothstep(t / k.move)
            joints = {}
            for j in rig.JOINT_NAMES:
                value = s.from_pose[j] + (k.pose.get(j, 0.0) - s.from_pose[j]) * a
                if j in k.wiggle:
                    amplitude, frequency = k.wiggle[j]
                    value += amplitude * a * math.sin(2 * math.pi * frequency * t + s.phase_offset)
                s.pose[j] = value
                lower, upper = rig.LIMITS[j]
                joints[j] = value * upper if j in rig.PROPS else min(max(value, lower), upper)

            model = rig.model_name(s.id)
            for link, (position, orientation) in rig.forward_kinematics(joints).items():
                if link == 'pelvis':
                    continue
                pose = links.pose.add()
                pose.name = f'{model}::{link}'
                pose.position.x, pose.position.y, pose.position.z = position
                pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = orientation

            state = StudentState(id=s.id, row=s.row, col=s.col, behavior=s.behavior, phase=k.phase,
                                 is_cheating=s.cheating, cheating_visible=k.cheating_visible,
                                 partner_id=s.partner)
            state.seat.x, state.seat.y = s.seat
            state.behavior_start = Time(nanoseconds=int(s.behavior_start * 1e9)).to_msg()
            truth.students.append(state)

        self.link_pub.publish(links)
        self.truth_pub.publish(truth)


def main():
    rclpy.init()
    node = StudentBehaviors()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise   # a timer firing mid-shutdown can fail to publish; only real errors matter
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
