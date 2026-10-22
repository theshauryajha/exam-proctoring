#!/usr/bin/env python3
"""Random patrol through the classroom while the camera stays on the desks.

The node tracks when each student was last in the camera's view ("in view":
inside the field of view and range; occlusion by other students is not modelled).

Motion: the aisles and cross corridors form a grid graph (see classroom_layout).
The robot random-walks that graph with random leg lengths, speeds and pauses,
so its path never cuts through the desks. With strategy 'coverage' (default)
it may reverse at an intersection (backtrack_probability), so passing a student
says nothing about when it returns; route_bias can also weight legs towards
students who have gone unseen longest. Strategy 'random' is a plain random walk
that never reverses.

Gaze: independently of driving, the pan/tilt head picks a desk to look at,
holds it for a few seconds once acquired, then moves on. 'coverage' aims where
the students in view have gone unseen longest (gaze_temperature > 0 samples
views instead; recheck_probability adds random re-checks). 'random' picks a
nearby desk at random. The gaze direction is held in the world frame, so it
stays locked while the base turns.

Defaults come from offline replays of this node (2 simulated hours each) against
students who wait for the best moment to cheat: aiming at the most overdue view
was in view for 23% of that cheating, sampled views with re-checks only 17%, and
reversing at half the intersections raised it to 25%. See the README.
"""
import math
import random

from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64

from exam_proctor_sim import classroom_layout as layout


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class ProctorPatrol(Node):

    def __init__(self, **kwargs):
        super().__init__('proctor_patrol', **kwargs)
        param = self.declare_parameter
        self.min_speed = param('min_linear_speed', 0.15).value
        self.max_speed = param('max_linear_speed', 0.35).value
        self.max_turn = param('max_angular_speed', 1.0).value
        self.max_leg_nodes = param('max_leg_nodes', 3).value
        self.pause_prob = param('pause_probability', 0.35).value
        self.pause_range = (param('pause_min_s', 2.0).value, param('pause_max_s', 6.0).value)
        self.dwell_range = (param('gaze_dwell_min_s', 2.5).value,
                            param('gaze_dwell_max_s', 6.0).value)
        self.gaze_min_dist = param('gaze_min_distance', 1.2).value
        self.gaze_max_dist = param('gaze_max_distance', 5.0).value
        self.camera_height = param('camera_height', 1.50).value
        self.pan_rate = param('pan_rate', 1.5).value
        self.tilt_rate = param('tilt_rate', 0.8).value
        self.goal_tolerance = param('goal_tolerance', 0.08).value
        self.strategy = param('strategy', 'coverage').value
        self.view_half_hfov = param('view_half_hfov', 0.45).value    # camera half FOV is 0.6 rad, less a margin
        self.view_half_vfov = param('view_half_vfov', 0.28).value
        self.view_max_dist = param('view_max_distance', 6.0).value
        self.stale_cap = param('stale_cap_s', 90.0).value
        self.turn_cost = param('turn_cost', 0.3).value
        self.gaze_temperature = param('gaze_temperature', 0.0).value     # 0: always the most overdue view
        self.recheck_prob = param('recheck_probability', 0.0).value
        self.backtrack_prob = param('backtrack_probability', 0.5).value
        self.route_bias = param('route_bias', 0.0).value
        self.route_radius = param('route_radius', 3.5).value
        if self.strategy not in ('coverage', 'random'):
            raise ValueError(f"strategy must be 'coverage' or 'random', not {self.strategy!r}")
        seed = param('seed', -1).value
        # Separate generators: the route depends only on the seed, whatever the gaze strategy.
        self.rng = random.Random(None if seed < 0 else seed)
        self.gaze_rng = random.Random(None if seed < 0 else seed + 1)

        layout.check_clearance()
        self.xs = layout.corridor_xs()
        self.ys = layout.aisle_ys()
        self.desks = layout.desks()
        self.student_ids = [layout.student_id(r, c) for r, c, _, _ in self.desks]
        self.last_seen = [-math.inf] * len(self.desks)   # sim time each student was last in view
        self.last_report = None
        self.node_desks = {
            (i, j): [k for k, d in enumerate(self.desks)
                     if math.dist((x, y), (d[2], d[3])) <= self.route_radius]
            for i, x in enumerate(self.xs) for j, y in enumerate(self.ys)}

        self.pose = None            # (x, y, yaw) in the odom (= world) frame
        self.node = None            # graph node (i, j) the robot last reached
        self.prev_node = None
        self.leg = []               # queued graph nodes to drive through
        self.speed = self.min_speed
        self.pause_until = None
        self.gaze_desk = None
        self.gaze_until = None      # set once the camera is on the desk
        self.gaze_bearing = None    # world-frame camera heading
        self.pan_cmd = 0.0
        self.tilt_cmd = 0.3
        self.last_time = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pan_pub = self.create_publisher(Float64, '/proctor/camera/pan_cmd', 10)
        self.tilt_pub = self.create_publisher(Float64, '/proctor/camera/tilt_cmd', 10)
        self.focus_pub = self.create_publisher(PointStamped, '/proctor/focus_target', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_timer(0.05, self.step)
        self.get_logger().info(
            f'patrolling a {len(self.xs)}x{len(self.ys)} aisle grid around {len(self.desks)} desks')

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def step(self):
        if self.pose is None or not rclpy.ok():
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self.last_time is None else clamp(now - self.last_time, 0.0, 0.2)
        self.last_time = now
        self.drive(now)
        self.aim_camera(now, dt)
        self.update_coverage(now)
        self.report_coverage(now)

    # ------------------------------------------------------------ motion
    def node_xy(self, node):
        return self.xs[node[0]], self.ys[node[1]]

    def on_grid(self, i, j):
        return 0 <= i < len(self.xs) and 0 <= j < len(self.ys)

    def overdue_near(self, grid_node, now):
        """Mean staleness (0..1) of the students around a grid node."""
        ages = [min(now - self.last_seen[k], self.stale_cap) / self.stale_cap
                for k in self.node_desks[grid_node]]
        return sum(ages) / len(ages) if ages else 0.0

    def plan_leg(self, now):
        i, j = self.node
        moves = [(di, dj) for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)) if self.on_grid(i + di, j + dj)]
        may_reverse = self.strategy == 'coverage' and self.rng.random() < self.backtrack_prob
        if self.prev_node is not None and len(moves) > 1 and not may_reverse:
            back = (self.prev_node[0] - i, self.prev_node[1] - j)
            moves = [m for m in moves if m != back]
        if self.strategy == 'random':
            di, dj = self.rng.choice(moves)
            steps = 1
            while steps < self.max_leg_nodes and self.on_grid(i + di * (steps + 1), j + dj * (steps + 1)):
                steps += 1
            steps = self.rng.randint(1, steps)
        else:
            # Any direction and length, weighted towards where students have gone unseen.
            options = [(di, dj, k) for di, dj in moves for k in range(1, self.max_leg_nodes + 1)
                       if self.on_grid(i + di * k, j + dj * k)]
            weights = [1.0 + self.route_bias * self.overdue_near((i + di * k, j + dj * k), now)
                       for di, dj, k in options]
            di, dj, steps = self.rng.choices(options, weights)[0]
        self.leg = [(i + di * k, j + dj * k) for k in range(1, steps + 1)]
        self.speed = self.rng.uniform(self.min_speed, self.max_speed)

    def drive(self, now):
        if self.pause_until is not None:
            if now < self.pause_until:
                self.cmd_pub.publish(Twist())
                return
            self.pause_until = None

        if self.node is None and not self.leg:
            # First drive to the nearest graph node (the robot should spawn on one).
            x, y, _ = self.pose
            nearest = min(((i, j) for i in range(len(self.xs)) for j in range(len(self.ys))),
                          key=lambda n: math.dist((x, y), self.node_xy(n)))
            self.leg = [nearest]
        elif not self.leg:
            self.plan_leg(now)

        x, y, yaw = self.pose
        gx, gy = self.node_xy(self.leg[0])
        dist = math.hypot(gx - x, gy - y)
        if dist < self.goal_tolerance:
            self.prev_node, self.node = self.node, self.leg.pop(0)
            if not self.leg:
                self.cmd_pub.publish(Twist())
                if self.rng.random() < self.pause_prob:
                    self.pause_until = now + self.rng.uniform(*self.pause_range)
            return

        heading_error = wrap(math.atan2(gy - y, gx - x) - yaw)
        cmd = Twist()
        cmd.angular.z = clamp(2.5 * heading_error, -self.max_turn, self.max_turn)
        if abs(heading_error) < 0.3:
            # Only translate when roughly aligned, so the robot stays on the aisle centreline.
            cmd.linear.x = min(self.speed, 0.6 * dist + 0.05) * math.cos(heading_error)
        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------ camera
    @staticmethod
    def target_point(desk):
        """Where the camera aims for a desk: between the desk top and the student's torso."""
        return desk[2] - 0.25, desk[3], 0.9

    def tilt_towards(self, x, y, tx, ty, tz):
        return clamp(math.atan2(self.camera_height - tz, max(math.hypot(tx - x, ty - y), 0.3)), -0.4, 1.2)

    def in_view(self, x, y, bearing, tilt, desk):
        """Whether a student is inside the camera's field of view and range (occlusion ignored)."""
        sx, sy, sz = desk[2] - 0.35, desk[3], 0.95     # student's torso
        dist = math.hypot(sx - x, sy - y)
        if not 0.5 <= dist <= self.view_max_dist:
            return False
        elevation = math.atan2(self.camera_height - sz, dist)
        return (abs(wrap(math.atan2(sy - y, sx - x) - bearing)) <= self.view_half_hfov
                and abs(elevation - tilt) <= self.view_half_vfov)

    def pick_gaze_desk(self, now):
        x, y, _ = self.pose
        by_dist = sorted(self.desks, key=lambda d: math.dist((x, y), (d[2], d[3])))
        candidates = [d for d in by_dist
                      if self.gaze_min_dist <= math.dist((x, y), (d[2], d[3])) <= self.gaze_max_dist
                      and d is not self.gaze_desk] or by_dist[:3]
        if self.strategy == 'random':
            # Favour desks near the current view so the camera sweeps rather than spins.
            weights = [1.2 + math.cos(math.atan2(d[3] - y, d[2] - x) - self.gaze_bearing)
                       for d in candidates]
            self.gaze_desk = self.gaze_rng.choices(candidates, weights)[0]
        elif self.gaze_rng.random() < self.recheck_prob:
            # Optional re-check that ignores history.
            self.gaze_desk = self.gaze_rng.choice(candidates)
        else:
            def score(desk):
                tx, ty, tz = self.target_point(desk)
                bearing = math.atan2(ty - y, tx - x)
                tilt = self.tilt_towards(x, y, tx, ty, tz)
                # Each student in that view counts up to 1, the longer unseen the more.
                overdue = sum(min(now - seen, self.stale_cap) / self.stale_cap
                              for d, seen in zip(self.desks, self.last_seen)
                              if self.in_view(x, y, bearing, tilt, d))
                turn = abs(wrap(bearing - self.gaze_bearing))
                return overdue - self.turn_cost * turn
            scores = [score(d) for d in candidates]
            best = max(scores)
            if self.gaze_temperature <= 0.0:
                self.gaze_desk = candidates[scores.index(best)]
            else:
                weights = [math.exp((sc - best) / self.gaze_temperature) for sc in scores]
                self.gaze_desk = self.gaze_rng.choices(candidates, weights)[0]
        self.gaze_until = None

    def update_coverage(self, now):
        x, y, _ = self.pose
        for i, desk in enumerate(self.desks):
            if self.in_view(x, y, self.gaze_bearing, self.tilt_cmd, desk):
                self.last_seen[i] = now

    def report_coverage(self, now, period=30.0, window=60.0):
        if self.last_report is None:
            self.last_report = now
        if now - self.last_report < period:
            return
        self.last_report = now
        recent = sum(now - seen <= window for seen in self.last_seen)
        stalest = max(range(len(self.desks)), key=lambda i: now - self.last_seen[i])
        age = now - self.last_seen[stalest]
        self.get_logger().info(
            f'gaze coverage ({self.strategy}): {recent}/{len(self.desks)} students in view in the '
            f'last {window:.0f} s; longest unseen {self.student_ids[stalest]} '
            f'({"never" if math.isinf(age) else f"{age:.0f} s"})')

    def aim_camera(self, now, dt):
        x, y, yaw = self.pose
        if self.gaze_bearing is None:
            self.gaze_bearing = yaw
        if self.gaze_desk is None or (self.gaze_until is not None and now >= self.gaze_until):
            self.pick_gaze_desk(now)

        tx, ty, tz = self.target_point(self.gaze_desk)
        dx, dy = tx - x, ty - y

        step = self.pan_rate * dt
        bearing_error = wrap(math.atan2(dy, dx) - self.gaze_bearing)
        self.gaze_bearing += clamp(bearing_error, -step, step)
        if self.gaze_until is None and abs(bearing_error) < 0.05:
            self.gaze_until = now + self.gaze_rng.uniform(*self.dwell_range)
        # The pan joint is continuous: move the command the short way round.
        self.pan_cmd += wrap(self.gaze_bearing - yaw - self.pan_cmd)

        tilt_goal = self.tilt_towards(x, y, tx, ty, tz)
        step = self.tilt_rate * dt
        self.tilt_cmd += clamp(tilt_goal - self.tilt_cmd, -step, step)

        self.pan_pub.publish(Float64(data=self.pan_cmd))
        self.tilt_pub.publish(Float64(data=self.tilt_cmd))

        focus = PointStamped()
        focus.header.stamp = self.get_clock().now().to_msg()
        focus.header.frame_id = 'odom'
        focus.point.x, focus.point.y, focus.point.z = tx, ty, tz
        self.focus_pub.publish(focus)


def main():
    rclpy.init()
    node = ProctorPatrol()
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
