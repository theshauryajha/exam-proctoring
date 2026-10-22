"""Seated-student rig, shared by the world generator and the behaviour node.

Student model frame: origin on the floor at the chair centre, +x towards the desk.
The rig is a tree of links connected by (virtual) joints. At the zero pose every
link frame is parallel to the model frame and each joint sits at its child link's
origin. Gazebo models are static and jointless; student_behaviors runs
forward_kinematics() and streams the resulting link poses to Gazebo.

Positive joint values mean:
  torso_pitch lean forward      torso_roll lean to +y (left)    torso_yaw turn to +y
  head_yaw    turn to +y        head_pitch look down
  *_shoulder_pitch raise arm forward   *_shoulder_roll raise arm outwards
  *_shoulder_yaw rotate upper arm inwards (swings a bent forearm towards the body)
  *_elbow bend forearm forward
Props are prismatic joints that slide a hidden object out of the body; the
behaviour node commands them as 0 (hidden) .. 1 (shown).
"""
import math
from collections import namedtuple

HIP_Z = 0.52
NECK_Z = 1.05
SHOULDER_Z = 0.98
SHOULDER_Y = 0.22
UPPER_ARM = 0.27
FOREARM = 0.24

Joint = namedtuple('Joint', 'name type parent child origin axis lower upper')


def _arm(side, s):
    y = s * SHOULDER_Y
    return [
        Joint(f'{side}_shoulder_pitch', 'revolute', 'torso', f'{side}_shoulder',
              (0, y, SHOULDER_Z), (0, -1, 0), -0.6, 3.1),
        Joint(f'{side}_shoulder_roll', 'revolute', f'{side}_shoulder', f'{side}_shoulder_roll_link',
              (0, y, SHOULDER_Z), (s, 0, 0), -0.8, 1.8),
        Joint(f'{side}_shoulder_yaw', 'revolute', f'{side}_shoulder_roll_link', f'{side}_upper_arm',
              (0, y, SHOULDER_Z), (0, 0, -s), -0.8, 1.5),
        Joint(f'{side}_elbow', 'revolute', f'{side}_upper_arm', f'{side}_forearm',
              (0, y, SHOULDER_Z - UPPER_ARM), (0, -1, 0), 0.0, 2.6),
        Joint(f'{side}_note_slide', 'prismatic', f'{side}_forearm', f'{side}_note',
              (0, y, SHOULDER_Z - UPPER_ARM - FOREARM), (0, 0, -1), 0.0, 0.05),
    ]


JOINTS = [
    Joint('torso_pitch', 'revolute', 'pelvis', 'torso_pitch_link', (0, 0, HIP_Z), (0, 1, 0), -0.5, 0.8),
    Joint('torso_roll', 'revolute', 'torso_pitch_link', 'torso_roll_link', (0, 0, HIP_Z), (-1, 0, 0), -0.5, 0.5),
    Joint('torso_yaw', 'revolute', 'torso_roll_link', 'torso', (0, 0, HIP_Z), (0, 0, 1), -1.0, 1.0),
    Joint('head_yaw', 'revolute', 'torso', 'head_yaw_link', (0, 0, NECK_Z), (0, 0, 1), -1.4, 1.4),
    Joint('head_pitch', 'revolute', 'head_yaw_link', 'head', (0, 0, NECK_Z), (0, 1, 0), -0.5, 1.0),
    *_arm('l', 1),
    *_arm('r', -1),
    # The phone hides inside the right thigh and rises to (0.24, 0.0, 0.84): held flat, screen up,
    # just behind the desk edge and above the desk top, where the robot camera can see it.
    Joint('phone_slide', 'prismatic', 'pelvis', 'phone', (0.18, -0.10, 0.53), (0.1811, 0.3018, 0.9357), 0.0, 0.3313),
    # The cheat sheet hides inside the desk top and rises 2.4 cm to lie on it, left of the exam paper.
    Joint('sheet_slide', 'prismatic', 'pelvis', 'cheat_sheet', (0.36, 0.22, 0.732), (0, 0, 1), 0.0, 0.024),
]

JOINT_NAMES = [j.name for j in JOINTS]
PROPS = {j.name for j in JOINTS if j.type == 'prismatic'}
LIMITS = {j.name: (j.lower, j.upper) for j in JOINTS}


ORIGINS = {'pelvis': (0.0, 0.0, 0.0), **{j.child: j.origin for j in JOINTS}}


def model_name(student_id):
    return f'student_{student_id}'


def _quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def _rotate(q, v):
    w, x, y, z = q
    cx, cy, cz = y * v[2] - z * v[1], z * v[0] - x * v[2], x * v[1] - y * v[0]
    return (v[0] + 2 * (w * cx + y * cz - z * cy),
            v[1] + 2 * (w * cy + z * cx - x * cz),
            v[2] + 2 * (w * cz + x * cy - y * cx))


def forward_kinematics(values):
    """{joint: value} -> {link: ((x, y, z), (qw, qx, qy, qz))} in the model frame.

    Revolute values are angles [rad], prismatic values distances [m]; missing joints are 0.
    """
    poses = {'pelvis': ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))}
    for j in JOINTS:   # parents always come before children
        parent_pos, parent_rot = poses[j.parent]
        value = values.get(j.name, 0.0)
        offset = [c - p for c, p in zip(j.origin, ORIGINS[j.parent])]
        if j.type == 'revolute':
            half = value / 2.0
            s = math.sin(half)
            rot = _quat_mul(parent_rot, (math.cos(half), j.axis[0] * s, j.axis[1] * s, j.axis[2] * s))
        else:
            offset = [o + a * value for o, a in zip(offset, j.axis)]
            rot = parent_rot
        d = _rotate(parent_rot, offset)
        poses[j.child] = (tuple(p + e for p, e in zip(parent_pos, d)), rot)
    return poses


def hand_position(poses, side):
    """Hand centre from forward_kinematics() output (the note prop can slide out of the hand)."""
    position, rotation = poses[f'{side}_forearm']
    return tuple(p + d for p, d in zip(position, _rotate(rotation, (0.0, 0.0, -FOREARM))))


def _solve_linear(a, b):
    n = len(b)
    m = [list(row) + [b[i]] for i, row in enumerate(a)]
    for c in range(n):
        pivot = max(range(c, n), key=lambda r: abs(m[r][c]))
        m[c], m[pivot] = m[pivot], m[c]
        for r in range(c + 1, n):
            f = m[r][c] / m[c][c]
            for k in range(c, n + 1):
                m[r][k] -= f * m[c][k]
    x = [0.0] * n
    for r in reversed(range(n)):
        x[r] = (m[r][n] - sum(m[r][k] * x[k] for k in range(r + 1, n))) / m[r][r]
    return x


ARM_JOINTS = ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow')


UPPER_ARM_RADIUS = 0.042
FOREARM_RADIUS = 0.036
HAND_RADIUS = 0.045


def solve_arm(values, side, hand, elbow=None, elbow_weight=0.3, avoid=None, avoid_weight=3.0,
              seed=(0.8, 0.0, 0.0, 1.0)):
    """Arm joint values putting the hand centre at `hand` (and the elbow near `elbow`).

    Targets are in the model frame; the rest of the body is posed by `values`. `avoid` is an
    optional horizontal surface (x0, x1, y0, y1, top), e.g. the desk: arm segments over it are
    pushed above it. Damped least squares within joint limits; returns {joint: value} for the
    four arm joints.
    """
    names = [f'{side}_{j}' for j in ARM_JOINTS]
    q = dict(values, **dict(zip(names, seed)))

    def below_surface(point, radius):
        x0, x1, y0, y1, top = avoid
        x, y, z = point
        if x0 - radius < x < x1 + radius and y0 - radius < y < y1 + radius:
            return max(0.0, top + radius - z)
        return 0.0

    def residual(q):
        poses = forward_kinematics(q)
        hand_at = hand_position(poses, side)
        r = [p - t for p, t in zip(hand_at, hand)]
        elbow_at = poses[f'{side}_forearm'][0]
        if elbow is not None:
            r += [elbow_weight * (p - t) for p, t in zip(elbow_at, elbow)]
        if avoid is not None:
            shoulder_at = poses[f'{side}_upper_arm'][0]
            for f in (0.0, 0.25, 0.5, 0.75, 1.0):
                r.append(avoid_weight * below_surface(
                    [a + f * (b - a) for a, b in zip(shoulder_at, elbow_at)], UPPER_ARM_RADIUS))
                r.append(avoid_weight * below_surface(
                    [a + f * (b - a) for a, b in zip(elbow_at, hand_at)], FOREARM_RADIUS))
            r.append(avoid_weight * below_surface(hand_at, HAND_RADIUS))
        return r

    for _ in range(100):
        r = residual(q)
        jac = []
        for name in names:
            dq = dict(q)
            dq[name] += 1e-5
            jac.append([(a - b) / 1e-5 for a, b in zip(residual(dq), r)])
        jtj = [[sum(ja * jb for ja, jb in zip(jac[a], jac[b])) + (1e-3 if a == b else 0.0)
                for b in range(len(names))] for a in range(len(names))]
        jtr = [sum(ja * ri for ja, ri in zip(jac[a], r)) for a in range(len(names))]
        step = _solve_linear(jtj, jtr)
        for name, delta in zip(names, step):
            lower, upper = LIMITS[name]
            q[name] = min(max(q[name] - delta, lower), upper)
        if max(abs(d) for d in step) < 1e-5:
            break
    return {name: q[name] for name in names}
