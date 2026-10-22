#!/usr/bin/env python3
"""Generate the classroom SDF world from classroom_layout and student_rig.

Usage: ros2 run exam_proctor_sim generate_world [output.sdf]
(default output: worlds/classroom.sdf in the source tree, found relative to this file)
"""
import math
import os
import random
import sys

from exam_proctor_sim import classroom_layout as L
from exam_proctor_sim import student_rig as R

WOOD = (0.55, 0.38, 0.22)
METAL = (0.35, 0.35, 0.38)
CHAIR_BLUE = (0.15, 0.25, 0.55)
PAPER = (0.97, 0.97, 0.95)
SHIRTS = [(0.80, 0.15, 0.15), (0.15, 0.45, 0.75), (0.20, 0.60, 0.25), (0.90, 0.75, 0.20),
          (0.55, 0.25, 0.65), (0.95, 0.95, 0.95), (0.20, 0.20, 0.22), (0.95, 0.50, 0.15)]
PANTS = [(0.15, 0.18, 0.35), (0.25, 0.25, 0.25), (0.40, 0.32, 0.22), (0.10, 0.10, 0.12)]
SKIN = [(0.96, 0.80, 0.69), (0.87, 0.67, 0.50), (0.72, 0.52, 0.36), (0.55, 0.38, 0.26),
        (0.38, 0.26, 0.18)]
HAIR = [(0.08, 0.06, 0.05), (0.30, 0.18, 0.08), (0.60, 0.45, 0.25), (0.15, 0.12, 0.10)]
SHOES = (0.1, 0.08, 0.07)


def _pose(xyz, rpy=(0.0, 0.0, 0.0)):
    return '<pose>' + ' '.join(f'{v:.4f}' for v in (*xyz, *rpy)) + '</pose>'


def _material(rgb, emissive=None):
    r, g, b = rgb
    s = (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
         f'<specular>0.1 0.1 0.1 1</specular>')
    if emissive:
        s += f'<emissive>{emissive[0]} {emissive[1]} {emissive[2]} 1</emissive>'
    return s + '</material>'


def g_box(size):
    return '<box><size>' + ' '.join(f'{v:.4f}' for v in size) + '</size></box>'


def g_sphere(radius):
    return f'<sphere><radius>{radius}</radius></sphere>'


def g_cylinder(radius, length):
    return f'<cylinder><radius>{radius}</radius><length>{length}</length></cylinder>'


def g_capsule(radius, length):
    return f'<capsule><radius>{radius}</radius><length>{length}</length></capsule>'


def g_ellipsoid(a, b, c):
    return f'<ellipsoid><radii>{a} {b} {c}</radii></ellipsoid>'


def shape(name, geometry, xyz, rgb, collide=False, rpy=(0.0, 0.0, 0.0), emissive=None):
    pose = _pose(xyz, rpy)
    s = (f'<visual name="{name}">{pose}<geometry>{geometry}</geometry>'
         f'{_material(rgb, emissive)}</visual>')
    if collide:
        s += f'<collision name="{name}_col">{pose}<geometry>{geometry}</geometry></collision>'
    return s


def box(name, xyz, size, rgb, collide=True, rpy=(0.0, 0.0, 0.0)):
    return shape(name, g_box(size), xyz, rgb, collide, rpy)


def static_model(name, xyz, yaw, body):
    return (f'<model name="{name}"><static>true</static>{_pose(xyz, (0, 0, yaw))}'
            f'<link name="body">{body}</link></model>\n')


def desk_body(depth, width, height, rng=None):
    """Desk centred at the origin; the student sits on the -x side."""
    s = box('top', (0, 0, height - 0.02), (depth, width, 0.04), WOOD)
    lx, ly, lh = depth / 2 - 0.04, width / 2 - 0.05, height - 0.04
    for i, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
        s += box(f'leg{i}', (sx * lx, sy * ly, lh / 2), (0.04, 0.04, lh), METAL)
    s += box('panel', (depth / 2 - 0.03, 0, height * 0.62), (0.02, width - 0.12, 0.3), WOOD, False)
    if rng is not None:
        # Exam sheet and pen on the student's side of the desk.
        s += box('exam_paper', (-0.08, rng.uniform(-0.2, 0.2), height + 0.002),
                 (0.297, 0.21, 0.003), PAPER, False, (0, 0, rng.uniform(-0.25, 0.25)))
        s += shape('pen', g_cylinder(0.005, 0.14), (-0.05, rng.uniform(0.25, 0.4), height + 0.006),
                   (0.1, 0.1, 0.6), False, (0, math.pi / 2, rng.uniform(-0.6, 0.6)))
    return s


def chair_body(cx):
    """Chair centred at (cx, 0), backrest on the -x side."""
    half = L.CHAIR_SIZE / 2
    s = box('seat', (cx, 0, 0.45), (0.42, 0.42, 0.03), CHAIR_BLUE)
    s += box('back', (cx - half + 0.015, 0, 0.70), (0.03, 0.42, 0.45), CHAIR_BLUE)
    for i, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
        s += box(f'chair_leg{i}', (cx + sx * 0.18, sy * 0.18, 0.2175), (0.03, 0.03, 0.435), METAL)
    return s


def student_model(name, xy, rng):
    """Student (see student_rig) seated on the chair at xy, facing +x, in its zero pose."""
    shirt, pants = rng.choice(SHIRTS), rng.choice(PANTS)
    skin, hair = rng.choice(SKIN), rng.choice(HAIR)
    origins = R.ORIGINS
    visuals = {link: '' for link in origins}

    def add(link, part, geometry, xyz, rgb, rpy=(0.0, 0.0, 0.0), emissive=None):
        # Parts are placed in model coordinates; visual poses are relative to the link.
        o = origins[link]
        rel = (xyz[0] - o[0], xyz[1] - o[1], xyz[2] - o[2])
        visuals[link] += shape(part, geometry, rel, rgb, rpy=rpy, emissive=emissive)

    add('pelvis', 'hips', g_ellipsoid(0.13, 0.19, 0.09), (0.0, 0, 0.55), pants)
    for side, y in (('l', 0.10), ('r', -0.10)):
        add('pelvis', f'{side}_thigh', g_capsule(0.075, 0.40), (0.22, y, 0.53), pants, (0, math.pi / 2, 0))
        add('pelvis', f'{side}_shin', g_capsule(0.055, 0.36), (0.45, y, 0.28), pants)
        add('pelvis', f'{side}_foot', g_box((0.22, 0.09, 0.07)), (0.50, y, 0.035), SHOES)

    add('torso', 'chest', g_ellipsoid(0.12, 0.19, 0.25), (0, 0, 0.80), shirt)
    add('torso', 'shoulders', g_capsule(0.065, 0.36), (0, 0, 0.95), shirt, (math.pi / 2, 0, 0))
    # The neck belongs to the head so it tilts with it; on the torso it stuck out behind a lowered head.
    add('head', 'neck', g_cylinder(0.045, 0.12), (0.005, 0, 1.08), skin)

    add('head', 'skull', g_ellipsoid(0.095, 0.08, 0.115), (0.01, 0, 1.19), skin)
    add('head', 'hair', g_ellipsoid(0.10, 0.085, 0.08), (-0.015, 0, 1.235), hair)
    add('head', 'nose', g_box((0.03, 0.022, 0.035)), (0.105, 0, 1.18), tuple(c * 0.9 for c in skin))
    add('head', 'mouth', g_box((0.01, 0.04, 0.008)), (0.10, 0, 1.13), (0.45, 0.15, 0.15))
    for side, y in (('l', 0.035), ('r', -0.035)):
        add('head', f'{side}_eye', g_sphere(0.013), (0.09, y, 1.215), (0.05, 0.05, 0.05))

    for side, y in (('l', R.SHOULDER_Y), ('r', -R.SHOULDER_Y)):
        elbow_z = R.SHOULDER_Z - R.UPPER_ARM
        hand_z = elbow_z - R.FOREARM
        add(f'{side}_upper_arm', 'upper_arm', g_capsule(0.042, 0.20),
            (0, y, R.SHOULDER_Z - R.UPPER_ARM / 2), shirt)
        add(f'{side}_forearm', 'forearm', g_capsule(0.036, 0.19), (0, y, elbow_z - R.FOREARM / 2), skin)
        add(f'{side}_forearm', 'hand', g_sphere(0.045), (0, y, hand_z), skin)
        add(f'{side}_note', 'note', g_box((0.04, 0.05, 0.012)), (0, y, hand_z), (0.98, 0.95, 0.75))

    phone_x, phone_y, phone_z = origins['phone']
    add('phone', 'body', g_box((0.15, 0.075, 0.01)), (phone_x, phone_y, phone_z), (0.05, 0.05, 0.06))
    add('phone', 'screen', g_box((0.13, 0.065, 0.002)), (phone_x, phone_y, phone_z + 0.006),
        (0.2, 0.35, 0.6), emissive=(0.25, 0.45, 0.8))
    add('cheat_sheet', 'paper', g_box((0.15, 0.10, 0.003)), origins['cheat_sheet'], (0.98, 0.95, 0.75))

    # Static and jointless: the StudentAnimator system sets every link pose directly from the
    # forward kinematics computed by student_behaviors, so physics never simulates students.
    links = ''.join(f'<link name="{link}">{_pose(origin)}{visuals[link]}</link>'
                    for link, origin in origins.items())
    return f'<model name="{name}"><static>true</static>{_pose((xy[0], xy[1], 0))}{links}</model>\n'


def light(name, xyz):
    return (f'<light type="point" name="{name}">{_pose(xyz)}<cast_shadows>false</cast_shadows>'
            '<diffuse>0.55 0.55 0.52 1</diffuse><specular>0.1 0.1 0.1 1</specular>'
            '<attenuation><range>12</range><constant>0.4</constant><linear>0.05</linear>'
            '<quadratic>0.01</quadratic></attenuation></light>\n')


def build_world(seed=7):
    L.check_clearance()
    rng = random.Random(seed)
    hx, hy, t, h = L.ROOM_X / 2, L.ROOM_Y / 2, L.WALL_T, L.WALL_H
    wall = (0.85, 0.84, 0.78)

    out = ['<?xml version="1.0"?>\n<!-- Generated by generate_world.py. Do not edit by hand. -->\n'
           '<sdf version="1.9">\n<world name="classroom">\n'
           '<physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size>'
           '<real_time_factor>1.0</real_time_factor></physics>\n'
           '<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>\n'
           '<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>\n'
           '<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>\n'
           '<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
           '<render_engine>ogre2</render_engine></plugin>\n'
           '<plugin filename="exam_proctor_student_animator" name="exam_proctor::StudentAnimator">'
           '<topic>/classroom/student_link_poses</topic></plugin>\n'
           '<scene><ambient>0.45 0.45 0.45 1</ambient><background>0.75 0.82 0.9 1</background>'
           '<shadows>true</shadows><grid>false</grid></scene>\n'
           '<light type="directional" name="sun"><cast_shadows>true</cast_shadows>'
           '<pose>0 0 10 0 0 0</pose><diffuse>0.7 0.7 0.68 1</diffuse>'
           '<specular>0.2 0.2 0.2 1</specular><direction>-0.3 0.2 -1</direction></light>\n']
    for i, lx in enumerate((-4.0, 0.0, 4.0)):
        for j, ly in enumerate((-3.0, 3.0)):
            out.append(light(f'ceiling_light_{i}_{j}', (lx, ly, 2.8)))

    # Room shell.
    shell = box('floor', (0, 0, -0.05), (L.ROOM_X + 2 * t, L.ROOM_Y + 2 * t, 0.1), (0.78, 0.76, 0.70))
    shell += box('wall_front', (hx + t / 2, 0, h / 2), (t, L.ROOM_Y + 2 * t, h), wall)
    shell += box('wall_back', (-hx - t / 2, 0, h / 2), (t, L.ROOM_Y + 2 * t, h), wall)
    shell += box('wall_left', (0, hy + t / 2, h / 2), (L.ROOM_X, t, h), wall)
    shell += box('wall_right', (0, -hy - t / 2, h / 2), (L.ROOM_X, t, h), wall)
    shell += box('whiteboard_frame', (hx - 0.015, 0, 1.5), (0.02, 4.1, 1.3), METAL, False)
    shell += box('whiteboard', (hx - 0.03, 0, 1.5), (0.02, 4.0, 1.2), (0.98, 0.98, 0.98), False)
    for i, wx in enumerate((-3.5, 0.0, 3.5)):
        shell += box(f'window{i}', (wx, hy - 0.01, 1.7), (2.0, 0.02, 1.3), (0.55, 0.75, 0.9), False)
    shell += box('door', (hx - 1.5, -hy + 0.01, 1.05), (1.0, 0.02, 2.1), (0.45, 0.28, 0.15), False)
    shell += box('clock', (-hx + 0.01, 0, 2.3), (0.02, 0.35, 0.35), (0.95, 0.95, 0.95), False)
    out.append(static_model('room', (0, 0, 0), 0, shell))

    # Teacher's desk and chair at the front, facing the class.
    tx = L.TEACHER_DESK_X
    out.append(static_model('teacher_desk', (tx, 0, 0), math.pi,
                            desk_body(L.TEACHER_DESK_DEPTH, L.TEACHER_DESK_WIDTH, 0.76)))
    out.append(static_model('teacher_chair', (tx + 0.75, 0, 0), math.pi, chair_body(0)))

    # Student desks, chairs and articulated students.
    for r, c, x, y in L.desks():
        sid = L.student_id(r, c)
        out.append(static_model(f'desk_{sid}', (x, y, 0), 0,
                                desk_body(L.DESK_DEPTH, L.DESK_WIDTH, L.DESK_HEIGHT, rng)))
        out.append(static_model(f'chair_{sid}', (x, y, 0), 0, chair_body(-L.CHAIR_OFFSET)))
        out.append(student_model(R.model_name(sid), L.seat_position(r, c), rng))

    out.append('</world>\n</sdf>\n')
    return ''.join(out)


def main():
    default = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'worlds', 'classroom.sdf')
    path = sys.argv[1] if len(sys.argv) > 1 else default
    with open(path, 'w') as f:
        f.write(build_world())
    print(f'wrote {os.path.abspath(path)}')


if __name__ == '__main__':
    main()
