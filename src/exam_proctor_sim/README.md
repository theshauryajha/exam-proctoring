# exam_proctor_sim

ROS 2 Jazzy + Gazebo Harmonic simulation of a mobile exam-proctoring robot.

- **World** (`worlds/classroom.sdf`): 14 × 13 m classroom, 4 rows × 3 desk pairs (24 seats),
  exam papers, a teacher's desk and a whiteboard.
- **Students**: 24 seated figures with a torso/head/arm rig (plus a phone, cheat sheet and
  notes that slide out of hiding) animated by `student_behaviors`, which also publishes
  ground truth of who is doing what. Students are posed kinematically, outside physics.
- **Robot** (`urdf/proctor_bot.urdf.xacro`): differential-drive base, 1.5 m mast,
  pan/tilt head with a 1280×720 @ 15 Hz RGB camera (69° horizontal FOV).
- **Patrol** (`exam_proctor_sim/patrol_node.py`): random walk along the aisle grid (random leg
  lengths, speeds and pauses; it reverses at half the intersections, so passing a student says
  nothing about when it returns). The pan/tilt head aims where students have gone longest without
  being in the camera's view (`patrol_strategy:=random` gives a plain random walk and random gaze).
  Every 30 s it logs how many students were in view in the last minute. See
  [Patrol predictability](#patrol-predictability).

Cheating detection (perception) is not implemented yet.

## Packages

| Package | |
|---|---|
| `exam_proctor_sim` | world, robot, launch, patrol and student behaviour nodes |
| `exam_proctor_msgs` | `StudentState`, `ClassroomState` ground-truth messages |
| `exam_proctor_gz_plugins` | `StudentAnimator` Gazebo system that sets student link poses from a topic |

## Build

```bash
conda deactivate            # ROS Jazzy needs system Python 3.12, not conda's
source /opt/ros/jazzy/setup.bash
cd ~/proctor_ws
colcon build --symlink-install
source install/setup.bash
```

Dependencies: `sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro python3-colcon-common-extensions ros-jazzy-rqt-image-view`

## Run

```bash
ros2 launch exam_proctor_sim proctor_sim.launch.py
```

This opens Gazebo (with a *Proctor camera* panel on the right) and `rqt_image_view`
on the camera feed, starts the patrol and animates the students.

| Argument | Default | |
|---|---|---|
| `headless` | `false` | Gazebo server only, no GUI |
| `patrol` | `true` | `false` to drive manually |
| `camera_view` | `true` | open `rqt_image_view` |
| `patrol_strategy` | `coverage` | `coverage`: U-turns and gaze at students unseen longest; `random`: plain random walk and gaze |
| `patrol_seed` | `-1` | random seed for the patrol route and gaze (`-1`: random) |
| `students` | `true` | animate students and publish ground truth |
| `cheat_probability` | `0.1` | chance that a new student episode is cheating (scaled per student) |
| `student_seed` | `-1` | random seed for student behaviour (`-1`: random) |
| `x`, `y`, `yaw` | `3.8`, `1.8`, `3.14159` | spawn pose; must be an aisle intersection |
| `cam_width`, `cam_height`, `cam_fps` | `1280`, `720`, `15` | camera resolution/rate |

Manual driving: `ros2 launch exam_proctor_sim proctor_sim.launch.py patrol:=false`, then
`ros2 run teleop_twist_keyboard teleop_twist_keyboard` and
`ros2 topic pub --once /proctor/camera/pan_cmd std_msgs/msg/Float64 "{data: 1.0}"`.

On this hybrid-graphics laptop, to render on the NVIDIA GPU instead of the Intel iGPU:
`export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia` before launching.

## Students

Each student plays back behaviour episodes. A student's chance of cheating is
`cheat_probability` × a per-student propensity (lognormal, mean 1), so a few students cheat
often and most rarely.

| Normal | Cheating |
|---|---|
| `writing`, `reading`, `thinking` (chin on hand), `looking_up`, `glancing_around` (brief head turn), `stretching`, `scratching_head` | `peeking` at the neighbour's paper, `phone` held just above the desk edge, `cheat_sheet` on the desk beside the exam paper, `passing_note` to the neighbour low between the seats (both students), `whispering` with the neighbour (both), `turning_around` to the row behind |

`glancing_around` is deliberately similar to `peeking`, but shorter and not aimed at the
neighbour's desk, so a detector cannot just flag every head turn.

Arm poses that put a hand somewhere (on the desk, on the phone, at the neighbour) are solved with
inverse kinematics (`student_rig.solve_arm`) against the desk surface, so hands rest on the desk
whatever the torso is doing.

Force a behaviour (for demos or debugging):

```bash
ros2 topic pub --once /classroom/trigger std_msgs/msg/String "{data: 'r1_c2 phone'}"
```

Ground truth is on `/classroom/ground_truth` (`exam_proctor_msgs/ClassroomState`, 30 Hz sim
time): per student `behavior` (episode), `phase` (step within it, e.g. `reaching`,
`handing_over`), `is_cheating` (the episode is cheating), `cheating_visible` (the current
phase shows the action or prop), `partner_id` and the seat position. Record it with the camera
feed to build a labelled dataset:

```bash
ros2 bag record /proctor/camera/image_raw /proctor/camera/camera_info /tf /tf_static /odom /classroom/ground_truth
```

## Patrol predictability

A student who watches the robot can wait for it to move away before cheating. Offline replays of
the patrol node, 2 simulated hours each. A cheater counts as caught if they were in the camera's view
(within 6 m, occlusion not modelled) at some point during 20 s of cheating:

| Patrol | Caught: cheating at a random moment | Caught: cheating at the best moment after the camera or robot leaves |
|---|---|---|
| random walk, random gaze | 38% | 15% |
| look at students unseen longest | 48% | 23% |
| same, sampled views and random re-checks | 46% | 17% |
| look at students unseen longest, reverse at 50% of intersections (**default**) | 50% | 25% |
| sampled views, camera range 9 m instead of 6 m | 80% | 63% |

The best moment for a cheater is 45–60 s after the robot leaves. Route and gaze changes move that
by a few points; seeing students from farther away (a higher-resolution or zoom camera) is what
closes the gap. The node's `gaze_temperature`, `recheck_probability`, `backtrack_probability` and
`route_bias` parameters expose the variants above.

## Topics

| Topic | Type | |
|---|---|---|
| `/proctor/camera/image_raw` | `sensor_msgs/Image` | camera feed (frame `camera_optical_frame`) |
| `/proctor/camera/camera_info` | `sensor_msgs/CameraInfo` | intrinsics |
| `/proctor/camera/pan_cmd`, `/proctor/camera/tilt_cmd` | `std_msgs/Float64` | head joint targets [rad]; +tilt looks down |
| `/proctor/focus_target` | `geometry_msgs/PointStamped` | point the camera is tracking (frame `odom`) |
| `/classroom/ground_truth` | `exam_proctor_msgs/ClassroomState` | what every student is doing |
| `/classroom/trigger` | `std_msgs/String` | force `'<student_id> <behavior>'` |
| `/classroom/student_link_poses` | `gz.msgs.Pose_V` (Gazebo topic, not ROS) | student link poses sent to Gazebo |
| `/cmd_vel` | `geometry_msgs/Twist` | base velocity |
| `/odom` | `nav_msgs/Odometry` | ground-truth pose; `odom` coincides with the world frame |
| `/joint_states`, `/tf`, `/clock` | | from Gazebo / robot_state_publisher |

## Changing the classroom or the students

Room size, desk grid and spacing live in `exam_proctor_sim/classroom_layout.py`; the student
skeleton lives in `exam_proctor_sim/student_rig.py`. The world generator, patrol node and
behaviour node all read them. After editing either:

```bash
ros2 run exam_proctor_sim generate_world   # rewrites src/.../worlds/classroom.sdf
colcon build --symlink-install
```

(Run from the source tree, or pass an output path.) Models are named `desk_r{row}_c{col}`,
`chair_r{row}_c{col}` and `student_r{row}_c{col}`; row 0 is nearest the whiteboard, and
columns 0/1, 2/3, 4/5 share a desk pair.
