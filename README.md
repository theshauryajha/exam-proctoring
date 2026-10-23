# exam-proctoring

A ROS 2 Jazzy + Gazebo Harmonic simulation of a mobile robot that proctors an exam.

The robot patrols a 24-seat classroom and points a pan/tilt camera at the students. The students
are animated: most of the time they write, read or think, and now and then they cheat. They peek at
a neighbour's paper, use a phone, pull out a cheat sheet, pass notes, whisper or turn around. The
simulation publishes ground truth for what every student is doing, so you can record labelled
camera data for a cheating detector.

Cheating detection (perception) is not implemented yet.

## Packages

| Package | |
|---|---|
| [`exam_proctor_sim`](src/exam_proctor_sim) | classroom world, robot, launch file, patrol and student behaviour nodes |
| [`exam_proctor_msgs`](src/exam_proctor_msgs) | `StudentState` and `ClassroomState` ground-truth messages |
| [`exam_proctor_gz_plugins`](src/exam_proctor_gz_plugins) | `StudentAnimator`, a Gazebo system that poses the student models from a topic |

## Quick start

Requires Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic.

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro python3-colcon-common-extensions ros-jazzy-rqt-image-view

git clone https://github.com/theshauryajha/exam-proctoring.git
cd exam-proctoring
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch exam_proctor_sim proctor_sim.launch.py
```

This opens Gazebo with a view from the robot's camera, starts the patrol and animates the students.
If you use conda, run `conda deactivate` first, because ROS Jazzy needs the system Python 3.12.

To record a labelled dataset:

```bash
ros2 bag record /proctor/camera/image_raw /proctor/camera/camera_info /tf /tf_static /odom /classroom/ground_truth
```

See the [`exam_proctor_sim` README](src/exam_proctor_sim/README.md) for the full details: the
student behaviours, launch arguments, topics, how to change the classroom layout, and how
predictable different patrol strategies are to a cheating student.

## License

Apache-2.0, as declared in each package's `package.xml`.

:)
