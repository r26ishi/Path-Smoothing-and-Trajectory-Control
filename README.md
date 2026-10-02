# Path-Smoothing-and-Trajectory-Control
This project implements a modular navigation pipeline for a differential-drive robot. It takes a set of discrete 2D waypoints, generates a smoothed path, converts the path into a time-parameterized trajectory, and uses a feedback controller to follow that trajectory.

Terminal 1: Raw Path Publisher
ros2 run path_smoothing raw_path_publisher --ros-args \
  -p csv_file:=/home/hild/ros_workspace/src/path_smoothing/raw_path.csv \
  -p frame_id:=map
Terminal 2: Path Smoother
ros2 run path_smoothing smooth_planner --ros-args \
  -p frame_id:=map \
  -p csv_file:=/home/hild/ros_workspace/src/path_smoothing/smoothed_plan.csv
Terminal 3: Trajectory Generator
ros2 run path_smoothing trajectory_generator --ros-args \
  -p sample_spacing:=0.1 \
  -p max_velocity:=0.5 \
  -p max_acceleration:=0.2 \
  -p max_deceleration:=0.2 \
  -p frame_id:=map
Terminal 4: Trajectory Tracker
ros2 run path_smoothing trajectory_tracker --ros-args \
  -p control_rate:=20.0 \
  -p kx:=0.8 \
  -p ky:=2.0 \
  -p ktheta:=2.0 \
  -p max_linear:=0.5 \
  -p max_angular:=1.0

Only one active node should publish commands to /cmd_vel.
