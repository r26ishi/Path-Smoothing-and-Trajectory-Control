# Path-Smoothing-and-Trajectory-Control
This project implements a modular navigation pipeline for a differential-drive robot. It takes a set of discrete 2D waypoints, generates a smoothed path, converts the path into a time-parameterized trajectory, and uses a feedback controller to follow that trajectory.

## Terminal 1: To spawn turtlbot
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
## Terminal 2: coarse path Publisher
```bash
ros2 run nav_planner coarse_path.py   --ros-args   -p csv_file:=/workspace_diry/raw_path.csv   -p frame_id:=map
```
## Terminal 3: Path Smoother
```bash
ros2 run nav_planner smoother_path.py 
```
## Terminal 4: Trajectory Generator
```bash
ros2 run nav_planner trajectory.py
```
## Terminal 5: Trajectory Controller
```bash
ros2 run nav_planner trajectory_controller.py 
```
<img width="1000" height="800" alt="coarse_path_vs_smooth_plan" src="https://github.com/user-attachments/assets/b53605f7-6c6d-49bd-98d1-5bf88b4c299c" />

The coarse_paath vs smooth_path

## Single launch file to run all the nodes
```bash
ros2 launch nav_planner navigation.launch.py
```
