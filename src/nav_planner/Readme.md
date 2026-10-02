### Start Gazebo

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Start Localization

```bash
ros2 launch nav2_bringup localization_launch.py \
    use_sim_time:=True \
    autostart:=True \
    map:=/home/rishi/nav_ws/src/nav_planner/maps/my_map.yaml
```

### Start Navigation Stack

```bash
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=True \
    autostart:=True \
    map:=/home/rishi/nav_ws/src/nav_planner/maps/my_map.yaml
```

---

## Start Navigation

To navigate the AMR within the environment, run:

```bash
ros2 run nav_planner navigator_final.py
```

---

## Gazebo GUI Follow Camera Offset

To adjust the Gazebo GUI follow camera offset:

```bash
gz service -s /gui/follow/offset \
    --reqtype gz.msgs.Vector3d \
    --reptype gz.msgs.Boolean \
    --timeout 1000 \
    --req 'x: -1.5, y: 0.0, z: 1.0'
```
