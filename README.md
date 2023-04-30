# haruto_description
This repository is for haruto robot related urdf model

## Dependencies involved
- [ ] robot_state_publisher
- [ ] joint_state_publisher_gui

## Install dependencies
```
  rosdep install --from-paths src/haruto_description -y --ignore-src
```

## Build and Execution
- To build the package

```
  colcon build --packages-select haruto_description
```

- To visualize the robto 
```
  ros2 launch haruto_description visualize_robot.launch.py
```
