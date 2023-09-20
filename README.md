# f1_tenth_ros2 - ROS2 Package for F1/10 Autonomous Racing
 
This repository contains the stanley_controller package for ROS2, enabling better control for F1/10 autonomous racing.

## Install
```bash
sudo apt-get install libeigen3-dev
sudo apt install ros-<ros2-distro>-ackermann-msgs
```

# Stanley controller/Lateral controller
The Stanley controller, often referred to as a lateral controller. Its primary role is to reduce the cross-track error (the lateral distance between the vehicle's current position and a reference path) as the vehicle progresses forward. By doing so, the Stanley controller ensures that the vehicle follows the desired trajectory as closely as possible. We use the `yaw of the path making a subset of interpollated waypoints` that are within some lookahead distance from the closest point to the car.

![Description of GIF](https://github.com/armando-genis/f1_tenth_ros2/blob/main/images/car_f1_10.gif)

```bash
source /opt/ros/foxy/setup.bash #for ros2 foxy
source /opt/ros/humble/setup.bash #for ro2 humble
colcon build --packages-select stanley_node_pkg
source install/setup.bash
ros2 launch stanley_node_pkg stanley_controller.launch.py
```

## Documentation

[Stanley lateral controller](https://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf)


# obstacle avoidance

```bash
source /opt/ros/foxy/setup.bash #for ros2 foxy
source /opt/ros/humble/setup.bash #for ro2 humble
colcon build --packages-select obstacle_pkg
source install/setup.bash
ros2 launch obstacle_pkg Obstacles.launch.py
```

## Distribution
```bash
f1_tenth_ros2/
|-- regulated_pure_pursuit
```


[Stanley lateral controller](https://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf)



## Authors

- [@armando-genis](https://github.com/armando-genis)
- [@RodrigoGE9772](https://github.com/RodrigoGE9772)
- [@Alexjgxt18](https://github.com/Alexjgxt18)