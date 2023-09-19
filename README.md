# f1_tenth_ros2 - ROS2 Package for F1/10 Autonomous Racing
 
This repository contains the stanley_controller package for ROS2, enabling better control for F1/10 autonomous racing.

![Description of GIF](https://github.com/armando-genis/f1_tenth_ros2/raw/main/images/car.gif)

```bash
source /opt/ros/foxy/setup.bash
source /opt/ros/humble/setup.bash
colcon build --packages-select stanley_node_pkg
source install/setup.bash
ros2 launch stanley_node_pkg stanley_controller.launch.py
```

## Distribution
```bash
f1_tenth_ros2/
|-- regulated_pure_pursuit
```

## Install
```bash
sudo apt-get install libeigen3-dev
```

## Authors

- [@armando-genis](https://github.com/armando-genis)
- [@RodrigoGE9772](https://github.com/RodrigoGE9772)
- [@Alexjgxt18](https://github.com/Alexjgxt18)