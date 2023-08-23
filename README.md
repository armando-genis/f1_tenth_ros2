# f1_tenth_ros2
 
Launch regulated_pure_pursuit pkg

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select regulated_pure_pursuit
source install/setup.bash
ros2 launch regulated_pure_pursuit pure_pursuit_launch.py
```

```bash
source /opt/ros/foxy/setup.bash
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