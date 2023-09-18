# f1_tenth_ros2
 
Launch stanley_controller pkg

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