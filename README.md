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
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash #for ros2 foxy
source /opt/ros/humble/setup.bash #for ro2 humble
colcon build --packages-select stanley_node_pkg
source install/setup.bash
ros2 launch stanley_node_pkg stanley_controller.launch.py
```




# obstacle detecter
By subscribing to the /scan topic, the node processes sensor_msgs::LaserScan data, identifying clusters (or groups) of points that represent obstacles based on their proximity, and visualizes these obstacles as point clusters using markers in the rviz visualization tool. Also, it computes the size of the cluster and the average distance of points in that cluster. It use the `geometry_msgs::msg::Point` message to represent a point in a 3D space, with its x, y, and z components, it provides a standard, consistent, and compatible way to represent, process, and visualize the spatial points derived from the LaserScan data.

![Description of GIF](https://github.com/armando-genis/f1_tenth_ros2/blob/main/images/obstacles_pk.gif)

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash #for ros2 foxy
source /opt/ros/humble/setup.bash #for ro2 humble
colcon build --packages-select obstacle_pkg
source install/setup.bash
ros2 launch obstacle_pkg Obstacles.launch.py
```



# Trajectory Planning in the Frenet Space

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash #for ros2 foxy
source /opt/ros/humble/setup.bash #for ro2 humble
colcon build --packages-select frenet_pkg
source install/setup.bash
ros2 launch frenet_pkg Frenet_frame.launch.py
```


## Distribution
```bash
f1_tenth_ros2/
|-- regulated_pure_pursuit
|-- obstacle_pkg
|-- waypoint_visualizer
```


## Authors

- [@armando-genis](https://github.com/armando-genis)
- [@RodrigoGE9772](https://github.com/RodrigoGE9772)
- [@Alexjgxt18](https://github.com/Alexjgxt18)

## Documentation

- [Stanley lateral controller](https://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf)
- [Stanley lateral controller](https://github.com/Hyunwoo-Park-Yonsei/Stanley_method/blob/main/01_stanley.py)
- [Obstacle avoidance ref1](http://www.iri.upc.edu/files/academic/master_thesis/32-MS-Thesis.pdf)
- [Obstacle avoidance ref2](https://ipsj.ixsq.nii.ac.jp/ej/?action=repository_uri&item_id=222945&file_id=1&file_no=1)
- [Frenet frame ref1](https://fjp.at/posts/optimal-frenet/)
- [Frenet frame ref2](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/FrenetOptimalTrajectory/frenet_optimal_trajectory.py)




