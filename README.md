# Laser Scan Matcher for ROS
This repo is the implementation of Point-to-Line Iterative Closest Point (PL-ICP) algorithm proposed by Censi [1]. The original code can be found in [2]. However, there are some bugs about ROS tf transform in the original code. I fixed the bugs and added the mapping module following the programming habits in open karto library [3].

## How to use on Ubuntu?
    1. This package has been tested well in Ubuntu 16.04 with ROS Kinetic.

    2. If you want to use it, you must install csm first:
        $ sudo apt-get install ros-kinetic-csm

    3. Clone the repo to your workspace and complie it
        $ cd ~/catkin_ws/src/
        $ git clone https://github.com/nkuwenjian/laser_scan_matcher.git
        $ cd ..
        $ catkin_make
        $ source devel/setup.bash

    4. Run offline rosbag
        $ roslaunch laser_scan_matcher demo.launch
        $ rosbag play <rosbagfile> --clock

## Topics

### Subscribed topics
- `/scan` ([sensor_msgs/LaserScan](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html))
- `/tf` ([tf2_msgs/TFMessage](http://docs.ros.org/melodic/api/tf2_msgs/html/msg/TFMessage.html))

### Published topics
- `/map` ([nav_msgs/OccupancyGrid](http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html))
- `/map_metadata` ([nav_msgs/MapMetaData](http://docs.ros.org/melodic/api/nav_msgs/html/msg/MapMetaData.html))
- `/tf` ([tf2_msgs/TFMessage](http://docs.ros.org/melodic/api/tf2_msgs/html/msg/TFMessage.html))

## Thanks

[1] A. Censi, "An ICP variant using a point-to-line metric," in *Proceedings of the IEEE International Conference on Robotics and Automation*, 2008, pp. 19-25.

[2] https://github.com/ccny-ros-pkg/scan_tools

[3] https://github.com/ros-perception/open_karto
