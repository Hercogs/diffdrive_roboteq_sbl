# diffdrive_roboteq_sbl

<hr>

## Description
This package contains working hardware interface for Roboteq SBL BLDC controller family. ALthough it might work for other Roboteq controller as well. It has velocity command interface and velocity and position state interfaces. It handles interrupted communication - in case controller gets disconnected, it automatically reconnects.

## Installation

Download source and install dependencies:
```
cd <path/to/your/ros_ws>
git clone https://github.com/Hercogs/diffdrive_roboteq_sbl.git src/diffdrive_roboteq_sbl
rosdep update
rosdep install --ignore-src --default-yes --from-path src
```

Build package:
```
colcon build
source install/setup.bash
```

## Usage

**Important:** Before running any code, calibrate your motors and controller using Roboteq <a href="https://readme.com/" target="_blank">`Roborun+ PC Utility`</a>. If you cannot spin motors from `Roborun+ PC Utility`, then try to find problem before continue with ROS2 hardware interface.

Before testing this package, you **must check controller port name and chnage** to correct one in file: `./diffdrive_roboteq_sbl/description/ros2_control/diffbot.ros2_control.xacro`. Parameter name to change: `device_name`.
Check other paramaters as well, otherwise odometry and velocity command will be wrong. 

To try out this package, execute:
`ros2 launch diffdrive_roboteq_sbl diffbot.launch.py`

Now you should be able to see RVIZ and control your robot using: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`


If everything works, then this hardware interface is ready to be used in your project!



<<<<<<< HEAD
### Thanks to:
    - Christoph Fröhlich for great ROS@_control examples and video tutorials!
=======
#### Thanks to:
   - Christoph Fröhlich for great ROS2_control examples and video tutorials!
>>>>>>> 948deaa (feat: updated README.md)

