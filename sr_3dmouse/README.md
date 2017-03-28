The /sr_3dmouse package allows you to control the ur10 robot with the 3Dconnection mouse in 6-axis

In order to communicate with the device, you must first install the spacenav daemon from here:http://spacenav.sourceforge.net/

Download the daemon, configure, make and make install. 
Then you need to execute the daemon spacenavd once until you restart the computer or execute the provided script to auto load.
 
The mouse controls Gazebo as well so make sure you disable this by pressing the left button of the 3dmouse once. 

You also need to install the spacenav ROS node from: http://wiki.ros.org/spacenav_node

To run it:

First execute spacenav daemon:
```
./spacenav
```
Launch gazebo with rviz with:
```
roslaunch sr_robot_launch sr_right_ur10arm_hand.launch
```
Run spacenav node:
```
rosrun spacenav_node spacenav_node
```
Run mousecontrol:
```
roslaunch sr_3dmouse 3dmouse.launch
```
or run the python file directly