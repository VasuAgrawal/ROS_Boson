# ROS_Boson
ROS plugin for FLIR Boson Thermal camera

-----------------------------
Getting started
-----------------------------
1.- Set the ROS environment variables
```$xslt
$ source ~/catkin_ws/devel/setup.bash
```
2.- Compile the source code
```$xslt
$ cd ~/catkin_ws
$ catkin_make
```
3.- move to the project folder
```$xslt
$ roscd flir_boson
```
4.- Launch the driver.
Inside the flir_boson.launch file you will find the serial port parameters also it launchs the nodes
This command launchs ROS core if it is not running.
```$xslt
$ roslaunch flir_boson flir_boson.launch
```
5.- Check the topics running on the system
```$xslt
rgomez@maxwell:~/catkin_ws/src/flir_boson$ rostopic list
```
The output should be similar to this:
```$xslt
/flir/command/thermal/colorpalette
/flir/command/thermal/reboot
/flir/state/thermal/colorpalette
/flir/state/thermal/coretemperature
/flir/state/thermal/fwversion
/flir/state/thermal/partnumber
/flir/state/thermal/serialnumber
/rosout
/rosout_agg
```
6.- To check the status of each command run:
```$xslt
$ rostopic echo /flir/state/thermal/serialnumber
$ rostopic echo /flir/state/thermal/colorpalette
```
7.- Set a new value:
- Reboot
```$xslt
$ rostopic pub /flir/command/thermal/reboot std_msgs/String -- \"00000001\"
```
- Color Palette (The LUT ID value must be in Hexadecimal)
```$xslt
$ rostopic pub /flir/command/thermal/colorpalette std_msgs/String -- \"00000001\"
```
