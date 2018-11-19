# ROS_Boson
ROS plugin for FLIR Boson Thermal camera

1.- Set the ROS environment variables
$ source ~/catkin_ws/devel/setup.bash

2.- Compile the source code
$ cd ~/catkin_ws
$ catkin_make

3.- move to the project folder
$ roscd flir_boson

4.- Launch the driver.
Inside the flir_boson.launch file you will find the serial port parameters also it launchs the nodes
This command launchs ROS core if it is not running.
$ roslaunch flir_boson flir_boson.launch

5.- Check the topics running on the system
rgomez@maxwell:~/catkin_ws/src/flir_boson$ rostopic list

The output should be similar to this:

/flir/command/thermal/colorpalette
/flir/command/thermal/reboot
/flir/state/thermal/colorpalette
/flir/state/thermal/coretemperature
/flir/state/thermal/fwversion
/flir/state/thermal/partnumber
/flir/state/thermal/serialnumber
/rosout
/rosout_agg

6.- To check the status of each command run:
$ rostopic echo /flir/state/thermal/serialnumber
$ rostopic echo /flir/state/thermal/colorpalette

7.- Set a new value:
- Reboot
$ rostopic pub /flir/command/thermal/reboot std_msgs/String -- \"00000001\"
- Color Palette (The LUT ID value must be in Hexadecimal)
$ rostopic pub /flir/command/thermal/colorpalette std_msgs/String -- \"00000001\"

