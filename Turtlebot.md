# EECE 5550: Turtlebot session

## Prereqs

Make sure the turtlebot packages are installed
```
sudo apt install ros-kinetic-turtlebot
```

## Connecting to the Turtlebot

### Environment
Edit your `.bashrc` file and make sure that both your ROS and catkin environment are setup:
```
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel.setup.bash
```

### Network
Now connect to NUWave and make sure you can ping your Turtlebot by running
```
ping <turtlebot_name>.neu.edu
```
Example for Splinter:
```
ping splinter.neu.edu
```
You should get some response.

### ROS Master
Now we need to inform ROS that the turtlebot is our ROS Master. We do so by setting the `ROS_MASTER_URI` variable. Make sure you change your robot's name and add the port number 11311 like below.
```
export ROS_MASTER_URI=http://splinter.neu.edu:11311
```
Check that you're connected to the robot by running
```
rostopic list
```
You should see all the topics from the Turtlebot available.

## Running scripts

In your catkin workspace (`catkin_ws` or similar), create a package if you have not and a scripts
folder to put our code in. 
```
cd ~/catkin_ws/src
catkin_create_pkg mobile_robotics std_msgs geometry_msgs rospy roscpp
cd mobile_robotics && mkdir scripts
```
Always make sure that your scripts are executable! You can do so by running
```
chmod +x <filename>
```
and if you run `ls` in the directory, you should notice that your file is now green, meaning it is executable.

### Using ROS run
To run a script, simply run
```
rosrun <pkg_name> <file_name>
```
for example
```
rosrun mobile_robotics simple_vel_cmd.py
```
