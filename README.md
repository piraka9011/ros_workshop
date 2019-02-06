# ROS Advanced Workshop

## Setup

#### Prereqs
```bash
sudo apt-get install gazebo7 ros-kinetic-qt-build ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-ros-pkgs ros-kinetic-ros-control ros-kinetic-control-toolbox ros-kinetic-realtime-tools ros-kinetic-ros-controllers ros-kinetic-xacro python-wstool ros-kinetic-tf-conversions ros-kinetic-kdl-parser ros-kinetic-sns-ik-lib python-rosinstall git-core python-argparse python-wstool python-vcstools python-rosdep ros-kinetic-control-msgs ros-kinetic-joystick-drivers ros-kinetic-xacro ros-kinetic-tf2-ros ros-kinetic-rviz ros-kinetic-cv-bridge ros-kinetic-actionlib ros-kinetic-actionlib-msgs ros-kinetic-dynamic-reconfigure ros-kinetic-trajectory-msgs ros-kinetic-rospy-message-converter
```

#### First create a Catkin workspace
```bash
cd ~/
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
cd ..
catkin build
```

#### Now clone the sawyer simulator repo

```bash
cd ~/catkin_ws/src
git clone https://github.com/RethinkRobotics/sawyer_simulator.git
wstool init .
wstool merge sawyer_simulator/sawyer_simulator.rosinstall
wstool update
cd ..
catkin build
```

Launch the simulation:
```bash
roslaunch sawyer_gazebo sawyer_world.launch
```

## Workshop

#### Dynamic reconfigure

Show the reconfigure GUI
```
rosrun rqt_reconfigure rqt_reconfigure
``` 

##### Hints
- Source your workspace: `source ~/catkin_ws/devel/setup.bash`
- Checking the robot state: `rostopic echo /robot/state`
- Hello Robot!: `rosrun intera_examples joint_torque_springs.py`
- Make your files executable: `chmod a+x cfg/Tutorials.cfg`
- Keyboard Teleop: `rosrun intera_examples joint_position_keyboard.py`
