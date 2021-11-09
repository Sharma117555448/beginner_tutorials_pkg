# beginner_tutorials_pkg

ROS publisher/subscriber beginner tutorials
[![License: MIT](https://img.shields.io/badge/License-MIT-Default.svg)](https://opensource.org/licenses/MIT)


## Authors
[Charu Sharma](<https://github.com/Sharma117555448>) (UID 117555448)

# Overview
This is ROS publisher/subscriber beginner tutorials. Where a publisher named talker, publishes a message and the subscriber called, listener hears the message. The message type here is a string. A launch file is then created to launch both the nodes.

# Dependencies
Ubuntu 18.04

ROS Melodic 


# Build
## Steps to build
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/Sharma117555448/beginner_tutorials
cd ~/catkin_ws/
catkin_make

```

# Run
Steps to run
## Launch all nodes individually
## 1. Run roscore
```
cd ~/catkin_ws/
source ./devel/setup.bash
roscore
```
## 2. Run talker node (Open a new terminal)
```
cd ~/catkin_ws/
source ./devel/setup.bash
rosrun beginner_tutorials talker
```
## 3. Run listener node (Open a new terminal)
```
cd ~/catkin_ws/
source ./devel/setup.bash
rosrun beginner_tutorials listener
```
## 4. Generate the RQT graph
```
cd ~/catkin_ws/
source ./devel/setup.bash
rosrun rqt_graph rqt_graph
```
## Run code using launch file
## 1. Run roslaunch
```
cd ~/catkin_ws
source devel/setup.bash
catkin_make
roslaunch beginner_tutorials begin_tutorials.launch
```
## 2. Run roslaunch with arguments
```
roslaunch beginner_tutorials talker_listener.launch freq:=<publish_rate>
```
## Example:
```
roslaunch beginner_tutorials talker_listener.launch freq:=10
```

# ROS Service
While the nodes are running, open a new terminal and run
```
cd catkin_ws
source devel/setup.bash
rosservice call /change_string "input_string: <String of your choice>"
```
# Get RQT Console
## Install RQT packages
```
sudo apt-get install ros-melodic-rqt ros-meldoic-rqt-common-plugins
```

## Display outputs from nodes
```
rosrun rqt_console rqt_console
```
## Change the verbosity level of nodes
```
rosrun rqt_logger_level rqt_logger_level
```

# Run cppcheck
Results are stored in `./results/cppcheck.txt` 
```
cppcheck --enable=all --std=c++11 --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./lib") > results/cppcheck.txt 2>&1
```

# Run cpplint
Results are stored in `./results/cpplint.txt`
```
cpplint $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./docs/" -e "^./results" -e "^./lib/") > results/cpplint.txt 2>&1
```

# Reference
http://wiki.ros.org/