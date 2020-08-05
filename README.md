The state_rviz_plugin
==============================================

## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/> Introduction

This repository includes Rviz plugins to display the whole-body state and trajectory of a robot. These plugins uses dedicated ROS messages defined in [state_msgs](https://github.com/cmastalli/state_msgs) package. To process efficiently the messages, e.g. robot display without TF broadcasting, it requires [pinocchio](https://github.com/stack-of-tasks/pinocchio) library.

[![](https://j.gifs.com/wV4xzJ.gif)](https://youtu.be/6FSIQkOwTJM)

## :penguin: Building

1. Installation pinocchio from any source (ros / robotpkg binaries or source)

2. Building the state_msgs and these packages in your catkin workspace:
    ```bash
	cd your_ros_ws/
	catkin_make
    ```

## :copyright: Credits

### :writing_hand: Written by

- [Carlos Mastalli](https://cmastalli.github.io/), The University of Edinburgh :uk: