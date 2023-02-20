# Whole-body state Rviz plugins

## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/> Introduction

This repository includes Rviz plugins to display the whole-body state and trajectory of a robot. These plugins use dedicated ROS messages defined in [whole_body_state_msgs](https://github.com/loco-3d/whole_body_state_msgs) package. To process efficiently the messages, e.g. robot display without TF broadcasting, the plugins use [Pinocchio](https://github.com/stack-of-tasks/pinocchio) library.

![output](https://user-images.githubusercontent.com/3601935/89519116-c6b45600-d7d3-11ea-89a0-fc8df97df2f0.gif)

The whole-body state plugin displays

1. the position and velocity of center of mass,
1. the contact forces,
1. the center of pressure,
1. the instantaneous capture point,
1. the friction cone, and
1. the support polygon.

Instead, the whole-body trajectory plugin displays

1. the center of mass trajectory and body orientation,
1. the swing trajectory and its orientation, and
1. the target posture and contact forces.

In the whole-body state plugin is possible to configure the diplay of the center of mass information in such a way that is projected in the support polygon. In both plugins, the contact forces are normalized according to the robot's weights. Furthermore, it is possible

All visuals are configurable through Rviz GUI. For example, the user can configure the color and the dimension of points, arrows and cones. Additionally, the user can select different lines style display.

## :penguin: Building

1. Installation pinocchio from any source (ros / robotpkg binaries or source)

1. Building the whole_body_state_msgs and these packages in your catkin workspace:

   ```bash
   cd your_ros_ws/
   catkin build #catkin_make
   ```

## Formatting

Run the following in the root of the project:

```
clang-format-6.0 -i $(find . -path ./cmake -prune -o -iregex '.*\.\(h\|c\|hh\|cc\|hpp\|cpp\|hxx\|cxx\)$' -print)
```

## :copyright: Credits

### :writing_hand: Written by

- [Carlos Mastalli](https://romilab.org) Heriot-Watt University :uk:

### :construction_worker: With contributions from

- [Wolfgang Merkt](http://www.wolfgangmerkt.com/research/), University of Oxford :uk:
- [James Foster](https://www.ihmc.us/groups/james-foster/), Institute for Human and Machine Cognition :us:

and maintained by the Robot Motor Intelligence (RoMI) lab.
