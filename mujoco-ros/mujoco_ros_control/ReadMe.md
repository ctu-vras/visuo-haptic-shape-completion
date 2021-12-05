### Introduction
The _ros_control_ interface for the _MuJoCo_ simulator.
The code parses a given model and register a control interface for each _slide_ or _hinge_ joint.
The _ros_control_ effort based controllers could be then loaded as shown in example _start_simple_robot.launch_.
It provides trajectory based interface e.g. for MoveIt.  

### Installation
#### MuJoCo
Download MuJoCo simulator from http://www.mujoco.org and put the simulator code as well as your MuJoCo key into the folder _~/.mujoco/_, i.e.:
```
ls ~/.mujoco
mjkey.txt  mujoco200

ls ~/.mujoco/mujoco200/
bin  doc  include  model  sample
```

#### Ros package install
Put this package into your ros workspace, in case you do not have a workspace yet:

```
mkdir -p ~/ros/src && cd ~/ros/src
git clone https://github.com/JKBehrens/mujoco-ros.git
cd ..
catkin build
```

#### Run tests
All tests should pass if the installation was successful.
```
cd ~/ros && catkin run_tests mujoco_ros_control && catkin_test_results build/mujoco_ros_control/
...
Summary: X tests, 0 errors, 0 failures, 0 skipped
where X >= 4
```


### Simple 1 DOF Example:

```
roslaunch mujoco_ros_control start_simple_robot.launch
roslaunch mujoco_ros_control rviz.launch
```

### Simulation of Kinova gen 3 with Robotiq 852f gripper:

see my other repository: coming soon!

### Control the camera in mujoco via ROS

You can control the camera in mujoco (images published on `/<node_name>/rgb`).

```shell script
rosrun mujoco_ros_control camera_control x y z rot elev dist
```

 
### Resources
- http://www.mujoco.org/
