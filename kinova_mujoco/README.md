
# Kinova MuJoCo simulation with scene of objects

![Image of Yaktocat](scripts/simstate.png)

Start simulation by 

```shell script
roslaunch kinova_mujoco table_simulation_generated.launch
```

## Scene preparation for MuJoCo

MuJoCo requires the whole scene to be described by a single model.
We generate URDFs from the robot workspace plus objects. To 
make the simulation more stable, we decompose the object meshes
into mant small convex bodies. 

A sample scene description is given in `data_processing/full_table_scene`
(A [SHOP VRB scene](https://michaal94.github.io/SHOP-VRB/)). 
The scenes can be loaded from compatible json files using `prepare_scene.py`
or created in python code. 

MuJoCo requires that all meshes for the simulation are placed in a single 
folder. Therefore, we copy all meshes referenced in our URDF to the folder 
[meshes](meshes) using the script `scripts/collect_meshes.py`.

## Simulating soft objects

Mujoco is capable of simulating soft objects defined, but these can only be defined using
the mujoco xml format. They are defined as follows:
```xml
<body name="sponge" pos="0 0 2">
    <freejoint/>
        <composite type="ellipsoid" count="5 7 9" spacing="0.05">
        <skin texcoord="true" material="matsponge" rgba=".7 .7 .7 1"/>
        <geom type="capsule" size=".015 0.05" rgba=".8 .2 .1 1"/>
    </composite>
</body>
```
Make sure to give a name to the body tag (here: sponge), because otherwise there will be an exception 
with reading the mujoco world state:
```shell script
[ERROR] [1610016288.815110785]: Exception thrown while processing service call: basic_string::_M_construct null not valid
Exception in thread Thread-6:
Traceback (most recent call last):
  File "/usr/lib/python2.7/threading.py", line 801, in __bootstrap_inner
    self.run()
  File "/opt/ros/melodic/lib/python2.7/dist-packages/rospy/timer.py", line 234, in run
    self._callback(TimerEvent(last_expected, last_real, current_expected, current_real, last_duration))
  File "/home/behrejan/ros_kinetic_ws/ros_cortex_ws/src/kinova_mujoco/nodes/displayObjectMarkers.py", line 31, in update
    res = STATE_SRV.call(req)
  File "/opt/ros/melodic/lib/python2.7/dist-packages/rospy/impl/tcpros_service.py", line 522, in call
    responses = transport.receive_once()
  File "/opt/ros/melodic/lib/python2.7/dist-packages/rospy/impl/tcpros_base.py", line 735, in receive_once
    p.read_messages(b, msg_queue, sock)
  File "/opt/ros/melodic/lib/python2.7/dist-packages/rospy/impl/tcpros_service.py", line 360, in read_messages
    self._read_ok_byte(b, sock)
  File "/opt/ros/melodic/lib/python2.7/dist-packages/rospy/impl/tcpros_service.py", line 343, in _read_ok_byte
    raise ServiceException("service [%s] responded with an error: %s"%(self.resolved_name, str))
ServiceException: service [/kinova_mujoco/getAllObjectPoses] responded with an error: basic_string::_M_construct null not valid
```
There is a working example. Run:
```shell script
roslaunch kinova_mujoco table_simulation_generated_mjxml.launch
```
Note: for now, the sponge can't be displayed in RVIZ. Maybe, we solve that later.