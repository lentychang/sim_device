# Intro

Some pkgs, depth_sensor + iiwa_stack + moveit + wsg50_ros_pkgs in gazebo and rviz which are used in my thesis.

File structure is still messy. If you have any question please contact me.

**p.s. The top level launch file is in thesis repository, call thesis.launch**

Folders wsg50, iiwa_stack, and depth sensor are modified from other open source packages.
**Need to do some investigation about licenses.**
**If I violate the license please inform me.**

### What have I modified
- depth_sensor
    - change model from kinectv1 to kinectv2, the frame pose need to be further checked
- iiwa_stack
    - toplevel launch file in `thesis` repository is modified from iiwa_gazebo.launch
- wsg50_ros_pkgs
    - wsg_50_drive didn't work for me, probably I changed urdf file before.
    - I use tcp/ip connection to send command(GCL) directly to gripper. See python code `thesis_moveit_config/wsg_gcl_socket.py`


### Env
The enviroment setup files are in my repository `bringups`

There are two environments used for this repository:
    
**Env_name: pkgs**
gazebo: gazebo, iiwa_stack , depth_sensor(kinect2)
moveit: rviz, iiwa_stack, 

---
### Notes

#### General

- Movegroup cannot get joint_states
    1. launchfile solution: using remap flag, if your joint states topic is i.e. /iiwa/joint_states, instead /joint_states
    2. rosrun solution: `rosrun pkgname exacutable joint_states:=/iiwa/joint_states`

#### Simulation

#### Real connection

- No topic `/iiwa/joint_states` is not published from ROSMonintor at Sunriseside
    1. solution1: launch the iiwa_hw
    2. solution2: set ROS param `/iiwa/publishJointStates` to true
    ref: see [issue](https://github.com/IFL-CAMP/iiwa_stack/issues/39)

- Cannot move arm due to time asynchronization
    1. solution: run a joint_state_controller to get a dummy current one
    2. solution: update sunrise's clock might work.
    3. solution: using ntp or chrony...

