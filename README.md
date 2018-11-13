## Intro
Use for my master thesis
A ROS pkg for kinect2+ kuka iiwa7 + moveit in gazebo

Folders wsg50, iiwa_stack, and depth sensor are modified from other open source packages.
Need to do some investigation about licenses.
If I violate the license please inform me.


### Notes about real hardware connection
- topic `/iiwa/joint_states` is not published
    - solution1: launch the iiwa_hw
    - solution2: set ROS param `/iiwa/publishJointStates` to true
    ref: see [issue](https://github.com/IFL-CAMP/iiwa_stack/issues/39)
