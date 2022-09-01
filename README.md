# Tactile Sensing Glove

The Tactile Glove is a fabric-based sensing glove equipped with up to 64 tactile sensing elements (taxels) measuring normal contact forces.
Optionally, we can also measure hand posture using strain gauges. The acquisition electronics fits into a small housing at the wrist and connects via USB to the PC.

[![Watch the glove in action](https://img.youtube.com/vi/LKwOpRUCs7s/maxresdefault.jpg)](https://www.youtube.com/embed/LKwOpRUCs7s)

## (Optional) Tactile Cyber Glove

Some Tactile Gloves are fitted with pockets at the back to insert CyberGlove strain gauges to measure the hand posture. These CyberGlove sensors employ a separate acquisition system as well, connected via an external acquisition box + power supply, and hooked up to the PC via serial communication (or USB adapter if no COM port is available on the PC).

If using a USB2COM adapter, it will appear as /dev/ttyUSB*. Otherwise adapt the settings to the COM port device.

# Software

Software is primarily maintained for [linux](linux). But there is a [Windows app](win) available too. See corresponding folders for more details.

### Human Hand URDF

We use the `URDF` provided by the ROS package [`human_hand_decription`](https://github.com/ubi-agni/human_hand) to visualize a human hand in `rviz`.

### CyberGlove

* [sr-teleop](https://github.com/ubi-agni/sr-teleop)/[cyberglove](https://github.com/ubi-agni/sr-teleop/tree/indigo-devel/cyberglove): is used to read a CyberGlove' strain gauge values from serial port, calibrating them, and sending joint values to ROS as a [`sensor_msgs/JointState`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/JointState.html).

* [sr-teleop](https://github.com/ubi-agni/sr-teleop)/[sr_remapper](https://github.com/ubi-agni/sr-teleop/tree/indigo-devel/sr_remappers) converts CyberGlove joint angles to human-hand-model angles, based on a configurable mixing matrix.
