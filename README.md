# Tactile Glove

The Tactile Glove is a fabric-based glove equipped with up to 64 tactile sensing elements (taxels). It requires an electronic device for the acquisition and conversion to digital signals before sending the stream to a PC over USB. The electronics is small enough to be integrated into a housing attached to the wrist.

A pair is composed of 2 Gloves, a right and a left glove, each with their independent electronics, and a separate mini-USB connection to be hooked up to the wrist-mounted housing.

The plugged-in gloves should appear as /dev/ttyACM* with * being 0 or 1, assigned in the order of connection. Please connect the left glove first to ensure it gets the number 0.

## (Optional) Tactile Cyber Glove

Some Tactile Gloves are fitted with pockets at the back to insert CyberGlove strain gauges to measure the hand posture. These CyberGlove sensors employ a separate acquisition system as well, connected via an external acquisition box + power supply, and hooked up to the PC via serial communiation (or USB adapter if no COM port is available on the PC).

If using a USB2COM adapter, it will appear as /dev/ttyUSB*. Otherwise adapt the settings to the COM port device.

# Software

Software is primarily maintained for [linux](linux). But there is a [Windows app](win) available too. See corresponding folders for more details.

### Humand Hand URDF

We use the `URDF` provided by the ROS package [`human_hand_decription`](https://github.com/ubi-agni/human_hand) to visualize a human hand in `rviz`.

### CyberGlove

* [sr-teleop](https://github.com/ubi-agni/sr-teleop)/[cyberglove](https://github.com/ubi-agni/sr-teleop/tree/indigo-devel/cyberglove): is used to read a CyberGlove' straing gauge values from serial port, calibrating them, and sending joint values to ROS as a [`sensor_msgs/JointState`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/JointState.html).

* [sr-teleop](https://github.com/ubi-agni/sr-teleop)/[sr_remapper](https://github.com/ubi-agni/sr-teleop/tree/indigo-devel/sr_remappers) converts CyberGlove joint angles to human-hand-model angles, based on a configurable mixing matrix.
