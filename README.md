Arduino Playground
==================

A space for small, exploratory [Arduino][arduino] projects.

  [arduino]: http://arduino.cc

Sensors
-------

### Sharp GP2Y0A02YK and GP2Y0A710K0F

An Arduino-based driver for two Sharp Infrared distance sensors (IR rangers), to be used as part of a [ROS][ros] robot.

**Note**: The instructions below a [_Desktop-Full Install_][ros-setup] of ROS Kinectic Kame is available.

**Note**: The instructions below also assume you're using an Arduino UNO, and that it is accessible `/dev/ttyACM0`.

  [ros]: http://www.ros.org
  [ros-setup]: http://wiki.ros.org/kinetic/Installation/Ubuntu

#### Installation

```bash
# Install the rosserial_arduino and rosserial_python packages
sudo apt-get install ros-rosserial-arduino ros-rosserial-python

# Compile the firmware
catkin_make sensor_gp2y0a02yk_firmware_main

# Upload the firmware to the arduino
catkin_make sensor_gp2y0a02yk_firmware_main-upload
```
#### Usage

```bash
# Start the ROS master node
roscore

# In a second terminal start a Python client for rosserial
rosrun rosserial_python serial_node.py /dev/ttyACM0

# In a third terminal listen to the Arduino topic
rostopic echo chatter
```

#### Roadmap

1. Determine when to start measuring and how to properly timestamp measurement data
1. Publish raw voltage values to the `/raw_voltage` topic
1. Measure and trace a characteristic: output voltage [V] = f(distance [m])
1. Add calibration data and compute distance values
1. Publish `sensor_msgs/Range` messages to the `/scan` topic
