Arduino Playground
==================

A space for small, exploratory [Arduino][arduino] projects.

  [arduino]: http://arduino.cc

Sensors
-------

### Sharp GP2Y0A02YK

[![ROS Version](https://img.shields.io/badge/ROS-Kinetic-blue.svg)](http://wiki.ros.org/kinetic)
![Package Version](https://img.shields.io/badge/GP2Y0A02YK%20Driver-0.1.0-brightgreen.svg)

An Arduino-based driver for a Sharp Infrared distance sensor (IR ranger), to be used as part of a [ROS][ros] robot.

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

This project is configured following the ROS tutorial: [CMake with rosserial_arduino][cmake-rosserial-arduino].

  [cmake-rosserial-arduino]: http://wiki.ros.org/rosserial_arduino/Tutorials/CMake

#### Usage

```bash
# Start the ROS master node
roscore

# In a second terminal start a Python client for rosserial
roslaunch src/sensor_gp2y0a02yk/getting_started.launch

# In a third terminal listen to the Arduino topic
rostopic echo /scan
```

#### Roadmap

1. Determine when to start measuring and how to properly timestamp measurement data
1. Publish raw measurement data to the `/raw_data` topic (4.9mV/unit as per the [Arduino documentation][analogRead])
1. Measure and trace a characteristic: output voltage [V] = f(distance [m])
1. Add calibration data and compute distance values
1. Publish `sensor_msgs/Range` messages to the `/scan` topic

  [analogRead]: https://www.arduino.cc/en/Reference/AnalogRead

#### Documentation

See [`src/sensor_gp2y0a02yk/doc/`](src/sensor_gp2y0a02yk/doc/) for details.

Credits
-------

This project is based on code published as part of the [ROS Wiki][ros-wiki] under the terms of the [Creative Commons Attribution 3.0][cc-by-3.0] license.

The [GP2Y0A02YK datasheet][datasheet-copy] was created by Callahan, is believed to be property of Sharp, and is only included here for reference ([original PDF][datasheet-original]). The separate annotations are not part of the original datasheet and correspond to [this repository author][me] measurements.

  [ros-wiki]: http://wiki.ros.org/rosserial_arduino/Tutorials/CMake
  [cc-by-3.0]: https://creativecommons.org/licenses/by/3.0/
  [datasheet-copy]: ./src/sensor_gp2y0a02yk/doc/
  [datasheet-original]: http://docs-asia.electrocomponents.com/webdocs/0a71/0900766b80a71447.pdf
  [me]: https://github.com/gonzalo-bulnes

License
-------

    Arduino Playground
    Copyright (C) 2016 Gonzalo Bulnes Guilpain

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
