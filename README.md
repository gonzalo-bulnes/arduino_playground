Arduino Playground
==================

A space for small, exploratory [Arduino][arduino] projects.

  [arduino]: http://arduino.cc

Sensors
-------

### Sharp GP2Y0A02YK and GP2Y0A710K0F

1. Determine when to start measuring and how to properly timestamp measurement data
1. Publish raw voltage values to the `/raw_voltage` topic
1. Measure and trace a characteristic: output voltage [V] = f(distance [m])
1. Add calibration data and compute distance values
1. Publish `sensor_msgs/Range` messages to the `/scan` topic
