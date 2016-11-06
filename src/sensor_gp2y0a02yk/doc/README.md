GP2Y0A02YK Driver Development Log
=================================

Sensor characterization
-----------------------

### Measurement setup

```

     Sensor (vertical)    Target (18x13cm notebook)
      __
     |  ]                 |
    _| |                  |
   | |__]                 |
   | 
   |   |------------------|------> distance [m]
   |
   v 
   Arduino analog input
   Arduino GND
   Arduino 5V power
```

**Notes**

- The sensor is fixed vertically and aligned with the center of the target.
- The origin of thedistance is the flat surface between the lenses.
- The 18x13cm notebook has a white paper cover and is kept in a landscape orientation, orthogonal to the sensor line of sight.

### Measurment data

See [`measurement_data.csv`][data].

  [data]: measurement_data.csv

### Interpretation

At very low range, the sensor output (`V_o`) matches closely the specs. Then the measurments are consistently lower than the specs, but similar in behaviour. Maybe the small size of the target reduces the amount of light that is reflected when distance increases? (I've been usable to observe how the sensor illuminated the scene, however, it is described as a wide angle sensor and I believe this might be a reasonable explanation.)

References
----------

- Arduino's `analogRead` reference: https://www.arduino.cc/en/Reference/AnalogRead
- The `sensor_msgs/Range` definition: http://docs.ros.org/api/sensor_msgs/html/msg/Range.html
- Also: http://www.ros.org/reps/rep-0117.html
- About representing `INFINITY` and `NAN`: https://www.gnu.org/software/libc/manual/html_node/Infinity-and-NaN.html

