# Calibration of a (real) Robot
It is a Probabilistic Robotics Project and consist in the calibration of: the kinematic parameters and the sensor positions of a front-rear tricycle-like robot.

### Dataset
The dataset.txt contains the data that come from the sensor of a real robot:
- first lines: information about the kinematic model of the robot, the kinematic parameters to be estimate, an intial guess of those, the encoder order of the field ticks and their max ranges.
- a record consist in: the time stamp, the reading of the steering encoder, the reading of the traction encoder, the odometry and the position of the sensor.

### Input
- A file containing the encoder ticks of all encoders in the system: absolute on the steer axis and incremental on the steering wheel.
- The positions of the sensor w.r.t. an external tracking system.

### Output
- 2D position of the sensor w.r.t the mobile platform.
- The kinematic parameters: Ksteer, Ktraction, SteerOffset and Baseline.

### How run the code
```shell
octave main.m
```
to observer the robot and sensor trajectories moving set:
```shell
octave main.m on
```

#### Threshold for estimation process
In the estimation process is used different threshold for x, y, theta and phi. The achieve is skip increments that are too small, because they don’t bring any information to the estimate. The threshold have been chosen after tested different value. The best values achieve the following result for mean error: 0.310960 on x and 0.252648 on y. Without threshold get: 0.316149 on x and 0.255728 on y.

#### Reference
- https://gitlab.com/grisetti/probabilistic_robotics_2024_25
