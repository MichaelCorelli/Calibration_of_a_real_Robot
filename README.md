# Calibration of a (real) Robot
It is a Probabilistic Robotics Project and consist in the calibration of: the kinematic parameters and the sensor positions of a front-rear tricycle-like robot.

### Definition of the problem
The problem is composotite by: a front-rear tricycle-like robot, the encoder ticks, positions of the sensor w.r.t. an external tracking system.
In the following image we can observe the robot rappresentation:
![robot](./images/robot.png)

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

### Methodology
Identify the state space X:
- Qualify the domain
- Find a locally Euclidean parameterization

Identify the measurement space(s) Z:
- Qualify the domain
- Find a locally Euclidean parameterization

Identify the prediction functions h(x)

#### Threshold for estimation process
In the estimation process is used different threshold for x, y, theta and phi. The achieve is skip increments that are too small, because they don’t bring any information to the estimate. The threshold have been chosen after tested different value. The best values achieve the following L2 Norm error results: mean 0.190549, min 0.000000, max 0.306568. Without threshold get: mean 0.265269, min 0.000000, max 0.594321.

#### Reference
- https://gitlab.com/grisetti/probabilistic_robotics_2024_25
