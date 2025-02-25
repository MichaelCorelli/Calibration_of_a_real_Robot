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

#### Reference
- https://gitlab.com/grisetti/probabilistic_robotics_2024_25
