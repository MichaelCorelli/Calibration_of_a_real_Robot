
close all
clear
clc

source "calibration/calibration.m"

#to observer the robot and sensor trajectories moving set: on
args = argv();
if length(args) > 0 && args{1} == 'on'
    moving = true;
else
    moving = false;
end

h = figure(1);
more off;

#parameters: Ksteer, Ktraction, axis_length and steer_offset
Ksteer = 0.1;
Ktraction = 0.0106141;
axis_length = 1.4;
steer_offset = 0;

#load the dataset and split the data in variables: time, ticks, model_pose and tracker_pose
disp('Loading the dataset');

file = fopen('./dataset.txt', 'r');
Z = textscan(file, 'time: %f ticks: %f %f model_pose: %f %f %f tracker_pose: %f %f %f', 'MultipleDelimsAsOne', true, 'HeaderLines', 8);
fclose(file);

time = Z{1};
ticks = [Z{2}, Z{3}];
model_pose = [Z{4}, Z{5}, Z{6}];
tracker_pose = [Z{7}, Z{8}, Z{9}];

disp('Dataset loaded');

%2D position of the sensor w.r.t. the base link
pose_sensor = position_sensor(model_pose, tracker_pose);

disp("2D position of the sensor w.r.t. the base link:");
disp(pose_sensor);

#2D trajectory of the sensor w.r.t. the base link
disp('2D trajectory of the sensor w.r.t. the base link');
plot_sensor_trajectory(pose_sensor, model_pose, tracker_pose, moving, h, time);

#2D error trajectory of the sensor w.r.t. the base link
disp('2D error trajectory of the sensor w.r.t. the base link');
plot_sensor_error(pose_sensor, tracker_pose, moving, h, time);
