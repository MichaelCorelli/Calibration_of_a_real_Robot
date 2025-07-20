
close all
clear
clc

source "robot_calibration/calibration.m"

#to observer the odometry and tracker trajectories moving, set: on
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
tracker_pose = [Z{7}, Z{8}, Z{9}]; #the ground truth trajectory
max_encoder_value = [8192, 5000];

disp('Dataset loaded');

#delta ticks
delta_ticks = delta_ticks(ticks, max_encoder_value);

#delta time
delta_time = delta_time(time);

#odometry of front-tractor tricycle
x = [Ksteer, Ktraction, axis_length, steer_offset];
odometry_pose = odometry(x, delta_ticks, delta_time);

#plot of: odometry, tracker and odometry estimated
plot_odometry_trajectory(odometry_pose, model_pose, tracker_pose, moving, h, delta_time);
pause(1);

#plot of odometry estimated: L2 Norm error
plot_odometry_error(model_pose, odometry_pose, h, time);
pause(1);

tracker_pose = correction_tracker_pose(tracker_pose);
odometry_pose = odometry_pose(:, 1:3);
n_iter = 100;
[X, chi_stats] = odometry_calibration(odometry_pose, tracker_pose, n_iter);
disp('X:')
disp(X);
disp('chi stat:')
disp(chi_stats);
pause(1);

plot_chi_stats(chi_stats, h);
pause(1);

odometry_corrected = odometry_correction(X, odometry_pose);

#plot of calibrated odometry
plot_odometry_calibrated(odometry_corrected, tracker_pose, moving, h, delta_time);
pause(1);

#plot of odometry estimated: L2 Norm error
plot_odometry_calibrated_error(odometry_corrected, tracker_pose, h, time);
pause(1);
