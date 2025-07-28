
close all
clear
clc

source "robot_calibration/calibration.m"
source "robot_calibration/utilities.m"

#to observe the odometry and tracker trajectories in motion, set: on
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

diary('./output/output.txt'); #to save the output
#load the dataset and split the data in variables: time, ticks, model_pose and tracker_pose
disp('Loading the dataset');

file = fopen('./dataset.txt', 'r');
Z = textscan(file, 'time: %f ticks: %f %f model_pose: %f %f %f tracker_pose: %f %f %f', 'MultipleDelimsAsOne', true, 'HeaderLines', 8);
fclose(file);

time = Z{1};
ticks = [Z{2}, Z{3}];
model_pose = [Z{4}, Z{5}, Z{6}];
tracker_pose = [Z{7}, Z{8}, Z{9}]; #ground truth trajectory
max_encoder_value = [8192, 5000];

disp('Dataset loaded');

#delta ticks
delta_ticks = delta_ticks(ticks, max_encoder_value);
#delta time
delta_time = delta_time(time);

#odometry of front-tractor tricycle
x_initial = [Ksteer, Ktraction, axis_length, steer_offset];
odometry_pose = odometry(x_initial, delta_ticks, delta_time);

#plot of: odometry, tracker and odometry estimated
plot_odometry_trajectory(odometry_pose, model_pose, tracker_pose, moving, h, delta_time);
pause(1);
#plot of odometry estimated: L2 Norm error
plot_odometry_error(model_pose, odometry_pose, h, time);
pause(1);

odometry_pose = odometry_pose(:, 1:3);
n_iter = 18;
jacobian_type = false; #set true for numerical jacobian and false for analytical jacobian
[X, laser_params, chi_stats, n_inliers] = odometry_calibration(odometry_pose, tracker_pose, n_iter, jacobian_type);

disp('Correction matrix X:')
disp(X);
disp('Laser calibrated parameters [x, y, theta]:')
fprintf('x: %.4f m\n', laser_params(1));
fprintf('y: %.4f m\n', laser_params(2));
fprintf('theta: %.4f rad (%.2f degrees)\n', laser_params(3), laser_params(3)*180/pi);
disp('Chi-square statistics:')
disp(chi_stats);

fprintf('\nLaser changes:\n');
fprintf('Initial values: x = 1.500, y = 0.000, theta = 1.000\n');
fprintf('Calibrated values: x = %.3f, y = %.3f, theta = %.3f\n', laser_params(1), laser_params(2), laser_params(3));
pause(1);

plot_chi_stats(chi_stats, h);
pause(1);

odometry_corrected = odometry_correction(X, laser_params, odometry_pose);
error_before = mean(vecnorm(tracker_pose - odometry_pose(:, 1:3), 2, 2));

odometry_corrected = to_tracker_frame(odometry_corrected, laser_params);
error_after = mean(vecnorm(tracker_pose - odometry_corrected(:, 1:3), 2, 2));

fprintf('\nCalibration results\n');
fprintf('Mean error before calibration: %.4f m\n', error_before);
fprintf('Mean error after calibration: %.4f m\n', error_after);
fprintf('Improvement: %.4f m (%.1f%%)\n', error_before - error_after, (error_before - error_after) / error_before * 100);

plot_odometry_calibrated(odometry_corrected, tracker_pose, moving, h, delta_time);
pause(1);
plot_odometry_calibrated_error(odometry_corrected, tracker_pose, h, time);
pause(1);
plot_theta_calibrated(odometry_corrected, tracker_pose, h, time);
pause(1);

fprintf('2D position of the sensor w.r.t. the base link\n');
fprintf('x: %.4f m\n', laser_params(1));
fprintf('y: %.4f m\n', laser_params(2));
fprintf('theta: %.4f rad (%.2f degrees)\n', laser_params(3), laser_params(3)*180/pi);

fprintf('\nKinematic calibrated parameters\n');
scale_x = X(1,1);
scale_y = X(2,2);
scale_th = X(3,3);
ksteer_calibrated = x_initial(1) * scale_th;
fprintf('ksteer: %.6f rad/tick (initial value: %.6f)\n', ksteer_calibrated, x_initial(1));
ktraction_calibrated = x_initial(2) * sqrt(scale_x^2 + scale_y^2);
fprintf('ktraction: %.6f m/tick (initial value: %.6f)\n', ktraction_calibrated, x_initial(2));
steer_offset_calibrated = x_initial(4) * scale_th;
fprintf('steer offset: %.6f rad (initial value: %.6f)\n', steer_offset_calibrated, x_initial(4));
baseline_calibrated = x_initial(3);
fprintf('base line: %.4f m (initial value: %.4f)\n', baseline_calibrated, x_initial(3));

fprintf('\nAnalysis on correction matrix X\n');
fprintf('X:\n');
disp(X);

fprintf('\nScale factor x: %.4f (%.1f%%)\n', X(1,1), (X(1,1)-1)*100);
fprintf('Scale factor y: %.4f (%.1f%%)\n', X(2,2), (X(2,2)-1)*100);
fprintf('Scale factor theta: %.4f (%.1f%%)\n', X(3,3), (X(3,3)-1)*100);
fprintf('Cross-coupling xy: %.6f\n', X(1,2));
fprintf('Cross-coupling yx: %.6f\n', X(2,1));

fprintf('\nCalibration verification\n');
fprintf('Error improvement: %.1f%%\n', (error_before - error_after) / error_before * 100);
fprintf('Final Chi-square: %.6e\n', chi_stats(end));
diary off;