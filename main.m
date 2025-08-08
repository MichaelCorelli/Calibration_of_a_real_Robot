close all
clear
clc

source "robot_calibration/trajectory.m"
source "robot_calibration/calibration.m"
source "robot_calibration/plots.m"
source "robot_calibration/utilities.m"

moving = false;
samples_factor = [];
range1_start = []; range1_end = []; range1_samples_factor = [];
range2_start = []; range2_end = []; range2_samples_factor = [];

#to observe the odometry and tracker trajectories in motion, set: on
#to set a value for subsample, use: subsample=value
#to set two subsample values with two ranges, use: range_subsample=value1,start1,end1,value2,start2,end2
args = argv();
moving = false;
samples_factor = [];
range1_start = []; range1_end = []; range1_samples_factor = [];
range2_start = []; range2_end = []; range2_samples_factor = [];

for i = 1:length(args)
    arg = args{i};

    if strcmp(arg, "on")
        moving = true;
    elseif startsWith(arg, "subsample=")
        if isempty(samples_factor) && isempty(range1_samples_factor) && isempty(range2_samples_factor)
            tokens = strsplit(arg, "=");
            samples_factor = str2double(tokens{2});
        else
            fprintf('range_subsample is already set.\n');
        end
    elseif startsWith(arg, "range_subsample=")
        if isempty(samples_factor) && isempty(range1_samples_factor) && isempty(range2_samples_factor)
            tokens = strsplit(arg, "=");
            range_params = strsplit(tokens{2}, ",");
            if length(range_params) == 6
                range1_samples_factor = str2double(range_params{1});
                range1_start = str2double(range_params{2});
                range1_end = str2double(range_params{3});
                range2_samples_factor = str2double(range_params{4});
                range2_start = str2double(range_params{5});
                range2_end = str2double(range_params{6});
            else
                error('range_subsample: incorrect number of parameters');
            end
        else
            fprintf('subsample is already set.\n');
        end
    end
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

if ~isempty(range1_samples_factor) && ~isempty(range2_samples_factor)
    samples_to_keep = [];
    total_samples = length(time);

    if range1_start < 1 || range1_end > total_samples || range2_start < 1 || range2_end > total_samples
        error('Not valid ranges.');
    end

    if range1_start > range1_end || range2_start > range2_end
        error('Not valid ranges.');
    end

    for i = 1:total_samples
        keep_sample = true;

        if i >= range1_start && i <= range1_end
            if mod(i - range1_start + 1, range1_samples_factor) == 0
                keep_sample = false;
            end
        elseif i >= range2_start && i <= range2_end
            if mod(i - range2_start + 1, range2_samples_factor) == 0
                keep_sample = false;
            end
        end

        if keep_sample
            samples_to_keep = [samples_to_keep, i];
        end
    end

    time = time(samples_to_keep);
    ticks = ticks(samples_to_keep, :);
    model_pose = model_pose(samples_to_keep, :);
    tracker_pose = tracker_pose(samples_to_keep, :);

elseif ~isempty(samples_factor)

    samples_to_keep = [];
    for i = 1:length(time)
        if mod(i, samples_factor) ~= 0
            samples_to_keep = [samples_to_keep, i];
        end
    end

    time = time(samples_to_keep);
    ticks = ticks(samples_to_keep, :);
    model_pose = model_pose(samples_to_keep, :);
    tracker_pose = tracker_pose(samples_to_keep, :);

    fprintf('Final samples %d (initial samples %d), (removed 1 every %d)\n', length(time), length(Z{1}), samples_factor);
else
    fprintf('Using full dataset (%d samples)\n', length(time));
end

#delta ticks
delta_ticks = delta_ticks(ticks, max_encoder_value);
#delta time
delta_time = delta_time(time);

#odometry of front-tractor tricycle robot
x_initial = [Ksteer, Ktraction, axis_length, steer_offset];
odometry_pose = odometry(x_initial, delta_ticks, delta_time);

#plot of: odometry, tracker and odometry estimated
plot_odometry_trajectory(odometry_pose, model_pose, tracker_pose, moving, h, delta_time);
pause(1);
plot_odometry_error(model_pose, odometry_pose, h, time);
pause(1);

#start calibration
odometry_pose = odometry_pose(:, 1:3);
n_iter = 25;
jacobian_type = false; #set true for numerical jacobian and false for analytical jacobian

[X, laser_params, axis_length_final, chi_stats, n_inliers] = odometry_calibration(odometry_pose, tracker_pose, x_initial, n_iter, jacobian_type);

#errors
odometry_corrected = odometry_correction(X, laser_params, odometry_pose);
error_before = mean(vecnorm(tracker_pose - odometry_pose(:, 1:3), 2, 2));
odometry_corrected = to_tracker_frame(odometry_corrected, laser_params);
error_after = mean(vecnorm(tracker_pose - odometry_corrected(:, 1:3), 2, 2));

fprintf('\nCalibration:\n');
fprintf('Mean error before calibration: %.4f m\n', error_before);
fprintf('Mean error after calibration: %.4f m\n', error_after);
fprintf('Improvement: %.4f m (%.1f%%)\n', error_before - error_after, (error_before - error_after) / error_before * 100);

#results
disp('Chi-square statistics:')
disp(chi_stats);
plot_chi_stats(chi_stats, h);
pause(1);

fprintf('\nCalibrated parameters\n');
steer_offset_cal = atan2(X(2,1), X(1,1));
R = [cos(steer_offset_cal), -sin(steer_offset_cal), 0;
     sin(steer_offset_cal),  cos(steer_offset_cal), 0;
                         0,                      0, 1];
S = R' * X;

ktraction_cal = S(1,1) * Ktraction;
ksteer_cal = S(2,2) * Ksteer;
ktheta_cal = S(3,3);

fprintf('ktraction (speed scale): %.6f (initial: %.6f), change: %.1f%%\n', ktraction_cal, Ktraction, (ktraction_cal/Ktraction - 1)*100);
fprintf('ksteer (steering scale): %.6f (initial: %.6f), change: %.1f%%\n', ksteer_cal, Ksteer, (ksteer_cal/Ksteer - 1)*100);
fprintf('ktheta (rotation scale): %.6f (initial: 1.0), change: %.1f%%\n', ktheta_cal, (ktheta_cal - 1)*100);
fprintf('steer_offset: %.6f rad = %.2f° (initial: %.2f°), change: %.2f°\n', steer_offset_cal, steer_offset_cal*180/pi, steer_offset*180/pi, (steer_offset_cal - steer_offset)*180/pi);
fprintf('axis_length: %.6f m (initial: %.3f m), change: %.1f%%\n', axis_length_final, axis_length, (axis_length_final/axis_length - 1)*100);

fprintf('Scale factor:\n');
fprintf('Scale X: %.6f (%.2f%% change)\n', S(1,1), (S(1,1)-1)*100);
fprintf('Scale Y: %.6f (%.2f%% change)\n', S(2,2), (S(2,2)-1)*100);
fprintf('Scale Theta: %.6f (%.2f%% change)\n', S(3,3), (S(3,3)-1)*100);

fprintf('\nSensor position\n');
fprintf('2D position of the sensor with respect to the base link:\n');
fprintf('x: %.4f m\n', laser_params(1));
fprintf('y: %.4f m\n', laser_params(2));
fprintf('theta: %.4f rad (%.2f degrees)\n', laser_params(3), laser_params(3)*180/pi);

fprintf('\nCalibration validation\n');
fprintf('Error improvement: %.1f%%\n', (error_before - error_after) / error_before * 100);
fprintf('Final chi-square: %.6e\n', chi_stats(end));
fprintf('Initial chi-square: %.6e\n', chi_stats(1));
fprintf('Chi-square reduction: %.1f%%\n', (chi_stats(1) - chi_stats(end)) / chi_stats(1) * 100);

plot_odometry_calibrated(odometry_corrected, tracker_pose, moving, h, delta_time);
pause(1);
plot_odometry_calibrated_error(odometry_corrected, tracker_pose, h, time);
pause(1);
plot_theta_calibrated(odometry_corrected, tracker_pose, h, time);
pause(1);

diary off;