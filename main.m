
close all
clear
clc

source "robot_calibration/calibration.m"

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
max_encoder_value = [8192, 5000];

disp('Dataset loaded');

%elimination of the overflowed ticks
delta_ticks = zeros(size(ticks, 1), 2);

for i = 1:size(ticks, 1)
    if i == 1
        delta_ticks(i, :) = [0, 0];
    else
        diff_steer = (ticks(i,1) - ticks(i-1,1));
        diff_traction = (ticks(i,2) - ticks(i-1,2));
        
        if abs(diff_steer) > max_encoder_value(1)/2
            if (diff_steer) < 0
                diff_steer = (max_encoder_value(1) - ticks(i-1, 1)) + ticks(i,1);
            else
                diff_steer = (max_encoder_value(1) - ticks(i, 1)) + ticks(i-1,1);
            end
        end

        if abs((ticks(i,2) - ticks(i-1,2))/max_encoder_value(2)) > max_encoder_value(2)/2
            if (diff_traction) < 0
                diff_traction = max_encoder_value(2) - ticks(i, 2);
            else
                diff_traction = max_encoder_value(2) + ticks(i-1, 2);
            end
        end

        steer = (diff_steer)/(max_encoder_value(1)/2);
        traction = (diff_traction*60)/(max_encoder_value(2)/2);

        steer_grad = steer*(360/(2*pi));
        traction_grad = traction/(2*pi);
        
        delta_ticks(i, :) = [steer_grad, traction_grad];
    end
end

#compute the delta time
delta_time = zeros(size(time, 1), 1);
for i=1:size(time, 1)
    if i == 1
        delta_time(i, 1) = 0;
    else
        delta_time(i, 1) = time(i, 1) - time(i-1, 1);
    end
end

#odometry of front-tractor tricycle
x = [Ksteer, Ktraction, axis_length, steer_offset];
odometry_pose = odometry(x, delta_ticks, delta_time);

#plot of: odometry, tracker and odometry estimated
plot_odometry_trajectory(odometry_pose, model_pose, tracker_pose, moving, h, delta_time)
pause(1);

#plot of odometry estimated: L2 Norm error
plot_odometry_error(model_pose, odometry_pose, moving, h, time)
pause(1);