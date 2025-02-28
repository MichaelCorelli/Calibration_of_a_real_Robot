
function R = rotation(theta)
  s = sin(theta);
  c = cos(theta);
  R = [c  -s; 
       s  c];
endfunction

function pose_sensor = position_sensor(model_pose, tracker_pose)
  pose_sensor = zeros(size(model_pose, 1), 3);

  for i = 1:size(model_pose, 1),
    x_b = model_pose(i, 1);
    y_b =  model_pose(i, 2);
    theta_b = model_pose(i, 3);
    x_t = tracker_pose(i, 1);
    y_t =  tracker_pose(i, 2);
    theta_t = tracker_pose(i, 3);

    R_inv = rotation(theta_b)';
    diff_xy = [x_t - x_b; y_t - y_b];

    pose_xy = R_inv*diff_xy;
    theta = theta_t - theta_b;

    pose_sensor(i, :) = [pose_xy(1), pose_xy(2), theta];

  end
endfunction

function plot_sensor_trajectory(pose_sensor, model_pose, tracker_pose, moving, h, time)
  hold on;
  grid on;
  title('2D trajectory of the sensor w.r.t. the base link');
  xlabel('x');
  ylabel('y');
  lim_offset = 2;
  min_model_tracker_pose_1 = min(min((model_pose(:, 1))), min((tracker_pose(:, 1))));
  max_model_tracker_pose_1 = max(max((model_pose(:, 1))), max((tracker_pose(:, 1))));
  min_model_tracker_pose_2 = min(min((model_pose(:, 2))), min((tracker_pose(:, 2))));
  max_model_tracker_pose_2 = max(max((model_pose(:, 2))), max((tracker_pose(:, 2))));
  xlim([min(min((pose_sensor(:, 1))), min_model_tracker_pose_1) - lim_offset max(max((pose_sensor(:, 1))), max_model_tracker_pose_1) + lim_offset]);
  ylim([min(min((pose_sensor(:, 2))), min_model_tracker_pose_2) - lim_offset max(max((pose_sensor(:, 2))), max_model_tracker_pose_2) + lim_offset]);
  delta_time = diff(time);
  delta_time = [delta_time; delta_time(end)];

  if moving
    line1 = plot(NaN, NaN, 'r-', 'linewidth', 2);
    line2 = plot(NaN, NaN, 'k-', 'linewidth', 2);
    line3 = plot(NaN, NaN, 'b-', 'linewidth', 2);
    legend([line1, line2, line3], {'Sensor', 'Odometry', 'Tracker'});
        
    for i = 1:size(pose_sensor, 1),
      if ishandle(h)
        set(line1, 'XData', pose_sensor(1:i, 1), 'YData', pose_sensor(1:i, 2));
        set(line2, 'XData', model_pose(1:i, 1), 'YData', model_pose(1:i, 2));
        set(line3, 'XData', tracker_pose(1:i, 1), 'YData', tracker_pose(1:i, 2));

        pause(delta_time(i));
        drawnow;
      else
        break;
      end
    end

  else
    plot(pose_sensor(:, 1), pose_sensor(:, 2), 'r-', 'linewidth', 2);
    plot(model_pose(:, 1), model_pose(:, 2), 'k-', 'linewidth', 2);
    plot(tracker_pose(:, 1), tracker_pose(:, 2), 'b-', 'linewidth', 2);
    legend('Sensor', 'Odometry', 'Tracker');

    drawnow;
  end
    
  waitfor(h);
endfunction

function plot_sensor_error(pose_sensor, tracker_pose, moving, h, time)
  hold on;
  grid on;
  title('2D error trajectory of the sensor w.r.t. the base link');
  xlabel('x');
  ylabel('y');
  lim_offset = 2;
  error_sensor = [pose_sensor(:, 1) - tracker_pose(:, 1), pose_sensor(:, 2) - tracker_pose(:, 2)];
  min_error_sensor_tracker_pose_1 = min(min((error_sensor(:, 1))), min((tracker_pose(:, 1))));
  max_error_sensor_tracker_pose_1 = max(max((error_sensor(:, 1))), max((tracker_pose(:, 1))));
  min_error_sensor_tracker_pose_2 = min(min((error_sensor(:, 2))), min((tracker_pose(:, 2))));
  max_error_sensor_tracker_pose_2 = max(max((error_sensor(:, 2))), max((tracker_pose(:, 2))));
  xlim([min(min((pose_sensor(:, 1))), min_error_sensor_tracker_pose_1) - lim_offset max(max((pose_sensor(:, 1))), max_error_sensor_tracker_pose_1) + lim_offset]);
  ylim([min(min((pose_sensor(:, 2))), min_error_sensor_tracker_pose_2) - lim_offset max(max((pose_sensor(:, 2))), max_error_sensor_tracker_pose_2) + lim_offset]);
  delta_time = diff(time);
  delta_time = [delta_time; delta_time(end)];

  if moving
    line1 = plot(NaN, NaN, 'r-', 'linewidth', 2);
    line2 = plot(NaN, NaN, 'b-', 'linewidth', 2);
    line3 = plot(NaN, NaN, 'g-', 'linewidth', 2);
    legend([line1, line2, line3], {'Sensor', 'Tracker', 'Sensor error trajectory'});
        
    for i = 1:size(pose_sensor, 1),
      if ishandle(h)
        set(line1, 'XData', pose_sensor(1:i, 1), 'YData', pose_sensor(1:i, 2));
        set(line2, 'XData', tracker_pose(1:i, 1), 'YData', tracker_pose(1:i, 2));
        set(line3, 'XData', error_sensor(1:i, 1), 'YData', error_sensor(1:i, 2));

        printf('The sensor error at time %f is %f on x and %f on y\n', time(i, 1), error_sensor(i, 1), error_sensor(i, 2));
        pause(delta_time(i));
        drawnow;
      else
        break;
      end
    end

  else
    plot(pose_sensor(:, 1), pose_sensor(:, 2), 'r-', 'linewidth', 2);
    plot(tracker_pose(:, 1), tracker_pose(:, 2), 'b-', 'linewidth', 2);
    plot(error_sensor(:, 1), error_sensor(:, 2), 'g-', 'linewidth', 2);

    legend('Sensor', 'Tracker', 'Sensor error trajectory');
    drawnow;
  end


  printf('The sensor error mean is %f on x and %f on y\n', mean(error_sensor(:, 1)), mean(error_sensor(:, 2)));
  waitfor(h);
endfunction

function ksteeer_ktraction_calibrated = ksteeer_ktraction_calibration(Ksteer_Ktraction, Z_ticks_pose, time)
	H = zeros(4, 4);
	b = zeros(4, 1);
  delta_time = diff(time);
  delta_time = [delta_time; delta_time(end)];
	
	for i = 1:size(Z_ticks_pose,1)
		error = Ksteer_Ktraction_error_estimate(i, Ksteer_Ktraction, Z_ticks_pose, delta_time);
		J = jacobian(i, Z_ticks_pose);
		H = H + J'*J;
		b = b + J'*error;
	end

	delta_Ksteer_Ktraction = -H\b;
	ksteeer_ktraction_calibrated = Ksteer_Ktraction + delta_Ksteer_Ktraction;
endfunction

function error = Ksteer_Ktraction_error_estimate(i, Ksteer_Ktraction, Z_ticks_pose, delta_time)
	error = [Ksteer_error(i, Ksteer_Ktraction(1), Z_ticks_pose); Ktraction_error(i, Ksteer_Ktraction(2), Z_ticks_pose, delta_time)];
endfunction

function Ksteer_error = Ksteer_error(i, Ksteer_Ktraction, Z_ticks_pose)
  Ksteer_ticks = Z_ticks_pose(i, 1);
  model_theta = Z_ticks_pose(i, 5);
  tracker_theta = Z_ticks_pose(i, 8);
  diff_theta = tracker_theta - model_theta;
  Ksteer_error = Ksteer_ticks - Ksteer_Ktraction*diff_theta;
endfunction

function Ktraction_error = Ktraction_error(i, Ksteer_Ktraction, Z_ticks_pose, delta_time)
  Ktraction_ticks = Z_ticks_pose(i, 2);
  model_pose = Z_ticks_pose(i, 3:4);
  tracker_pose = Z_ticks_pose(i, 6:7);
  diff_pose = sqrt((tracker_pose(1) - model_pose(1))^2 + (tracker_pose(2) - model_pose(2))^2);
  
  meters = diff_pose/delta_time(i);
  Ktraction_error = Ktraction_ticks - Ksteer_Ktraction*meters;
endfunction

function axis_length_steer_offset_calibrated = axis_length_steer_offset_calibration(axis_length_steer_offset, Z_pose)
	H = zeros(4, 4);
	b = zeros(4, 1);
	
	for i = 1:size(Z_pose, 1)
		error = axis_length_steer_offset_error_estimate(i, axis_length_steer_offset, Z_pose);
		J = jacobian(i, Z_pose);
		H = H + J'*J;
		b = b + J'*error;
	end

	delta_axis_length_steer_offset = -H\b;
	axis_length_steer_offset_calibrated = axis_length_steer_offset + delta_axis_length_steer_offset;
endfunction

function error = axis_length_steer_offset_error_estimate(i, axis_length_steer_offset, Z_pose)
  error = [axis_length_error(i, axis_length_steer_offset(1), Z_pose); steer_offset_error(i, axis_length_steer_offset(2), Z_pose)];
endfunction

function axis_length_error = axis_length_error(i, axis_length_steer_offset, Z_pose)
  sensor_pose = Z_pose(i, 1:2);
  model_pose = Z_pose(i, 4:5);
  tracker_pose = Z_pose(i, 7:8);
  diff_pose_sensor = sqrt((sensor_pose(1) - model_pose(1))^2 + (sensor_pose(2) - model_pose(2))^2);
  diff_pose = sqrt((tracker_pose(1) - model_pose(1))^2 + (tracker_pose(2) - model_pose(2))^2);
  
  axis_length_error = diff_pose_sensor - axis_length_steer_offset*diff_pose;
endfunction

function steer_offset_error = steer_offset_error(i, axis_length_steer_offset, Z_pose)
  theta_sensor = Z_pose(i, 3);
  model_theta = Z_pose(i, 6);
  tracker_theta = Z_pose(i, 9);
  diff_theta_sensor = theta_sensor - model_theta;
  diff_theta = tracker_theta - model_theta;
  steer_offset_error = diff_theta_sensor - axis_length_steer_offset*diff_theta;
endfunction

function J = jacobian(i, Z)
  u = Z(i, 1:2);
	J = zeros(2, 4);
  J(1, 1:2)= -u;
	J(2, 3:4)= -u;
endfunction
