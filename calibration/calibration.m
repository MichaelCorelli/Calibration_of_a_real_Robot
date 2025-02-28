
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
      
        disp(error_sensor(i, :));
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

    disp(error_sensor);
    legend('Sensor', 'Tracker', 'Sensor error trajectory');
    drawnow;
  end
    
  waitfor(h);
endfunction
