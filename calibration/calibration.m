
function pos_sensor = position_sensor(model_pose, tracker_pose, axis_length)

  pos_sensor = zeros(size(model_pose, 1), 3);

  for i = 1:size(model_pose, 1),

      x_b = model_pose(i, 1);
      y_b = model_pose(i, 2);
      theta_b = model_pose(i, 3);

      x_s = tracker_pose(i, 1);
      y_s = tracker_pose(i, 2);
      theta_s = tracker_pose(i, 3);

      x = x_b + axis_length*cos(theta_b);
      y = y_b + axis_length*sin(theta_b);
      theta = theta_b + theta_s;

      pos_sensor(i, :) = [x, y, theta];
  end

endfunction
