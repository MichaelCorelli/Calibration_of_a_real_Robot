
#odometry and calibration functions
function delta_ticks = delta_ticks(ticks, max_encoder_value)
  delta_ticks = zeros(size(ticks, 1), 2);

  for i = 1:size(ticks, 1)
      if i == 1
          delta_ticks(i, :) = [0, 0];
      else
          diff_steer = (ticks(i, 1) - ticks(i-1, 1));
          diff_traction = (ticks(i, 2) - ticks(i-1, 2));
          
          if abs(diff_steer) > max_encoder_value(1)/2
              if (diff_steer) < 0
                  diff_steer = (max_encoder_value(1) - ticks(i-1, 1)) + ticks(i, 1);
              else
                  diff_steer = (max_encoder_value(1) - ticks(i, 1)) + ticks(i-1, 1);
              endif
          endif

          if abs((ticks(i,2) - ticks(i-1, 2))/max_encoder_value(2)) > max_encoder_value(2)/2
              if (diff_traction) < 0
                  diff_traction = max_encoder_value(2) - ticks(i, 2);
              else
                  diff_traction = max_encoder_value(2) + ticks(i-1, 2);
              endif
          endif

          steer = (diff_steer*360)/(max_encoder_value(1)/2);
          traction = (diff_traction*60)/(max_encoder_value(2)/2);
          
          delta_ticks(i, :) = [steer/(2*pi), traction/(2*pi)];
      endif
  endfor
endfunction

function delta_time = delta_time(time)
  for i=1:size(time, 1)
      if i == 1
          delta_time(i, 1) = 0;
      else
          delta_time(i, 1) = time(i, 1) - time(i-1, 1);
      endif
  endfor
endfunction

function odometry_pose = odometry(x, delta_ticks, delta_time)
  ksteer = x(1);
  ktraction = x(2);
  axis_length = x(3);
  steer_offset = x(4);
  odometry_pose = zeros(size(delta_ticks, 1), 4);

  #threshold for increments that are too small
  alpha_x = 1e-5;
  alpha_y = 1e-5;
  alpha_th = 1e-4;
  alpha_phi = 1e-4;
  alpha = [alpha_x, alpha_y, alpha_th, alpha_phi];

  for i = 1:size(delta_ticks, 1)
    ticks_steer = delta_ticks(i, 1);
    ticks_traction = delta_ticks(i, 2);

    v = (ktraction*ticks_traction);
    dphi = (ksteer*ticks_steer) + steer_offset;

    if i == 1
      delta = [0, 0, 0, 0];
    else
      dx = v * cos(odometry_pose(i-1, 3))*cos(odometry_pose(i-1, 4))*delta_time(i, 1);
      dy = v * sin(odometry_pose(i-1, 3))*cos(odometry_pose(i-1, 4))*delta_time(i, 1);
      dth = v * (sin(odometry_pose(i-1, 4))/axis_length)*delta_time(i, 1);
      dphi = dphi*delta_time(i, 1);

      #noise
      n = 0.00001;

      delta = [dx, dy, dth, dphi] + n;

      #skip increments that are too small
      delta(abs(delta) < alpha) = 0;

      if any(delta ~= 0)
        delta = odometry_pose(i-1, :) + delta;
      else
        delta = odometry_pose(i-1, :);
      endif
    endif
    odometry_pose(i, :) = delta;
  endfor
endfunction

function v = t2v(A)
	v(1:2, 1) = A(1:2, 3);
	v(3, 1) = atan2(A(2,1), A(1,1));
endfunction

function A = v2t(v)
  c = cos(v(3));
  s = sin(v(3));
	A = [c, -s, v(1);
	     s,  c, v(2);
	     0,  0,   1];
endfunction

function X = boxplus(x1, x2)
  X = v2t(x2)*x1;
endfunction

function X = boxminus(x1, x2)
  X = t2v(x1*inv(x2));
endfunction

function odometry = odometry_update(x, i, odometry_pose, delta_ticks, delta_time)
  ksteer = x(1);
  ktraction = x(2);
  axis_length = x(3);
  steer_offset = x(4);

  #threshold for increments that are too small
  alpha_x = 1e-5;
  alpha_y = 1e-5;
  alpha_th = 1e-4;
  alpha_phi = 1e-4;
  alpha = [alpha_x, alpha_y, alpha_th, alpha_phi];

  ticks_steer = delta_ticks(i, 1);
  ticks_traction = delta_ticks(i, 2);

  v = (ktraction*ticks_traction);
  dphi = (ksteer*ticks_steer) + steer_offset;

  if i == 1
    delta = [0, 0, 0, 0];
  else
    dx = v * cos(odometry_pose(i-1, 3))*cos(odometry_pose(i-1, 4))*delta_time(i, 1);
    dy = v * sin(odometry_pose(i-1, 3))*cos(odometry_pose(i-1, 4))*delta_time(i, 1);
    dth = v * (sin(odometry_pose(i-1, 4))/axis_length)*delta_time(i, 1);
    dphi = dphi*delta_time(i, 1);

    #noise
    n = 0.00001;

    delta = [dx, dy, dth, dphi] + n;

    #skip increments that are too small
    delta(abs(delta) < alpha) = 0;

    if any(delta ~= 0)
      delta = odometry_pose(i-1, :) + delta;
    else
      delta = odometry_pose(i-1, :);
    endif
  endif
  odometry = delta;
endfunction

function [X, chi_stats, n_inliers, odometry_calibration] = odometry_calibration(X, tracker_pose, delta_ticks, delta_time, n_iter)

  chi_stats = zeros(1, n_iter);
  n_inliers = zeros(1, n_iter);
  threshold_kernel = 1e-4;
  damping = 1;
  odometry_pose = zeros(size(delta_ticks, 1), 4);

  for i = 1:n_iter
    H = zeros(length(X), length(X));
    b = zeros(length(X), 1);
    chi_stats(i) = 0;

    for j = 1:size(tracker_pose, 1)
      [e, J, odometry] = ErrorAndJacobian(X, j, odometry_pose, tracker_pose, delta_ticks, delta_time);
      odometry_pose(j, :) = odometry;
      chi = e'*e;

      if (chi > threshold_kernel)
        e = e*sqrt(threshold_kernel/chi);
        chi = threshold_kernel;
      else
        n_inliers(i)++;
      endif

      chi_stats(i) = chi_stats(i) + chi;
      H = H + J' * J;
      b = b + J' * e;
    endfor
    H = H + eye(4)*damping;

    dx = -H\b;
    X = X + dx;
    fprintf('Iteration %d: X = [%f, %f, %f, %f] and chi stat = %f\n', i, X(1), X(2), X(3), X(4), chi_stats(i));
  endfor
  odometry_calibration = odometry_pose;
endfunction

function [e, J, odometry] = ErrorAndJacobian(X, j, odometry_pose, tracker_pose, delta_ticks, delta_time)
  odometry = odometry_update(X, j, odometry_pose, delta_ticks, delta_time);
  pred = v2t(odometry);
  meas = v2t(tracker_pose(j, :));

  e = boxminus(meas, pred);
    
  epsilon = 1e-4;
  inv_eps2= 0.5/epsilon;
  n = length(X);
  J = zeros(3, n);

  for i = 1:size(J, 1)
    e_vec = zeros(length(X), 1);
    e_vec(i) = epsilon;
    odometry_plus = odometry_update(X+e_vec, j, odometry_pose, delta_ticks, delta_time);
    odometry_minus = odometry_update(X-e_vec, j, odometry_pose, delta_ticks, delta_time);
    J(i, :) = inv_eps2 * (odometry_plus - odometry_minus);
  endfor
endfunction

#plot functions
function plot_odometry_trajectory(odometry_pose, model_pose, tracker_pose, moving, h, delta_time)
  hold on;
  grid on;
  title('2D trajectory of the odometry estimated');
  xlabel('x');
  ylabel('y');
  lim_offset = 2;
  min_model_tracker_pose_1 = min(min((model_pose(:, 1))), min((tracker_pose(:, 1))));
  max_model_tracker_pose_1 = max(max((model_pose(:, 1))), max((tracker_pose(:, 1))));
  min_model_tracker_pose_2 = min(min((model_pose(:, 2))), min((tracker_pose(:, 2))));
  max_model_tracker_pose_2 = max(max((model_pose(:, 2))), max((tracker_pose(:, 2))));
  xlim([min(min((odometry_pose(:, 1))), min_model_tracker_pose_1) - lim_offset max(max((odometry_pose(:, 1))), max_model_tracker_pose_1) + lim_offset]);
  ylim([min(min((odometry_pose(:, 2))), min_model_tracker_pose_2) - lim_offset max(max((odometry_pose(:, 2))), max_model_tracker_pose_2) + lim_offset]);

  if moving
    line1 = plot(NaN, NaN, 'k-', 'linewidth', 2);
    line2 = plot(NaN, NaN, 'b-', 'linewidth', 2);
    line3 = plot(NaN, NaN, 'r-', 'linewidth', 2);
    legend('Odometry pose', 'Tracker pose', 'Odometry estimated pose');
        
    for i = 1:size(odometry_pose, 1),
      if ishandle(h)
        set(line1, 'XData', model_pose(1:i, 1), 'YData', model_pose(1:i, 2));
        set(line2, 'XData', tracker_pose(1:i, 1), 'YData', tracker_pose(1:i, 2));
        set(line3, 'XData', odometry_pose(1:i, 1), 'YData', odometry_pose(1:i, 2));

        pause(delta_time(i));
        drawnow;
      else
        break;
      endif
    endfor

  else
    name_file = "./output/odometry_estimated.png";
    if exist(name_file, "file")
        delete(name_file);
    endif

    plot(model_pose(:, 1), model_pose(:, 2), 'k-', 'linewidth', 2);
    plot(tracker_pose(:, 1), tracker_pose(:, 2), 'b-', 'linewidth', 2);
    plot(odometry_pose(:, 1), odometry_pose(:, 2), 'r-', 'linewidth', 2);

    legend('Odometry pose', 'Tracker pose', 'Odometry estimated pose');
    drawnow;
    try
      saveas(h, name_file);
    catch
      disp("Plot closed before saving is completed");
    end
  endif
    
  waitfor(h);
endfunction

function plot_odometry_error(model_pose, odometry_pose, h, time)
  hold on;
  grid on;
  title('2D error trajectory of the odometry estimated');
  xlabel('Time in seconds');
  ylabel('L2 Norm error');

  error_odometry = [model_pose(:, 1) - odometry_pose(:, 1), model_pose(:, 2) - odometry_pose(:, 2)];
  delta_time = time - time(1);

  L2_norm = sqrt(sum(error_odometry.^2, 2));

  name_file = "./output/error_odometry_estimated.png";
  if exist(name_file, "file")
    delete(name_file);
  endif

  plot(delta_time, L2_norm, 'g-', 'linewidth', 2);

  legend('Odometry estimated: L2 Norm error');
  drawnow;
  try
    saveas(h, name_file);
  catch
    disp("Plot closed before saving is completed");
  end

  printf('L2 Norm error: mean %f, min %f, max %f.\n', mean(L2_norm), min(L2_norm), max(L2_norm));
  waitfor(h);
endfunction

function plot_chi_stats(chi_stats, h)
  hold on;
  grid on;
  title('Chi stat');
  xlabel('Iteration');
  ylabel('Error');

  name_file = "./output/chi_stat.png";
  if exist(name_file, "file")
    delete(name_file);
  endif

  plot(log(chi_stats(1, :) + 1), '-b', 'linewidth', 3)

  legend('Chi stat');
  drawnow;
  try
    saveas(h, name_file);
  catch
    disp("Plot closed before saving is completed");
  end
  
  waitfor(h);
endfunction

function plot_odometry_calibration(odometry_calibration, tracker_pose, moving, h, delta_time)
  hold on;
  grid on;
  title('2D trajectory of the odometry pose during calibration');
  xlabel('x');
  ylabel('y');
  lim_offset = 2;
  xlim([min(min(odometry_calibration(:, 1)), min(tracker_pose(:, 1))) - lim_offset max(max(odometry_calibration(:, 1)), max(tracker_pose(:, 1))) + lim_offset]);
  ylim([min(min(odometry_calibration(:, 2)), min(tracker_pose(:, 1))) - lim_offset max(max(odometry_calibration(:, 2)), max(tracker_pose(:, 1))) + lim_offset]);

  if moving
    line1 = plot(NaN, NaN, 'r-', 'linewidth', 2);
    line2 = plot(NaN, NaN, 'b-', 'linewidth', 2);
    legend('Odometry pose during calibration', 'Tracker pose');
        
    for i = 1:size(odometry_calibration, 1),
      if ishandle(h)
        set(line1, 'XData', odometry_calibration(1:i, 1), 'YData', odometry_calibration(1:i, 2));
        set(line2, 'XData', tracker_pose(1:i, 1), 'YData', tracker_pose(1:i, 2));

        pause(delta_time(i));
        drawnow;
      else
        break;
      endif
    endfor

  else
    name_file = "./output/odometry_calibration.png";
    if exist(name_file, "file")
        delete(name_file);
    endif

    plot(odometry_calibration(:, 1), odometry_calibration(:, 2), 'r-', 'linewidth', 2);
    plot(tracker_pose(:, 1), tracker_pose(:, 2), 'b-', 'linewidth', 2);

    legend('Odometry during calibration', 'Tracker pose');
    drawnow;
    try
      saveas(h, name_file);
    catch
      disp("Plot closed before saving is completed");
    end
  endif
    
  waitfor(h);
endfunction

function plot_odometry_calibrated(odometry_calibrated, tracker_pose, moving, h, delta_time)
  hold on;
  grid on;
  title('2D trajectory of the odometry calibrated');
  xlabel('x');
  ylabel('y');
  lim_offset = 2;
  xlim([min(min(odometry_calibrated(:, 1)), min(tracker_pose(:, 1))) - lim_offset max(max(odometry_calibrated(:, 1)), max(tracker_pose(:, 1))) + lim_offset]);
  ylim([min(min(odometry_calibrated(:, 2)), min(tracker_pose(:, 1))) - lim_offset max(max(odometry_calibrated(:, 2)), max(tracker_pose(:, 1))) + lim_offset]);

  if moving
    line1 = plot(NaN, NaN, 'r-', 'linewidth', 2);
    line2 = plot(NaN, NaN, 'b-', 'linewidth', 2);
    legend('Odometry calibrated pose', 'Tracker pose');
        
    for i = 1:size(odometry_calibrated, 1),
      if ishandle(h)
        set(line1, 'XData', odometry_calibrated(1:i, 1), 'YData', odometry_calibrated(1:i, 2));
        set(line2, 'XData', tracker_pose(1:i, 1), 'YData', tracker_pose(1:i, 2));

        pause(delta_time(i));
        drawnow;
      else
        break;
      endif
    endfor

  else
    name_file = "./output/odometry_calibrated.png";
    if exist(name_file, "file")
        delete(name_file);
    endif

    plot(odometry_calibrated(:, 1), odometry_calibrated(:, 2), 'r-', 'linewidth', 2);
    plot(tracker_pose(:, 1), tracker_pose(:, 2), 'b-', 'linewidth', 2);

    legend('Odometry calibrated', 'Tracker pose');
    drawnow;
    try
      saveas(h, name_file);
    catch
      disp("Plot closed before saving is completed");
    end
  endif
    
  waitfor(h);
endfunction