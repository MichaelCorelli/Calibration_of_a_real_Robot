
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
      #dx = v * cos(odometry_pose(i-1, 3))*cos(odometry_pose(i-1, 4))*delta_time(i, 1);
      #dy = v * sin(odometry_pose(i-1, 3))*cos(odometry_pose(i-1, 4))*delta_time(i, 1);

      dx = v * cos(odometry_pose(i-1, 3)) * delta_time(i, 1);
      dy = v * sin(odometry_pose(i-1, 3)) * delta_time(i, 1);
      dth = v * (sin(odometry_pose(i-1, 4))/axis_length)*delta_time(i, 1);
      dth = mod(dth + pi, 2*pi) - pi;
      dphi = dphi*delta_time(i, 1);
      dphi = mod(dphi + pi, 2*pi) - pi;

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

function [X_final, laser_f, chi_stats, n_inliers] = odometry_calibration(odometry_pose, tracker_pose, n_iter, jacobian_type)

  odometry_inc = new_pose(odometry_pose);
  tracker_inc  = new_pose(tracker_pose);

  state = [reshape(eye(3), [], 1); 1.5; 0; 1];
  threshold = 0.0000198;
  c = 0.000000002;
  d = 1.2;

  chi_stats = zeros(1, n_iter);
  n_inliers = zeros(1, n_iter);
  min_chi = inf;
  n_count = 0;
  max_step = 3;

  for i = 1:n_iter
    [H, b, chi, inliers] = linear_s(state, odometry_inc, tracker_inc, threshold, jacobian_type);
    n_inliers(i) = inliers;

    lambda = c * max(diag(H));
    d_state = - (H + lambda * eye(length(state))) \ b;
    state_new = state + d_state;

    chi_new = chi_tot(state_new, odometry_inc, tracker_inc, threshold);

    if chi_new < chi
      state = state_new;
      chi = chi_new;
      c = c / d;
      n_count = (chi < min_chi) * 0 + (chi >= min_chi) * (n_count + 1);
      min_chi = min(min_chi, chi);
    else
      c = c * d;
      n_count = n_count + 1;
      fprintf('Iter %d: step refused, c = %e\n', i, c);
    endif

    chi_stats(i) = chi;
    laser_params = state(end-2:end);
    fprintf('Iteration %d: chi = %.2e, inliers = %d, laser = [%.3f,%.3f,%.3f]\n', i, chi, inliers, laser_params);

    if norm(d_state) < 1e-7 && i > 1 && abs(chi_stats(i) - chi_stats(i-1)) < 1e-7
      fprintf('Converged at iter %d: chi = %.6e\n', i, chi);
      break;

    elseif n_count >= max_step && i > 10
      fprintf('Stopping at iter %d: chi = %.6e\n', i, chi);
      break;

    endif
  endfor

  X_final = reshape(state(1:9), 3, 3);
  laser_f = state(end-2:end);
endfunction

function [H, b, chi, inliers] = linear_s(state, odom_inc, tracker_inc, threshold, jacobian_type)
  H = zeros(length(state)); b = zeros(length(state), 1);
  chi = 0; inliers = 0;

  for i = 1:size(odom_inc, 1)

    u = odom_inc(i,:)';
    z = tracker_inc(i,:)';
    e = error(state, u, z);

    if jacobian_type
      J = Jacobian_numerical(@(s) error(s, u, z), state, 1e-5);
    else
      J = Jacobian_analytical(state, u, z);
    endif

    r = e' * e;

    if r <= threshold
      w = 1; inliers = inliers + 1;
    else
      w = threshold/r;
    endif

    e = e*w;
    J = J*w;
    chi = chi + min(r, threshold);
    H = H + J'*J;
    b = b + J'*e;
  endfor
endfunction

function e = error(state, u, z)
  X = reshape(state(1:9), 3, 3);
  x_laser = state(10);
  y_laser = state(11);
  theta_laser = state(12);
    
  c = cos(theta_laser);
  s = sin(theta_laser);
  laser = [c, -s, x_laser;
           s,  c, y_laser;
           0,  0,       1];

  e = t2v(inv(laser) * v2t(z) * laser) - X * u;
  e(3) = mod(e(3) + pi, 2*pi) - pi;
endfunction

function J = Jacobian_numerical(f, state, d)
  n = numel(state);
  J = zeros(numel(f(state)), n);
    
  for k = 1:n
      state_p = state;
      state_m = state;
      state_p(k) = state_p(k) + d;
      state_m(k) = state_m(k) - d;
        
      J(:, k) = (f(state_p) - f(state_m))/(2 * d);
  endfor
endfunction

function J = Jacobian_analytical(state, u, z)

  X = reshape(state(1:9), 3, 3);
  x_laser = state(10);
  y_laser = state(11);
  theta_laser = state(12);
    
  c = cos(theta_laser);
  s = sin(theta_laser);

  laser = [c, -s, x_laser; 
           s,  c, y_laser; 
           0,  0,       1];

  dlaser_x = [0, 0, 1; 
              0, 0, 0;
              0, 0, 0];
  dlaser_y = [0, 0, 0;
              0, 0, 1;
              0, 0, 0];
  dlaser_theta = [-s, -c, 0;
                   c, -s, 0;
                   0, 0,  0];

  J = zeros(3, 12);
  J(:, 1:3) = -u(1) * eye(3);
  J(:, 4:6) = -u(2) * eye(3);
  J(:, 7:9) = -u(3) * eye(3);
    
  J(:, 10) = t2v((-inv(laser) * dlaser_x * inv(laser)) * v2t(z) * laser + (inv(laser) * v2t(z) * dlaser_x));
  J(:, 11) = t2v((-inv(laser) * dlaser_y * inv(laser)) * v2t(z) * laser + (inv(laser) * v2t(z) * dlaser_y));
  J(:, 12) = t2v((-inv(laser) * dlaser_theta * inv(laser)) * v2t(z) * laser + (inv(laser) * v2t(z) * dlaser_theta));
endfunction

function odometry_corrected = odometry_correction(X, laser_params, odometry_pose)

  odometry_corrected = zeros(size(odometry_pose, 1), 3);
  odometry_corrected(1, :) = odometry_pose(1, :);
  new_odometry_pose = new_pose(odometry_pose);

  for i = 1:size(new_odometry_pose, 1)
    u = new_odometry_pose(i, :)';
    uc = X * u;
    uc(3) = mod(uc(3) + pi, 2*pi) - pi;
    odometry_corrected(i + 1, :) = t2v((v2t(odometry_corrected(i, :)') * v2t(uc)))';
  endfor
endfunction

function odometry_corrected_tracker = to_tracker_frame(odometry_corrected, laser_params)
  x_laser = laser_params(1);
  y_laser = laser_params(2);
  theta_laser = laser_params(3);
  
  c = cos(theta_laser);
  s = sin(theta_laser);
  laser = [c, -s, x_laser;
           s,  c, y_laser;
           0,  0,      1];

  odometry_corrected_tracker = zeros(size(odometry_corrected));

  for i = 1:size(odometry_corrected, 1)
    T = v2t(odometry_corrected(i, :)');
    T_corrected = laser * T * inv(laser);
    odometry_corrected_tracker(i, :) = t2v(T_corrected)';
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

function plot_odometry_calibrated_error(tracker_pose, odometry_calibrated, h, time)
  hold on;
  grid on;
  title('2D error trajectory of the odometry calibrated');
  xlabel('Time in seconds');
  ylabel('L2 Norm error');

  error_odometry_calibrated = [tracker_pose(:, 1) - odometry_calibrated(:, 1), tracker_pose(:, 2) - odometry_calibrated(:, 2)];
  delta_time = time - time(1);

  L2_norm = sqrt(sum(error_odometry_calibrated.^2, 2));

  name_file = "./output/error_odometry_calibrated.png";
  if exist(name_file, "file")
    delete(name_file);
  endif

  plot(delta_time, L2_norm, 'g-', 'linewidth', 2);

  legend('Odometry calibrated: L2 Norm error');
  drawnow;
  try
    saveas(h, name_file);
  catch
    disp("Plot closed before saving is completed");
  end

  printf('L2 Norm error: mean %f, min %f, max %f.\n', mean(L2_norm), min(L2_norm), max(L2_norm));
  waitfor(h);
endfunction


function plot_theta_calibrated(tracker_pose, odometry_calibrated, h, time)
  hold on;
  grid on;
  title('Real \theta vs Calibrated \theta');
  xlabel('seconds');
  ylabel('\theta [rad]');

  delta_time = time - time(1);

  theta_tracker = tracker_pose(:, 3);
  theta_odometry = odometry_calibrated(:, 3);

  plot(delta_time, theta_tracker, 'r-', 'LineWidth', 2);
  plot(delta_time, theta_odometry, 'b--', 'LineWidth', 2);

  legend('Real \theta laser', 'Calibrated \theta laser');
  drawnow;

  name_file = "./output/theta_calibrated.png";
  if exist(name_file, "file")
    delete(name_file);
  endif

  try
    saveas(h, name_file);
  catch
    disp("Plot closed before saving is completed");
  end

  waitfor(h);
endfunction
