
function [X_final, laser_f, chi_stats, n_inliers] = odometry_calibration(odometry_pose, tracker_pose, n_iter, jacobian_type)

  odometry_inc = new_pose(odometry_pose);
  tracker_inc  = new_pose(tracker_pose);

  #initial state: transformation matrix + laser offset
  state = [reshape(eye(3), [], 1); 1.5; 0; 1];
  threshold = 0.0000198; #inlier threshold
  c = 0.000000002; #initial damping factor
  d = 1.2; #damping adjustment factor

  chi_stats = zeros(1, n_iter);
  n_inliers = zeros(1, n_iter);
  min_chi = inf;
  n_count = 0;
  max_step = 3;

  for i = 1:n_iter
    [H, b, chi, inliers] = linear_s(state, odometry_inc, tracker_inc, threshold, jacobian_type);
    n_inliers(i) = inliers;

    lambda = c * max(diag(H));
    d_state = - (H + lambda*eye(length(state))) \ b;
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
    fprintf('Iteration %d: chi = %.2e, inliers = %d, laser = [%.3f, %.3f, %.3f]\n', i, chi, inliers, laser_params);

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

#linear system for state update
function [H, b, chi, inliers] = linear_s(state, odom_inc, tracker_inc, threshold, jacobian_type)
  H = zeros(length(state)); b = zeros(length(state), 1);
  chi = 0; inliers = 0;

  for i = 1:size(odom_inc, 1)

    u = odom_inc(i,:)';
    z = tracker_inc(i,:)';
    e = error(state, u, z);

    #jacobian: numerical or analytical
    if jacobian_type
      J = Jacobian_numerical(@(s) error(s, u, z), state, 1e-5);
    else
      J = Jacobian_analytical(state, u, z);
    endif

    r = e' * e;

    #weight residual: reduce outlier impact
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

#error between corrected odometry and tracker
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

  #error in laser frame
  e = t2v(inv(laser) * v2t(z) * laser) - X * u;
  e(3) = mod(e(3) + pi, 2*pi) - pi;
endfunction

#Numerical jacobian
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

#Analytical jacobian
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

#calibration correction to odometry
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

#corrected odometry to tracker frame
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
