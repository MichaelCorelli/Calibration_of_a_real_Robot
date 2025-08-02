
function [X_final, laser_f, axis_length_final, chi_stats, n_inliers] = odometry_calibration(odometry_pose, tracker_pose, n_iter, jacobian_type)

  odometry_inc = new_pose(odometry_pose);
  tracker_inc  = new_pose(tracker_pose);

  Ksteer = 0.1;
  Ktraction = 0.0106141;
  steer_offset = 0;
  axis_length = 1.4;

  R = [cos(steer_offset), -sin(steer_offset), 0;
      sin(steer_offset),  cos(steer_offset), 0;
      0,                 0,                  1];

  S = [Ktraction, 0, 0;
      0, Ksteer, 0;
      0, 0, 1];

  X_init = R * S;
  X_init(1,3) = 0;
  X_init(2,3) = 0;

  #initial state
  state = [reshape(X_init, [], 1); 1.5; 0; 1; axis_length]; 

  threshold = 0.0000198;  #inlier threshold
  c = 0.000000003; #initial damping factor
  d = 1.2; #damping adjustment factor

  chi_stats = zeros(1, n_iter);
  n_inliers = zeros(1, n_iter);
  chi_min = inf;
  n_count = 0;
  max_step = 3;

  chi_initial = chi_tot(state, odometry_inc, tracker_inc, threshold, axis_length);
  fprintf('Initial chi: %.6e\n', chi_initial);

  for i = 1:n_iter
    [H, b, chi, inliers] = linear_s(state, odometry_inc, tracker_inc, threshold, jacobian_type, axis_length);
    n_inliers(i) = inliers;

    lambda = c * max(diag(H));
    d_state = - (H + lambda*eye(length(state))) \ b;
    state_new = state + d_state;

    if state_new(13) <= 0
      state_new(13) = 0.1;
    endif

    chi_new = chi_tot(state_new, odometry_inc, tracker_inc, threshold, axis_length);

    if chi_new < chi
      state = state_new;
      chi = chi_new;
      c = c / d;
      n_count = (chi < chi_min) * 0 + (chi >= chi_min) * (n_count + 1);
      chi_min = min(chi_min, chi);
    else
      c = c * d;
      n_count = n_count + 1;
      fprintf('Iter %d: step refused, c = %e\n', i, c);
    endif

    chi_stats(i) = chi;
    
    X_m = reshape(state(1:9), 3, 3);
    laser_params = state(10:12);
    axis_length_current = state(13);

    steer_offset_cal = atan2(X_m(2,1), X_m(1,1));

    R = [cos(steer_offset_cal), -sin(steer_offset_cal), 0;
         sin(steer_offset_cal),  cos(steer_offset_cal), 0;
         0,                                          0, 1];

    S_calibrated = R' * X_m;

    ktraction_calibrated = S_calibrated(1,1);  
    ksteer_calibrated = S_calibrated(2,2);     
    ktheta_calibrated = S_calibrated(3,3);     

    fprintf('Iter %d: chi = %.2e, inliers = %d\n', i, chi, inliers);
    
    if norm(d_state) < 1e-7 && i > 1 && abs(chi_stats(i) - chi_stats(i-1)) < 1e-7
      fprintf('Converged at iter %d: chi = %.6e\n', i, chi);
      break;

    elseif n_count >= max_step && i > 10
      fprintf('Stopping at iter %d: chi = %.6e\n', i, chi);
      break;

    endif
  endfor

  X_final = reshape(state(1:9), 3, 3);
  laser_f = state(10:12);
  axis_length_final = state(13);
endfunction

#linear system for state update
function [H, b, chi, inliers] = linear_s(state, odom_inc, tracker_inc, threshold, jacobian_type, axis_length)
  H = zeros(length(state)); 
  b = zeros(length(state), 1);
  chi = 0; 
  inliers = 0;

  for i = 1:size(odom_inc, 1)
    u = odom_inc(i,:)';
    z = tracker_inc(i,:)';
    e = error(state, u, z, axis_length);
    #jacobian: numerical or analytical
    if jacobian_type      
      J = Jacobian_numerical(@(s) error(s, u, z, axis_length), state);
    else
      J = Jacobian_analytical(state, u, z, axis_length);
    endif

    r = e' * e;

    #weight residual: reduce outlier impact
    if r <= threshold
      w = 1; 
      inliers = inliers + 1;
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
function e = error(state, u, z, axis_length)
  X = reshape(state(1:9), 3, 3);
  x_laser = state(10);
  y_laser = state(11);
  theta_laser = state(12);
  axis_length_new = state(13);
    
  c = cos(theta_laser);
  s = sin(theta_laser);
  laser = [c, -s, x_laser;
           s,  c, y_laser;
           0,  0,       1];

  u_corrected = u;
  u_corrected(3) = u(3) * axis_length_new / axis_length;

  e = t2v(inv(laser) * v2t(z) * laser) - X * u_corrected;
  e(3) = mod(e(3) + pi, 2*pi) - pi;
endfunction

#Analytical jacobian
function J = Jacobian_analytical(state, u, z, axis_length)
  X = reshape(state(1:9), 3, 3);
  x_laser = state(10);
  y_laser = state(11);
  theta_laser = state(12);
  axis_length_new = state(13);
    
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

  u_corrected = u;
  u_corrected(3) = u(3) * axis_length_new / 1.4;

  J = zeros(3, 13);
  J(:, 1:3) = -u_corrected(1) * eye(3);
  J(:, 4:6) = -u_corrected(2) * eye(3);
  J(:, 7:9) = -u_corrected(3) * eye(3);
    
  J(:, 10) = t2v((-inv(laser) * dlaser_x * inv(laser)) * v2t(z) * laser + (inv(laser) * v2t(z) * dlaser_x));
  J(:, 11) = t2v((-inv(laser) * dlaser_y * inv(laser)) * v2t(z) * laser + (inv(laser) * v2t(z) * dlaser_y));
  J(:, 12) = t2v((-inv(laser) * dlaser_theta * inv(laser)) * v2t(z) * laser + (inv(laser) * v2t(z) * dlaser_theta));
  J(:, 13) = -X(:, 3) * u(3) / axis_length;
endfunction

#Numerical jacobian
function J = Jacobian_numerical(f, state, iteration)

  if nargin < 3
    iteration = 1;
  endif

  d_rel = 1e-6 * max(0.3, 1.0 / sqrt(iteration));
  
  n = numel(state);
  f0 = f(state);
  J = zeros(numel(f0), n);
    
  for k = 1:n
      if abs(state(k)) > 1e-10
          d_abs = abs(state(k)) * d_rel;
      else
          d_abs = d_rel;
      end
      
      max_step = 1e-3 / sqrt(iteration);
      d_abs = max(min(d_abs, max_step), 1e-12);
      
      state_p = state;
      state_m = state;
      state_p(k) = state_p(k) + d_abs;
      state_m(k) = state_m(k) - d_abs;
        
      J(:, k) = (f(state_p) - f(state_m))/(2 * d_abs);
  endfor
endfunction

function chi = chi_tot(state, odom_inc, tracker_inc, threshold, axis_length)
  chi = 0;
  for i = 1:size(odom_inc, 1)
    u = odom_inc(i,:)';
    z = tracker_inc(i,:)';
    e = error(state, u, z, axis_length);
    r = e' * e;
    chi = chi + min(r, threshold);
  endfor
endfunction

function new = new_pose(pose)
  new = zeros(size(pose, 1) - 1, 3);

  for i = 1:size(pose, 1) - 1
    v = t2v(inv(v2t(pose(i,:)')) * v2t(pose(i + 1,:)'));
    v(3) = mod(v(3) + pi, 2*pi) - pi;
    new(i, :) = v';
  endfor
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