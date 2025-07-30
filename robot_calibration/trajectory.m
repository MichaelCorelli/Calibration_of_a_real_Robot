
function delta_ticks = delta_ticks(ticks, max_encoder_value)
  delta_ticks = zeros(size(ticks, 1), 2);

  for i = 1:size(ticks, 1)
    if i == 1
        delta_ticks(i, :) = [0, 0];
    else
        diff_steer = (ticks(i, 1) - ticks(i-1, 1));
        diff_traction = (ticks(i, 2) - ticks(i-1, 2));
          
        #handle steering encoder overflow
        if abs(diff_steer) > max_encoder_value(1)/2
            if (diff_steer) < 0
                diff_steer = (max_encoder_value(1) - ticks(i-1, 1)) + ticks(i, 1);
            else
                diff_steer = (max_encoder_value(1) - ticks(i, 1)) + ticks(i-1, 1);
            endif
        endif

        #handle traction encoder overflow
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

#time differences
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
      #Simplified model
      dx = v * cos(odometry_pose(i-1, 3))*delta_time(i, 1);
      dy = v * sin(odometry_pose(i-1, 3))*delta_time(i, 1);

      #Realistic model
      #dx = v * cos(odometry_pose(i-1, 3))*cos(odometry_pose(i-1, 4))*delta_time(i, 1);
      #dy = v * sin(odometry_pose(i-1, 3))*cos(odometry_pose(i-1, 4))*delta_time(i, 1);

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