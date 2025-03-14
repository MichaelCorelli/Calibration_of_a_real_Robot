
%odometry and calibration functions
function delta_ticks = delta_ticks(ticks, max_encoder_value)
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

          steer_rad = steer*(360/(2*pi));
          traction_rad = traction/(2*pi);
          
          delta_ticks(i, :) = [steer_rad, traction_rad];
      end
  end
endfunction

function delta_time = delta_time(time)
  for i=1:size(time, 1)
      if i == 1
          delta_time(i, 1) = 0;
      else
          delta_time(i, 1) = time(i, 1) - time(i-1, 1);
      end
  end
endfunction

function state = odometry(x, delta_ticks, delta_time)
  ksteer = x(1);
  ktraction = x(2);
  axis_length = x(3);
  steer_offset = x(4);
  state = zeros(size(delta_ticks, 1), 4);

  for i = 1:size(delta_ticks, 1)
    ticks_steer = delta_ticks(i, 1);
    ticks_traction = delta_ticks(i, 2);

    v = (ktraction*ticks_traction);
    dphi = (ksteer*ticks_steer) + steer_offset;

    if i == 1
      delta = [0, 0, 0, 0];
    else
      dx = v * cos(state(i-1, 3))*cos(state(i-1, 4));
      dy = v * sin(state(i-1, 3))*cos(state(i-1, 4));
      dth = v * (sin(state(i-1, 4))/axis_length);

      delta = [state(i-1, 1) + dx*delta_time(i, 1), state(i-1, 2) + dy*delta_time(i, 1), state(i-1, 3) + dth*delta_time(i, 1), state(i-1, 4) + dphi*delta_time(i, 1)];
    end
    state(i, :) = delta;
  end
endfunction

%plot functions
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
    legend('Odometry', 'Tracker', 'Odometry estimated');
        
    for i = 1:size(odometry_pose, 1),
      if ishandle(h)
        set(line1, 'XData', model_pose(1:i, 1), 'YData', model_pose(1:i, 2));
        set(line2, 'XData', tracker_pose(1:i, 1), 'YData', tracker_pose(1:i, 2));
        set(line3, 'XData', odometry_pose(1:i, 1), 'YData', odometry_pose(1:i, 2));

        pause(delta_time(i));
        drawnow;
      else
        break;
      end
    end

  else
    name_file = "./output/odometry_estimated.png";
    if exist(name_file, "file")
        delete(name_file);
    end

    plot(model_pose(:, 1), model_pose(:, 2), 'k-', 'linewidth', 2);
    plot(tracker_pose(:, 1), tracker_pose(:, 2), 'b-', 'linewidth', 2);
    plot(odometry_pose(:, 1), odometry_pose(:, 2), 'r-', 'linewidth', 2);

    legend('Odometry', 'Tracker', 'Odometry estimated');
    drawnow;
    try
      saveas(h, name_file);
    catch
      disp("Plot closed before saving is completed");
    end
  end
    
  waitfor(h);
endfunction

function plot_odometry_error(model_pose, odometry_pose, moving, h, time)
  hold on;
  grid on;
  title('2D error trajectory of the odometry estimated');
  xlabel('Time in seconds');
  ylabel('L2 Norm error');

  error_odometry = [model_pose(:, 1) - odometry_pose(:, 1), model_pose(:, 2) - odometry_pose(:, 2)];
  delta_time = time - time(1);

  L2_norm = sqrt(sum(error_odometry.^2, 2));


  if moving
    line = plot(NaN, NaN, 'g-', 'linewidth', 2);
    legend({'Odometry estimated: L2 Norm error'});
        
    for i = 1:size(model_pose, 1),
      if ishandle(h)
        set(line, 'XData', delta_time(1:i), 'YData', L2_norm(1:i));

        printf('Odometry estimated: error at time %f is %f on x and %f on y\n', time(i, 1), error_odometry(i, 1), error_odometry(i, 2));
        pause(delta_time(i));
        drawnow;
      else
        break;
      end
    end

  else
    name_file = "./output/error_odometry_estimated.png";
    if exist(name_file, "file")
        delete(name_file);
    end

    plot(delta_time, L2_norm, 'g-', 'linewidth', 2);

    legend('Odometry estimated: L2 Norm error');
    drawnow;
    try
      saveas(h, name_file);
    catch
      disp("Plot closed before saving is completed");
    end
  end

  printf('Odometry estimated: error mean is %f on x and %f on y\n', mean(error_odometry(:, 1)), mean(error_odometry(:, 2)));
  waitfor(h);
endfunction

%{
function calibrated_trajectory = odometry_correction(parameters_calibration, trajectory)
	calibrated_trajectory = zeros(size(trajectory, 1), 3);

	for i = 1:size(trajectory, 1),
		u = trajectory(i,1:3)';
		uc = parameters_calibration*u;
		calibrated_trajectory(i,:) = uc;
	end
end

function parameters_calibration = odometry_calibration(trajectory)
	H = zeros(9, 9);
	b = zeros(9, 1);
	parameters_calibration = eye(3); 
	
	for i=1:size(trajectory,1),
		e = error(i, parameters_calibration, trajectory);
		J = jacobian(i, trajectory);
		H = H + J'*J;
		b = b + J'*e;
	end

	#solve the linear system
	delta_parameters_calibration = -H\b;
	dparameters_calibration=reshape(delta_parameters_calibration, 3, 3)';
	parameters_calibration = parameters_calibration + dparameters_calibration;
end

function e = error(i,X,trajectory)
	ustar=trajectory(i,1:3)';
	u=trajectory(i,4:6)';
	e= ustar-X*u;
end

function J = jacobian(i, trajectory)
	u = trajectory(i, 4:6);
	J = zeros(3,9);
	J(1, 1:3) = -u;
	J(2, 4:6) = -u;
	J(3, 7:9) = -u;
end
%}