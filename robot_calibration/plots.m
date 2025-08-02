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
  ylim([min(min(odometry_calibrated(:, 2)), min(tracker_pose(:, 2))) - lim_offset max(max(odometry_calibrated(:, 2)), max(tracker_pose(:, 2))) + lim_offset]);
    
  if moving
    line1 = plot(NaN, NaN, 'r-', 'linewidth', 2);
    line2 = plot(NaN, NaN, 'b-', 'linewidth', 2);
    legend('Odometry calibrated pose', 'Tracker pose');
        
    for i = 1:size(odometry_calibrated, 1)
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
  plot(delta_time, theta_odometry, 'b-', 'LineWidth', 2);

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