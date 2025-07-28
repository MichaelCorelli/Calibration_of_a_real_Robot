
function A = v2t(v)
  c = cos(v(3));
  s = sin(v(3));
  A = [c, -s, v(1);
       s,  c, v(2);
       0,  0,   1];
endfunction

function v = t2v(A)
  v = [A(1, 3);
       A(2, 3); 
       atan2(A(2, 1), A(1, 1))];
endfunction

function new = new_pose(pose)
  new = zeros(size(pose, 1) - 1, 3);

  for i = 1:size(pose, 1) - 1
    v = t2v(inv(v2t(pose(i,:)')) * v2t(pose(i + 1,:)'));
    v(3) = mod(v(3) + pi, 2*pi) - pi;
    new(i, :) = v';
  endfor
endfunction

function chi = chi_tot(state, odom_inc, tracker_inc, threshold)
  chi = 0;
  
  for i = 1:size(odom_inc, 1)
    u = odom_inc(i,:)'; z = tracker_inc(i,:)';
    e = error(state, u, z);
    r = e' * e;
    chi = chi + min(r, threshold);
  endfor
endfunction