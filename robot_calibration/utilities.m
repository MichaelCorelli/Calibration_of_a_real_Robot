
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
