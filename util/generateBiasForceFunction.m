function generateBiasForceFunction
  I = sym('I%d%d',[3,3]); assume(I,'real');
  h = sym('h%d',[3,1]); assume(h,'real');
  m = sym('m'); assume(m,'real');

  %quat = sym('q%d',[4,1]); assume(quat,'real');
  %E = sym('E%d%d',[3,3]); assume(E,'real');
  %r = sym('r%d',[3,1]); assume(r,'real');

  %I_B = E*(I + crossmat(r)*crossmat(h) + crossmat(h-m*r)*crossmat(r))*E';
  %h_B = E*(h-m*r);

  v_hat = sym('v%d',[6,1]); assume(v_hat,'real');
  omega = v_hat(1:3);
  v = v_hat(4:6);
  n = I*omega + cross(h,v); 
  f = m*v - cross(h,omega);
  p = [cross(omega,n) + cross(v,f); ...
       cross(omega,f)];
  %p = subs(p_tmp,E,quat2rotmat(quat));
  dp = jacobian(p(:),[I(:);h(:);v_hat]);
  keyboard
  matlabFunction(p(:),dp(:),'file',fullfile(getDrakePath(),'util','biasForce.m'), 'vars', {m, h(:), I(:), v_hat});
end

