function generateSpatialInertiaTimesSpatialVelocityFunction
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
  I_hat_v_hat = [ I*omega + cross(h,v); ...
                  m*v - cross(h,omega) ];
  dI_hat_v_hat = jacobian(I_hat_v_hat(:),[h(:);I(:);v_hat]);
  keyboard
  matlabFunction(I_hat_v_hat(:),dI_hat_v_hat(:),'file',fullfile(getDrakePath(),'util','spatialInertiaTimesSpatialVelocity.m'), 'vars', {m, h(:), I(:), v_hat});
end

