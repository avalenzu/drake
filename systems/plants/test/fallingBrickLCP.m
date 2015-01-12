function fallingBrickLCP
dt = 0.01;
options.floating = true;
options.terrain = RigidBodyFlatTerrain();
options.enable_fastqp = false;
%p = TimeSteppingRigidBodyManipulator('FallingBrickBetterCollisionGeometry.urdf',.01,options);
p = TimeSteppingRigidBodyManipulator('FallingBrick.urdf',dt,options);
% p = TimeSteppingRigidBodyManipulator('FallingBrickContactPoints.urdf',dt,options);
options.use_bullet = false;
p_points = TimeSteppingRigidBodyManipulator('FallingBrickContactPoints.urdf',dt,options);
x0 = p.resolveConstraints([0;0;1;randn(9,1)]);

if 0
v = p.constructVisualizer();
  sys = cascade(p,v);
  [~, xtraj] = sys.simulate([0 8],x0);
  return;
end

v = p.constructVisualizer();
v.drawWrapper(0,x0);
tf = 4;
timer_points = tic;
xtraj_points = p_points.simulate([0 tf],double(x0));
T_points = toc(timer_points);
timer_bullet = tic;
xtraj = p.simulate([0 tf],x0);
T_bullet = toc(timer_bullet);
fprintf(['Real-time factor:\n\tPoints - %4.2f\tBullet - %4.2f\n', ...
         'Speed up: %4.2f\n'], ...
        tf/T_points, tf/T_bullet, T_points/T_bullet)
v.playback(xtraj);

t=xtraj.getBreaks();
x=xtraj.eval(t);
x_points=xtraj_points.eval(t);
err = x - x_points;

for t=xtraj.getBreaks()
  x_points=xtraj_points.eval(t);
  phi = p_points.contactConstraints(x_points(1:6));
  if any(phi<-0.05)
    phi
    error('penetration');
  end
end
max_xyz_err = max(max(abs(err(1:3,:))));
max_rpy_err = 180/pi*max(max(abs(err(4:6,:))));
fprintf('Max xyz error: %4.2e m\n',max_xyz_err)
fprintf('Max rpy error: %4.2e degrees\n',max_rpy_err)
assert(max_xyz_err < 1e-3)
assert(max_rpy_err < 1)

