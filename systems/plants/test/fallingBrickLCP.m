function fallingBrickLCP
  rng(4);
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
xtraj_points = p_points.simulate([0 4],double(x0));
xtraj = p.simulate([0 4],x0);
v.playback(xtraj);

t=xtraj.getBreaks();
x=xtraj.eval(t);
x_points=xtraj_points.eval(t);
err = x - x_points;

for t=xtraj.getBreaks()
  x=xtraj.eval(t);
  x_points=xtraj_points.eval(t);
%   assert(max(abs(x - x_points)) < 1e-3);
%   fprintf('t = %6.4f\t\tMax error: %6.4e\n',t,max(abs(x - x_points)))
  phi = p.contactConstraints(x(1:6));
  if any(phi<-0.05)
    phi
    error('penetration');
  end
end
fprintf('Max error: %6.4e\n',max(max(abs(err(1:3,:)))))
fprintf('Max error: %6.4e\n',max(max(abs(err(4:6,:)))))

