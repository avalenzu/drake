function stackedBricks(N)
  if nargin < 1, N = 2; end
  dt = 0.01;
  options.floating = true;
  options.terrain = RigidBodyFlatTerrain();
  %options.enable_fastqp = false;
%   options.use_bullet = false;
  options.ignore_self_collisions = true;
  p = TimeSteppingRigidBodyManipulator([],dt,options);
  for i = 1:N
    p = p.addRobotFromURDF('StackingBrick.urdf',[],[],options);
  end
  height = 0.5;
  width = 2;
  spacing = 0.5;
  x0 = zeros(p.getNumStates(), 1);
  x0(3:6:p.getNumPositions()) = 0.5*height + (height/2:spacing*height:N*spacing*height);
  x0(2:12:p.getNumPositions()) = 0.4*width;
  x0(8:12:p.getNumPositions()) = -0.4*width;
  x0 = p.resolveConstraints(x0);
  v = p.constructVisualizer();
  v.draw(0,x0);
  sys = cascade(p,v);
  tf = 10;
  [~, xtraj] = sys.simulate([0 tf],x0);
end
