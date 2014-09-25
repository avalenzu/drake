function xstar = pointInPolytopeTest()
  n = 10;
  N = 200;
  draw_time = 10;
  goal_circle_radius = 2;
  R2 =  drakeFunction.frames.realCoordinateSpace(2);
  Rn = drakeFunction.frames.realCoordinateSpace(n);
  lcmgl = LCMGLClient('pointInPolytopeTest');
  origins = 2*rand(2,n)-1;
  normals = rand(2,n); 
  normals = bsxfun(@rdivide,normals,sqrt(sum(normals.^2,1)));
  normals(:,dot(origins,normals,1)<0) = -normals(:,dot(origins,normals,1)<0);
  A = normals';
  b = -dot(normals,origins)';
  polytope = drakeFunction.Affine(R2,Rn,A,b);
  in_polytope_constraint = DrakeFunctionConstraint(-Inf(n,1),zeros(n,1),polytope);
  %goal_point = [10;10];
  prog = NonlinearProgram(2);
  prog = prog.addConstraint(in_polytope_constraint);
  %prog = prog.addDisplayFunction(@(x) displayCallback(x,lcmgl,goal_point));
  goal_point = goal_circle_radius*[cos(linspace(0,2*pi,N));sin(linspace(0,2*pi,N))];
  x0 = zeros(2,1);
  xstar = zeros(2,N);
  for i = 1:N
    objective = DrakeFunctionConstraint(-Inf,Inf,compose(drakeFunction.euclidean.NormSquared(R2),drakeFunction.Affine(R2,R2,eye(2),-goal_point(:,i))));
    prog_i = prog.addCost(objective);
    xstar(:,i) = prog_i.solve(x0);
    lcmgl.glColor4f(0,1,0,0.5);
    lcmgl.sphere([goal_point(:,i);0],0.2,20,20);
    lcmgl.glColor3f(0,0,1);
    for j = 1:i
      lcmgl.sphere([xstar(:,j);0],0.02,20,20);
    end
    lcmgl.switchBuffers();
    pause(draw_time/N);
  end
end
