function signedDistanceToVariableHyperplaneTest
  R3 = drakeFunction.frames.realCoordinateSpace(3);
  n_points = 20;
  fun = drakeFunction.euclidean.SignedDistanceToVariableHyperplane(R3,n_points);
  X = 2*rand(3,n_points)-1;
  X = bsxfun(@minus,X,mean(X,2));
  alpha = rand(3,1);
  a = 1;
  [f,df] = fun([alpha;a;X(:)]);
  geval_options.grad_method = {'user','taylorvar','numerical'};
  [f,df] = geval(@(x) eval(fun,x),[alpha;a;X(:)],geval_options);

  % Now test in a nonlinear program
  d = 0;
  hyperplane_constraint = DrakeFunctionConstraint(-inf(n_points,1),-d/2*ones(n_points,1),fun);
  prog = NonlinearProgram(3*n_points+3+1);
  prog = prog.addConstraint(ConstantConstraint(X),5:(3*n_points+3+1));
  prog = prog.addConstraint(hyperplane_constraint);
  prog = prog.addConstraint(QuadraticConstraint(0.5,0.5,eye(3),[0;0;0]),1:3);
  prog = prog.addCost(LinearConstraint(-Inf,Inf,1),4);
  [x,F,info] = prog.solve(rand(prog.num_vars,1));
  snoptInfo(info);
  valuecheck(x(5:end),X(:));
  fprintf('a = %f\n',x(4));

  lcmgl = LCMGLClient();
  lcmgl.glDrawAxes();
  lcmgl.glColor3f(1,0,0);
  for pt = X
    lcmgl.sphere(pt,0.02,20,20);
  end
  lcmgl.drawVector3d([0;0;0],x(4)*x(1:3));
  lcmgl.switchBuffers();
end
