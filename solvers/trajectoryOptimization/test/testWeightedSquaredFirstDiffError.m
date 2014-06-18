function testWeightedSquaredFirstDiffError
  nT = 5;
  ny = 3;
  Q = diag(rand(ny,1));
  y_nom = rand(ny,nT);
  y_test = rand(ny,nT);
  t = linspace(0,1,nT)+1/nT*(rand(1,nT)-0.5);
  y_nom_traj = PPTrajectory(foh(t,y_nom));
  constr = WeightedSquaredFirstDiffError(t,Q,y_nom_traj);
  %constr.checkGradient(1e-5,y_test(:));
  [c,dc] = geval(@constr.eval,y_test(:),struct('grad_method',{{'user','taylorvar'}}));
  [c,dc] = geval(@constr.eval,y_test(:), ...
    struct('grad_method',{{'user','numerical'}},'tol',1e-5));
end
