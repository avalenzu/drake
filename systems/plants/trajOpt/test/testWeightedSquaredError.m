function testWeightedSquaredError
  nT = 5;
  ny = 3;
  Q = diag(rand(ny,1));
  y_nom = rand(ny,nT);
  t = sort(rand(nT,1));
  y_nom_traj = PPTrajectory(foh(t,y_nom));
  constr = WeightedSquaredError(t,Q,y_nom_traj);
  constr.checkGradient(1e-7,y_nom(:));
end
