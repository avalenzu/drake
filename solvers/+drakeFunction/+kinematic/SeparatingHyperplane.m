function fcn = SeparatingHyperplane(rbm,frameA,pts_in_A)
  R1 = drakeFunction.frames.realCoordinateSpace(1);
  R3 = drakeFunction.frames.realCoordinateSpace(3);
  n_points = size(pts_in_A,2);
  position_fcn = drakeFunction.kinematic.WorldPosition(rbm,frameA,pts_in_A);
  hyperplane_fcn = drakeFunction.euclidean.SignedDistanceToVariableHyperplane(R3,n_points);
  fcn = compose(hyperplane_fcn,[drakeFunction.Identity(R3);drakeFunction.Identity(R1);position_fcn]);
end
