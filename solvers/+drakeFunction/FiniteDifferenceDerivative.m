function obj = FiniteDifferenceDerivative(frame,N)
  difference_fun = drakeFunction.Difference(frame,N);
  time_frame = drakeFunction.frames.realCoordinateSpace(N-1);
  duplicated_time_frame = MultiCoordinateFrame(repmat({frame},1,N-1));
  A = kron(speye(N-1),ones(frame.dim,1));
  recip_time_fun = compose(drakeFunction.ConstantPower(duplicated_time_frame,-1),drakeFunction.Linear(time_frame,duplicated_time_frame,A));
  obj = recip_time_fun.*difference_fun;
end
