I = eye(3);
m = 1;
N = 50;
M = 20;
dt = 1/N;
w0 = [1; 1; 1];
w_fixed_array = rand(3, N);
lam = 1;
tol = 1e-6;

for i = 1:M
  prog = StateAlternatingDynamicsProgram(I, m, N, dt, w_fixed_array);
  prog = prog.addOrientationConstraint(1, [1; 0; 0; 0]);
  prog = prog.addPositionConstraint(1, [0; 0; 0], [0; 0; 0]);
  prog = prog.addSymbolicConstraints(prog.vars.w.symb(:,1) == w0);
  prog = prog.addSymbolicConstraints(prog.vars.v.symb(:,1) == 0);
  [prog, solvertime, objval] = prog.solve();
  w_fixed_array = (1-lam)*w_fixed_array + lam*prog.vars.w.value;
  fprintf('Objective Value: %f\n', objval);
  if objval < tol, break; end
end

options.floating = true;
urdf = fullfile(getDrakePath(), 'systems', 'plants', 'test', 'FallingBrick.urdf');
rbm = RigidBodyManipulator(urdf, options);
v = rbm.constructVisualizer();
t = cumsum(repmat(prog.dt, [1, prog.N]));
rpy_value = zeros(3, size(prog.vars.z.value, 2));
for i = 1:size(rpy_value, 2)
  rpy_value(:, i) = quat2rpy(prog.vars.z.value(:, i));
end
position_traj = PPTrajectory(foh(t, [prog.vars.r.value; rpy_value]));
position_traj = position_traj.setOutputFrame(rbm.getPositionFrame());