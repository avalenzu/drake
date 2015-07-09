N = 20;
M = 1;
lam = 0.5;
% vertices = [diag([0.4, 0.4, 0.4])];
vertices = [zeros(3,1), diag([0.4, 0.4, 0.4])];
r_array = repmat(reshape(vertices, [], 1), [1, N]);
for iter = 1:M
%   lam = 1/iter;
  prog = ParticularizedDynamicsProgramTwoPositions.fromVertices(vertices, N, 0.05,r_array);
  prog.mass(2) = 2;
  %prog = prog.addInitialVelocityConstraints([0;0;0],1);
  %prog = prog.addInitialVelocityConstraints([0;0;0],2);
  %prog = prog.addInitialVelocityConstraints([0;0;0],3);
  %prog = prog.addInitialVelocityConstraints([0;0;0],4);
  prog = prog.addInitialVelocityConstraints([0;1;2],2);
  prog = prog.addInitialVelocityConstraints([-1;0;4],3);
  prog = prog.addInitialVelocityConstraints([1;-1;3],4);
  %prog = prog.addInitialCOMPositionConstraint([0;0;0]);
  prog = prog.addParticlePositionConstraint(bsxfun(@plus, [0; 0; 1], vertices), 1);
  [prog, solvertime, objval] = solve(prog);
  v = prog.constructVisualizer();
  xtraj = prog.extractXtraj();
  r_array_new = (1-lam)*r_array + lam*prog.extractRData();
  fprintf('Iteration %3d norm of delta r: %f\n', iter, norm(r_array_new(:)-r_array(:),2));
  prog.plotAngularMomentum
  drawnow
  r_array = r_array_new;
end

