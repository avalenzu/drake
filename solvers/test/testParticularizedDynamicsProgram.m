vertices = [diag([0.4, 0.4, 0.4])];
%vertices = [zeros(3,1), diag([0.4, 0.4, 0.4])];
prog = ParticularizedDynamicsProgramLotsOfBinary.fromVertices(vertices, 10, 0.05,'dodecahedron');
%prog = prog.addInitialVelocityConstraints([0;0;0],1);
%prog = prog.addInitialVelocityConstraints([0;0;0],2);
%prog = prog.addInitialVelocityConstraints([0;0;0],3);
%prog = prog.addInitialVelocityConstraints([0;0;0],4);
prog = prog.addInitialVelocityConstraints([0;1;-1],1);
prog = prog.addInitialVelocityConstraints([-1;0;1],2);
prog = prog.addInitialVelocityConstraints([1;-1;0],3);
%prog = prog.addInitialCOMPositionConstraint([0;0;0]);
prog = prog.addParticlePositionConstraint(vertices, 1);
[prog, solvertime, objval] = solve(prog);
v = prog.constructVisualizer();
xtraj = prog.extractXtraj();

