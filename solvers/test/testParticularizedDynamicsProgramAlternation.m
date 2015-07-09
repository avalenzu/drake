clear g_array v_array
vertices = [diag([0.4, 0.4, 0.4])];
N = 20;
%vertices = [zeros(3,1), diag([0.4, 0.4, 0.4])];
r_array = repmat(reshape(vertices, [], 1), [1, N]);
%r_array = r_array + 0.01*randn(size(r_array));


v_position = {};
v_force_velocity = {};
xtraj_position = {};
xtraj_force_velocity = {};
M = 5;
for iter = 1:M
  prog = ParticularizedDynamicsProgramFixedPositions.fromVertices(vertices, N, 0.05, r_array);
  if exist('v_array', 'var')
    for j = 1:size(vertices,2)
      for n = 1:prog.N
        prog.vars.(prog.velocityName(j, n)).start = v_array(:,j);
      end
    end
  end
  if exist('g_array', 'var')
    for e = prog.E
      i = e(1);
      j = e(2);
      for n = 1:prog.N
        prog.vars.(prog.forceName(i, j, n)).start = g_array(i,j,n);
      end
    end
  end

  prog = prog.addInitialVelocityConstraints([0;1;-1],1);
  prog = prog.addInitialVelocityConstraints([-1;0;1],2);
  prog = prog.addInitialVelocityConstraints([1;-1;0],3);d
  [prog, solvertime, objval] = solve(prog);
  v_position{end+1} = prog.constructVisualizer();
  xtraj_position{end+1} = prog.extractXtraj();
  fprintf('Fixed position cost: %f\n', objval);

  v_array = prog.extractVData();
  g_array = prog.extractFData();
  g_array(g_array < sqrt(eps)) = 0;
  v_array(v_array < sqrt(eps)) = 0;
  prog_2 = ParticularizedDynamicsProgramFixedForces.fromVertices(vertices, N, 0.05, v_array, g_array);
  prog_2 = prog_2.addParticlePositionConstraint(vertices, 1);
  for j = 1:size(vertices,2)
    for n = 1:prog_2.N
      prog_2.vars.(prog_2.positionName(j, n)).start = r_array(:,j);
    end
  end
  [prog_2, solvertime, objval] = solve(prog_2);
  v_force_velocity{end+1} = prog_2.constructVisualizer();
  xtraj_force_velocity{end+1} = prog_2.extractXtraj();
  r_array = prog_2.extractRData();
  fprintf('Fixed force/velocity cost: %f\n', objval);
end
