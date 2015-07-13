%clear *_seed
N = 15;
M = 5;
lam = 1;
 %vertices = [diag([0.4, 0.4, 0.4])];
vertices = [zeros(3,1), diag([0.4, 0.4, 0.4])];
r_seed = repmat(reshape(vertices, [], 1), [1, N]);
r_array = r_seed;
contact_flag_seed = repmat({NaN(1, N)},size(vertices,2),1);
for iter = 1:M
%   lam = 1/iter;
  prog = ParticularizedDynamicsProgramTwoPositions.fromVertices(vertices, N, 0.03,r_array);
  for k = 1:prog.n_regions
    prog.vars.(prog.contactName(k)).start = contact_flag_seed{k};
  end
  if exist('r_seed', 'var')
    for j = 1:size(vertices,2)
      for n = 1:prog.N
        prog.vars.(prog.positionName(j, n)).start = r_seed(:,j);
      end
    end
  end
  if exist('v_seed', 'var')
    for j = 1:size(vertices,2)
      for n = 1:prog.N
        prog.vars.(prog.velocityName(j, n)).start = v_seed(:,j);
      end
    end
  end
  if exist('g_seed', 'var')
    for e = prog.E
      i = e(1);
      j = e(2);
      for n = 1:prog.N
        prog.vars.(prog.forceName(i, j, n)).start = g_seed(i,j,n);
      end
    end
  end
  %prog.mass(2) = 2;
  prog = prog.addInitialVelocityConstraints([0;0;0],1);
  %prog = prog.addInitialVelocityConstraints([0;0;0],2);
  %prog = prog.addInitialVelocityConstraints([0;0;0],3);
  %prog = prog.addInitialVelocityConstraints([0;0;0],4);
  prog = prog.addInitialVelocityConstraints([0;1;-1],2);
  prog = prog.addInitialVelocityConstraints([-1;0;1],3);
  prog = prog.addInitialVelocityConstraints([1;-1;0],4);
  %prog = prog.addInitialCOMPositionConstraint([0;0;0]);
  prog = prog.addParticlePositionConstraint(bsxfun(@plus, [0; 0; 0.2], vertices), 1);
  [prog, solvertime, objval] = solve(prog);
  v = prog.constructVisualizer();
  xtraj = prog.extractXtraj();
  for k = 1:prog.n_regions
    contact_flag_seed{k} = prog.vars.(prog.contactName(k)).value;
  end
  v_seed = prog.extractVData();
  g_seed = prog.extractFData();
  g_seed(abs(g_seed) < sqrt(eps)) = 0;
  v_seed(abs(v_seed) < sqrt(eps)) = 0;
  r_array_new = (1-lam)*r_array + lam*prog.extractRData();
  r_array_new(abs(r_array_new) < sqrt(eps)) = 0;
  r_seed = r_array_new;
  fprintf('Iteration %3d norm of delta r: %f\n', iter, norm(r_array_new(:)-r_array(:),2));
  prog.plotAngularMomentum
  drawnow
  r_array = r_array_new;
end

