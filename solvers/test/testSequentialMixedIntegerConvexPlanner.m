if ~exist('continuation', 'var') || ~continuation
  clear R_seed RR_seed
  options = struct();
  options.floating = 'quat';
  options.use_new_kinsol = true;
  urdf = fullfile(getDrakePath(), 'solvers', 'test', 'littleBrick.urdf');
  particle_urdf = fullfile(getDrakePath(), 'systems', 'plants', 'test', 'PointMass.urdf');
  rbm = RigidBodyManipulator(urdf, options);
  rbm_vis = rbm;
  step_height = 0.0;
  platform1_start = -0.5;
  platform1_end = 0.3;
  platform2_start = 0.5;
  platform2_end = 1.5;
  platform1 = RigidBodyBox([platform1_end-platform1_start; 1; 0.1], [(platform1_end+platform1_start)/2; 0; -0.05], [0; 0; 0]);
  platform1.c = [0.2; 0.2; 0.2];
  platform2 = RigidBodyBox([platform2_end-platform2_start; 1; 0.1], [(platform2_end+platform2_start)/2; 0; step_height-0.05], [0; 0; 0]);
  platform2.c = [0.2; 0.2; 0.2];
  rbm_vis = rbm_vis.addVisualGeometryToBody(1, platform1);
  rbm_vis = rbm_vis.addVisualGeometryToBody(1, platform2);
  colormap('lines')
  colors = colormap';
  colormap('default')
  options.collision = false;
  for j = 1:4
    rbm_vis = rbm_vis.addRobotFromURDF(particle_urdf, [], [], options);
    body = rbm_vis.body(end);
    body.visual_geometry{1} = body.visual_geometry{1}.setColor(colors(:,j));
    body.visual_geometry{1}.radius = 0.02;
    rbm_vis = rbm_vis.setBody(rbm_vis.getNumBodies(), body);
  end
  rbm_vis = rbm_vis.compile();
  v = rbm_vis.constructVisualizer();
  I = diag([1;6;3]);
  m = 1;
  N = 16;
  M = 1000;
  dt = 2/N;
  r0 = [0; 0; 0.5];
  z0 = rpy2quat([0; 0*pi/180; 0]);
  zf = rpy2quat([0; 0*pi/180; 60*pi/180]);
  w0 = 0*[1e-3; 0; 0];
  v0 = 0*[0.5; 0; 0];
  w_fixed_array = 0*rand(3, N);
  z_fixed_array = repmat(z0, 1, N) + 0.0*randn(4, N);
  %z_fixed_array = linspacevec(z0, zf, N);
  z_fixed_array = bsxfun(@rdivide, z_fixed_array, sqrt(sum(z_fixed_array.^2,1)));
  M_fixed_array = zeros(3, N, 4);
  F_fixed_array = zeros(3, N, 4);
  lam = 1;
  tol = 1e-6;
  v_des = 0.1;
  r_foot_fixed_array = zeros(3, N, 4);
  for j = 1:4 
    pt = rbm.getTerrainContactPoints(2).pts(:,2+j);
    r_foot_fixed_array(:, :, j) = repmat(pt, [1, N]);
  end
  fix_forces = false;
  slack_max = 1e3;
end

total_solvertime = 0;
total_time = tic;
leg_length = 0.1;
for i = 1:M
  prog = SequentialMixedIntegerConvexPlanner(I, m, N, dt, z_fixed_array, w_fixed_array, F_fixed_array, M_fixed_array, false);
  prog.force_max = prog.m*9.81/2;
  prog.velocity_max = 2;
  prog.position_max = 1e1;
  prog.stance_velocity_max = 0.1;
  prog.swing_velocity_max = 1;
  prog.fix_forces = fix_forces;
  prog.slack_max = slack_max;
  for j = 1:4 
    pt = rbm.getTerrainContactPoints(2).pts(:,2+j);
    centers = [pt, pt + [0; 1*sign(pt(2))*leg_length*sin(0*pi/180); -1.5*leg_length*cos(0*pi/180)]];
    %centers = [pt+[0; -0.5*leg_length + sign(pt(2))*0.1*leg_length; -leg_length], pt+[0; 0.5*leg_length + sign(pt(2))*0.1*leg_length; -leg_length]];
    prog = prog.addFoot(centers, [leg_length, leg_length], r_foot_fixed_array(:,:,j));
  end
  %prog = prog.addRegion([0, 0, -1; 1, 0, 0], [-0.05; 0.7], [], [], [], []);
  %prog = prog.addRegion([0, 0, -1; -1, 0, 0], [-(step_height + 0.05); -0.7], [], [], [], []);
  prog = prog.addRegion([0, 0, -1], -0.05, [], [], [], []);
  %prog = prog.addRegion([0, 0, 1], 0, [], [], [], []);
  %prog = prog.addRegion([0, 0, 1; 0, 0, -1], [0; 0.06], [], [], [0; 0; 1], 1);
  %prog = prog.addRegion([0, 0, 1; 0, 0, -1], [0; 0.05], [], [], [0; 0; 1], 1);
  prog = prog.addRegion([1, 0, 0], platform1_end, [0, 0, 1], 0, [0; 0; 1], 1);
  prog = prog.addRegion([-1, 0, 0], -platform2_start, [0, 0, 1], step_height, [0; 0; 1], 1);
  prog = prog.addDefaultConstraintsAndCosts();
  if exist('R_seed', 'var') && exist('RR_seed', 'var')
    for j = 1:numel(prog.feet)
      prog.vars.(sprintf('R%d',j)).start = R_seed(:,:,j);
      prog.vars.(sprintf('RR%d',j)).start = RR_seed(:,:,j);
    end
  end
  %for j = 1:4
    %prog = prog.addSymbolicConstrints(prog.vars.(sprintf('r_foot%d',j)).symb(:,1) == prog.vars.(sprintf('r_foot%d',j)).symb(:,prog.N));
  %end
  %prog = prog.addOrientationConstraint(1, [1; 0; 0; 0]);
  prog = prog.addOrientationConstraint(1, z0);
  %prog = prog.addOrientationConstraint(N, zf);
  prog = prog.addPositionConstraint(1, r0 - [0; 0; 0.5], r0 + [0; 0; 0.8]);
  %prog = prog.addSymbolicConstraints(prog.vars.z.symb(:,1) == prog.vars.z.symb(:,prog.N));
  %prog = prog.addSymbolicConstraints(prog.vars.r.symb(3,1) == prog.vars.r.symb(3,prog.N));
  prog = prog.addPositionConstraint(N, r0 + [1; -1; -0.5], r0 + [1; 1; 0.8]);
  %prog = prog.addSymbolicConstraints(0.5 <= prog.vars.r.symb(3,1) <= 1.1);
  %prog = prog.addSymbolicConstraints(prog.vars.r.symb(1:2,1) == 0);
  prog = prog.addAngularVelocityConstraint([1,N], 0, 0);

  prog = prog.addVelocityConstraint([1,N], 0, 0);
  %Aeq = zeros(1, prog.nv);
  %beq = pi/6*prog.N;
  %Aeq(1, prog.vars.w.i(3,:)) = 1;
  %prog = prog.addLinearConstraints([], [], Aeq, beq);
  

  % Don't transition directly from one contact region to another
  ncons = 2*(prog.N-1)*numel(prog.feet);
  A = zeros(ncons, prog.nv);
  b = ones(ncons, 1);
  offset = 0;
  for j = 1:numel(prog.feet)
    indices = offset + (1:prog.N-1);
    R2_indices = prog.vars.(sprintf('R%d',j)).i(2,1:prog.N-1);
    R3_indices = prog.vars.(sprintf('R%d',j)).i(3,2:prog.N);
    A(indices, R2_indices) = eye(prog.N-1);
    A(indices, R3_indices) = eye(prog.N-1);
    offset = offset + prog.N - 1;
    indices = offset + (1:prog.N-1);
    R2_indices = prog.vars.(sprintf('R%d',j)).i(2,2:prog.N);
    R3_indices = prog.vars.(sprintf('R%d',j)).i(3,1:prog.N-1);
    A(indices, R2_indices) = eye(prog.N-1);
    A(indices, R3_indices) = eye(prog.N-1);
    offset = offset + prog.N - 1;
  end
  prog = prog.addLinearConstraints(A, b, [], []);

  % Start and end in a contact region
  ncons = 2;
  A = zeros(ncons, prog.nv);
  b = zeros(ncons, 1);
  for j = 1:numel(prog.feet)
    A(1, prog.vars.(sprintf('R%d',j)).i(1,1)) = 1;
    A(2, prog.vars.(sprintf('R%d',j)).i(1,prog.N)) = 1;
  end
  prog = prog.addLinearConstraints(A, b, [], []);

  ncons = prog.N;
  A = zeros(ncons, prog.nv);
  b = 2*ones(ncons, 1);
  for j = 1:numel(prog.feet)
    A(:, prog.vars.(sprintf('R%d',j)).i(1,:)) = eye(ncons);
  end
  prog = prog.addLinearConstraints(A, b, [], []);

  Q = zeros(prog.nv);
  c = zeros(prog.nv, 1);
  zf_indices = prog.vars.z.i(:, N);
  Q(zf_indices, zf_indices) = 1e3*eye(4);
  c(zf_indices) = -2*zf;
  alpha = sum(zf.^2);
  %prog = prog.addCost(Q, c, alpha);
  %prog = prog.addSymbolicConstraints(sum(prog.vars.w.symb(3,:)) == pi/2*prog.N);
  %prog = prog.addVelocityConstraint([1,N], 0, 0);
  %prog = prog.addSymbolicConstraints(prog.vars.v.symb(3,[1,N]) == 0);
  %prog = prog.addSymbolicConstraints(prog.vars.v.symb(1,1) == prog.vars.v.symb(1,prog.N));
  %prog = prog.addSymbolicConstraints(sum(prog.vars.v.symb(1,:)) == v_des*prog.N);
  %for j = 1:size(prog.contact_pts,2)
    %prog.vars.(sprintf('F%d',j)).lb(:,prog.N) = 0;
    %prog.vars.(sprintf('F%d',j)).ub(:,prog.N) = 0;
    %prog.vars.(sprintf('M%d',j)).lb(:,prog.N) = 0;
    %prog.vars.(sprintf('M%d',j)).ub(:,prog.N) = 0;
  %end
  params = struct();
  %params.mipgap = 0.2;
  params.outputflag = 1;
  %params.solver = 'gurobi';
  params.threads = 12;
 [prog, solvertime, objval] = prog.solveGurobi(params);
  %[prog, solvertime, objval] = prog.solve();
  delta_norm = sum((w_fixed_array(:) - prog.vars.w.value(:)).^2) ...
                + sum((z_fixed_array(:) - prog.vars.z.value(:)).^2);
  total_solvertime = total_solvertime + solvertime;
  w_fixed_array = (1-lam)*w_fixed_array + lam*prog.vars.w.value;
  w_fixed_array(abs(w_fixed_array) < 1e-6) = 0;
  z_fixed_array = (1-lam)*z_fixed_array + lam*prog.vars.z.value;
  for n = 2:prog.N
    z_fixed_array(:, n) = quatProduct(z_fixed_array(:, n-1), ...
                                      expmap2quat(prog.dt*0.5*sum(w_fixed_array(:,n-1:n),2)));
  end
  %z_fixed_array = bsxfun(@rdivide, z_fixed_array, sqrt(sum(z_fixed_array.^2, 1)));
  %z_fixed_array(abs(z_fixed_array) < 1e-6) = 0;
  R_seed = false([numel(prog.regions), prog.N, numel(prog.feet)]);
  RR_seed = false([numel(prog.regions), prog.N-1, numel(prog.feet)]);
  for j = 1:numel(prog.feet)
    delta_norm = delta_norm + sum(sum((F_fixed_array(:, :, j) - prog.vars.(sprintf('F%d',j)).value).^2));
    delta_norm = delta_norm + sum(sum((M_fixed_array(:, :, j) - prog.vars.(sprintf('M%d',j)).value).^2));
    delta_norm = delta_norm + sum(sum((r_foot_fixed_array(:, :, j) - prog.vars.(sprintf('r_foot%d',j)).value).^2));
    F_fixed_array(:, :, j) = (1-lam)*F_fixed_array(:, :, j) + lam*prog.vars.(sprintf('F%d',j)).value;
    M_fixed_array(:, :, j) = (1-lam)*M_fixed_array(:, :, j) + lam*prog.vars.(sprintf('M%d',j)).value;
    r_foot_fixed_array(:,:,j) = (1-lam)*r_foot_fixed_array(:, :, j) + lam*prog.vars.(sprintf('r_foot%d',j)).value;
    R_seed(:,:,j) = prog.vars.(sprintf('R%d',j)).value;
    RR_seed(:,:,j) = prog.vars.(sprintf('RR%d',j)).value;
  end
  M_fixed_array(abs(M_fixed_array) < 1e-6) = 0;
  F_fixed_array(abs(F_fixed_array) < 1e-6) = 0;
  fprintf('\nObjective Value: %f\tNorm squared of delta: %f\n\n', objval, delta_norm);
  t = cumsum([0, repmat(prog.dt, [1, prog.N-1])]);
  foot_positions = zeros(4*7, prog.N);
  for n = 1:prog.N
    kinsol = rbm.doKinematics([prog.vars.r.value(:,n); prog.vars.z.value(:,n)]);
    for j = 1:numel(prog.feet)
      rname = sprintf('r_foot%d',j);
      foot_positions((j-1)*7 + (1:3), n) = rbm.forwardKin(kinsol, 2, prog.vars.(rname).value(:,n));
      foot_positions((j-1)*7 + (4:7), n) = [1; 0; 0; 0];
    end
  end
  position_traj = PPTrajectory(foh(t, [prog.vars.r.value; prog.vars.z.value;
  foot_positions]));
  position_traj = position_traj.setOutputFrame(rbm_vis.getPositionFrame());
  v.playback(position_traj)
  %keyboard
  %if objval < tol && delta_norm < tol, break; end
  if slack_max < tol && delta_norm < tol, break; end
  fix_forces = ~fix_forces;
  slack_max = slack_max*1e-1;
end
toc(total_time);
%%
v.draw(0, position_traj.eval(t(end)))
%%
% options.floating = true;
% rbm = RigidBodyManipulator(urdf, options);
% v = rbm.constructVisualizer();
% t = cumsum(repmat(prog.dt, [1, prog.N]));
% rpy_value = zeros(3, size(prog.vars.z.value, 2));
% for i = 1:size(rpy_value, 2)
%   rpy_value(:, i) = quat2rpy(prog.vars.z.value(:, i));
% end
% position_traj = PPTrajectory(foh(t, [prog.vars.r.value; rpy_value]));
% % position_traj = PPTrajectory(foh(t, [prog.vars.r.value; prog.vars.z.value]));
% position_traj = position_traj.setOutputFrame(rbm.getPositionFrame());
