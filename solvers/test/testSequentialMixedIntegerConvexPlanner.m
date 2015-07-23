if ~exist('continuation', 'var') || ~continuation
  clear R_seed
  options = struct();
  options.floating = 'quat';
  options.use_new_kinsol = true;
  options.terrain = RigidBodyFlatTerrain;
  urdf = fullfile(getDrakePath(), 'solvers', 'test', 'littleBrick.urdf');
  particle_urdf = fullfile(getDrakePath(), 'systems', 'plants', 'test', 'PointMass.urdf');
  rbm = RigidBodyManipulator(urdf, options);
  rbm_vis = rbm;
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
  N = 20;
  M = 1000;
  dt = 2/N;
  r0 = [0; 0; 0.5];
  z0 = rpy2quat([0; 0*pi/180; 0]);
  zf = rpy2quat([0; 0*pi/180; 1*pi/180]);
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
end

total_solvertime = 0;
total_time = tic;
leg_length = 0.1;
for i = 1:M
  prog = SequentialMixedIntegerConvexPlanner(I, m, N, dt, z_fixed_array, w_fixed_array, F_fixed_array, M_fixed_array, false);
  prog.force_max = 1e2;
  prog.velocity_max = 2;
  prog.position_max = 1e1;
  prog.stance_velocity_max = 1;
  prog.swing_velocity_max = 1;
  prog.fix_forces = fix_forces;
  for j = 1:4 
    pt = rbm.getTerrainContactPoints(2).pts(:,2+j);
    centers = [pt, pt + [0; 1.5*sign(pt(2))*leg_length*sin(0*pi/180); -1.5*leg_length*cos(0*pi/180)]];
    %centers = [pt+[0; -0.5*leg_length + sign(pt(2))*0.1*leg_length; -leg_length], pt+[0; 0.5*leg_length + sign(pt(2))*0.1*leg_length; -leg_length]];
    prog = prog.addFoot(centers, [leg_length, leg_length], r_foot_fixed_array(:,:,j));
  end
  prog = prog.addRegion([0, 0, -1], -0.05, [], [], [], []);
  %prog = prog.addRegion([0, 0, -1], 0, [], [], [], []);
  %prog = prog.addRegion([0, 0, 1], 0, [], [], [], []);
  %prog = prog.addRegion([0, 0, 1; 0, 0, -1], [0; 0.06], [], [], [0; 0; 1], 1);
  %prog = prog.addRegion([0, 0, 1; 0, 0, -1], [0; 0.05], [], [], [0; 0; 1], 1);
  prog = prog.addRegion([], [], [0, 0, 1], 0, [0; 0; 1], 1);
  prog = prog.addDefaultConstraintsAndCosts();
  if exist('R_seed','var')
    for j = 1:numel(prog.feet)
      prog.vars.(sprintf('R%d',j)).start = R_seed(:,:,j);
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
  prog = prog.addPositionConstraint(N, r0 + [1; 0; -0.8], r0 + [1; 0; 0.8]);
  %prog = prog.addSymbolicConstraints(0.5 <= prog.vars.r.symb(3,1) <= 1.1);
  %prog = prog.addSymbolicConstraints(prog.vars.r.symb(1:2,1) == 0);
  %prog = prog.addAngularVelocityConstraint([1,N], 0, 0);
  prog = prog.addVelocityConstraint([1,N], 0, 0);
  Aeq = zeros(1, prog.nv);
  beq = pi/2*prog.N;
  Aeq(1, prog.vars.w.i(3,:)) = 1;
  %prog = prog.addLinearConstraints([], [], Aeq, beq);
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
  params.mipgap = 0.9;
  params.outputflag = 1;
  %params.solver = 'gurobi';
  %params.threads = 12;
 [prog, solvertime, objval] = prog.solveGurobi(params);
  %[prog, solvertime, objval] = prog.solve();
  delta_norm = sum((w_fixed_array(:) - prog.vars.w.value(:)).^2) ...
                + sum((z_fixed_array(:) - prog.vars.z.value(:)).^2);
  total_solvertime = total_solvertime + solvertime;
  w_fixed_array = (1-lam)*w_fixed_array + lam*prog.vars.w.value;
  w_fixed_array(abs(w_fixed_array) < 1e-6) = 0;
  z_fixed_array = (1-lam)*z_fixed_array + lam*prog.vars.z.value;
  %z_fixed_array = bsxfun(@rdivide, z_fixed_array, sqrt(sum(z_fixed_array.^2, 1)));
  z_fixed_array(abs(z_fixed_array) < 1e-6) = 0;
  R_seed = false([numel(prog.regions), prog.N, numel(prog.feet)]);
  for j = 1:numel(prog.feet)
    delta_norm = delta_norm + sum(sum((F_fixed_array(:, :, j) - prog.vars.(sprintf('F%d',j)).value).^2));
    delta_norm = delta_norm + sum(sum((M_fixed_array(:, :, j) - prog.vars.(sprintf('M%d',j)).value).^2));
    delta_norm = delta_norm + sum(sum((r_foot_fixed_array(:, :, j) - prog.vars.(sprintf('r_foot%d',j)).value).^2));
    F_fixed_array(:, :, j) = (1-lam)*F_fixed_array(:, :, j) + lam*prog.vars.(sprintf('F%d',j)).value;
    M_fixed_array(:, :, j) = (1-lam)*M_fixed_array(:, :, j) + lam*prog.vars.(sprintf('M%d',j)).value;
    r_foot_fixed_array(:,:,j) = (1-lam)*r_foot_fixed_array(:, :, j) + lam*prog.vars.(sprintf('r_foot%d',j)).value;
    R_seed(:,:,j) = prog.vars.(sprintf('R%d',j)).value;
  end
  M_fixed_array(abs(M_fixed_array) < 1e-6) = 0;
  F_fixed_array(abs(F_fixed_array) < 1e-6) = 0;
  fprintf('Objective Value: %f\tNorm squared of delta: %f\n', objval, delta_norm);
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
  if delta_norm < tol, break; end
  fix_forces = ~fix_forces;
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
