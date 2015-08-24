if ~exist('vis_only', 'var'), vis_only = false; end
if ~vis_only
clear prog_prev
N = 20;
tf = 1.6;
leg_length = 0.3;
dt = tf/N/sqrt(leg_length/9.81);
%dt = 0.08/sqrt(leg_length/9.81);
step_height = 0.2;
r0 = [0; leg_length/2];
th0 = 0.0;
rf = [2; leg_length];
v0 =  [0; 0];
w0 = 0;
hip_offset_x = 0.5;
hip_in_body = [[-hip_offset_x; -0.25], [hip_offset_x; -0.25]];
%hip_in_body = [-hip_offset_x ; -0.25];

options = struct();
options.floating = true;
options.use_new_kinsol = true;
urdf = fullfile(getDrakePath(), 'solvers', 'test', 'littleBrick.urdf');
particle_urdf = fullfile(getDrakePath(), 'systems', 'plants', 'test', 'PointMass.urdf');
rbm = RigidBodyManipulator(urdf, options);
rbm_vis = rbm;
%step_height = 0.0;
platform1_start = -0.5;
platform1_end = 0.3;
platform1_height = 0*step_height;
platform2_start = 0.7;
platform2_end = 1.3;
platform2_pitch = 0*pi/180;
platform2_height = step_height;
platform3_start = 1.7;
platform3_end = 9.5;
platform3_height = 2*step_height;
platform1 = RigidBodyBox([platform1_end-platform1_start; 1; 0.1], [(platform1_end+platform1_start)/2; 0; platform1_height-0.05], [0; 0; 0]);
platform1.c = [0.2; 0.2; 0.2];
platform2 = RigidBodyBox([platform2_end-platform2_start; 1; 0.1], [(platform2_end+platform2_start)/2; 0; platform2_height-0.05], [0; platform2_pitch; 0]);
platform2.c = [0.2; 0.2; 0.2];
platform3 = RigidBodyBox([platform3_end-platform3_start; 1; 0.1], [(platform3_end+platform3_start)/2; 0; platform3_height-0.05], [0; 0; 0]);
platform3.c = [0.2; 0.2; 0.2];
rbm_vis = rbm_vis.addVisualGeometryToBody(1, platform1);
rbm_vis = rbm_vis.addVisualGeometryToBody(1, platform2);
rbm_vis = rbm_vis.addVisualGeometryToBody(1, platform3);
colormap('lines')
colors = colormap';
colormap('default')
options.collision = false;
leg = RigidBodyCapsule(0.01, leg_length, [0; 0; leg_length/2], [0; 0; 0]);

for j = 1:size(hip_in_body,2)
  % Add feet
  rbm_vis = rbm_vis.addRobotFromURDF(particle_urdf, [], [], options);
  body = rbm_vis.body(end);
  body.visual_geometry{1} = body.visual_geometry{1}.setColor(colors(:,j));
  body.visual_geometry{1}.radius = 0.02;
  rbm_vis = rbm_vis.setBody(rbm_vis.getNumBodies(), body);
  rbm_vis = rbm_vis.addVisualGeometryToBody(rbm_vis.getNumBodies(), leg);

  %Add hips
  rbm_vis = rbm_vis.addRobotFromURDF(particle_urdf, [], [], options);
  body = rbm_vis.body(end);
  body.visual_geometry{1} = body.visual_geometry{1}.setColor(colors(:,j));
  body.visual_geometry{1}.radius = 0.03;
  rbm_vis = rbm_vis.setBody(rbm_vis.getNumBodies(), body);
end
rbm_vis = rbm_vis.compile();
v = HopperVisualizer(rbm_vis.constructVisualizer());
%v = rbm_vis.constructVisualizer();

%%
m = rbm.getMass();
I = rbm.body(2).inertia(2,2);
Istar = I/(m*leg_length^2);
%mipgap = 0.5*[ones(1,4),1e-4];
  
for M = 4%:5 
  prog = MixedIntegerHopperPlanner(Istar, N, dt);
  prog.hip_in_body = hip_in_body;
  prog.num_legs = size(hip_in_body,2);
  prog.M = M;
  prog.n_orientation_sectors = 1;
  prog.rotation_max = pi/8;
  prog.use_foot_acceleration = true;
  %prog.disjunctive_constraint_method = 'convex-hull';
  prog.disjunctive_constraint_method = 'big-M';
  %prog.use_slack = true;
  %prog.minimize_integral_of_squared_power = true;
  %prog.position_max = 10;
  prog.velocity_max = 3;
  prog.force_max = 10;
  prog.moment_max = prog.force_max;
  %prog = prog.addRegion([0, -1], 0.0, [], [], [], []);
  %prog = prog.addRegion([], [], [0, 1], 0, [0; 1], 1);
  %
  prog = prog.addRegion([0, -1; 1, 0], 1/leg_length*[-platform1_height; platform2_start], [], [], [], []);
  prog = prog.addRegion([0, -1; -1, 0; 1, 0], 1/leg_length*[-(platform2_height + 0.0); -platform1_end; max(platform2_end, platform3_start)], [], [], [], []);
  prog = prog.addRegion([0, -1], -1/leg_length*platform3_height, [], [], [], []);

  prog = prog.addRegion([-1, 0;  1, 0], 1/leg_length*[-platform1_start; platform1_end], [0, 1], 1/leg_length*platform1_height, [0; 1], 1);
  prog = prog.addRegion([-1, 0;  1, 0], 1/leg_length*[-platform2_start; platform2_end], [0, 1], 1/leg_length*platform2_height, [0; 1], 1);
  prog = prog.addRegion([-1, 0;  1, 0], 1/leg_length*[-platform3_start; platform3_end], [0, 1], 1/leg_length*platform3_height, [0; 1], 1);

  if exist('prog_prev','var')
    c_approx_splits = prog_prev.c_approx_splits;
    if isempty(c_approx_splits)
      c_approx_splits = cell(prog.dim, prog.N, prog.n_basis_vectors);
    end
    for k = 1:prog.dim
      for n = 1:prog.N
        for i = 1:prog.n_basis_vectors
          c_approx_splits{k,n,i} = [c_approx_splits{k,n,i}, find(prog_prev.vars.B.value(k,n,:))]; 
        end
      end
    end
    prog.c_approx_splits = c_approx_splits;
  else
    for k = 1:prog.dim
      for n = 1:prog.N
        for i = 1:prog.n_basis_vectors
          c_approx_splits{k,n,i} = [1,2,1];
        end
      end
    end
    %prog.c_approx_splits = c_approx_splits;
  end

  prog = prog.setupProblem();
  %prog = prog.addPositionConstraint(1, 1/leg_length*r0, 1/leg_length*r0);
  %prog = prog.addPositionConstraint(N, 1/leg_length*(rf - [0; 10*step_height]), 1/leg_length*(rf + [10; 10*step_height]));
  %prog.vars.r.lb(2,N) = 1/leg_length*(platform1_height);
  prog.vars.r.lb(1,1) = r0(1);
  prog.vars.r.ub(1,1) = r0(1);
  prog.vars.r.lb(1,N) = 1/leg_length*rf(1);
  prog = prog.addOrientationConstraint(1, th0, th0);
  prog = prog.addOrientationConstraint(N, th0, th0);
  prog = prog.addVelocityConstraint(1, v0, v0);
  %prog = prog.addVelocityConstraint(2, v0, v0);
  %prog = prog.addVelocityConstraint(N-1, v0, v0);
  prog = prog.addVelocityConstraint(N, v0, v0);
  prog.vars.r.lb(2,2:end-1) = leg_length/2;
  prog.vars.v.ub(2,:) = min(prog.vars.v.ub(2,:), 1);
  prog.vars.v.ub(1,:) = min(prog.vars.v.ub(1,:), 1);
  prog.vars.w.lb(:,1) = 0;
  prog.vars.w.ub(:,1) = 0;
  prog.vars.w.lb(:,N) = 0;
  prog.vars.w.ub(:,N) = 0;
  prog.vars.p.ub(1,:,1) = 0.5*hip_offset_x;
  prog.vars.p.lb(1,:,2) = -0.5*hip_offset_x;
  for j = 1:prog.num_legs
    prog.vars.p.lb(:,1,j) = [0; -0.6];
    prog.vars.p.ub(:,1,j) = [0; -0.6];
    prog.vars.p.lb(:,N,j) = [0; -0.6];
    prog.vars.p.ub(:,N,j) = [0; -0.6];
    %prog.vars.p.ub(2,1,j) = -0.6;
  end
  %prog.vars.p.lb(:,N) = [0; -0.5];
  %prog.vars.p.ub(:,N) = [0; -0.5];
  % prog = prog.addAngularVelocityConstraint(1, w0, w0);
  if exist('prog_prev','var')
    for field = fieldnames(prog_prev.vars)'
      name = field{1};
      switch name
        case 'slack'
          prog.vars.slack.start = prog.vars.slack.ub;
        case 'c'
          for k = 1:prog.dim
            for n = 1:prog.N
              for i = 1:prog.n_basis_vectors
                for j = 1:prog.num_legs
                  prog.vars.c.start(k,n,i,j) = prog_prev.vars.b.value(i,n,j).*(prog_prev.vars.p.value(k,n,j) + prog_prev.vars.r_hip.value(k,n,j));
                end
              end
            end
          end
        case 'B'
          prog.vars.(name).start(:) = 0;
          for k = 1:prog.dim
            for n = 1:prog.N
              for i = 1:prog.n_basis_vectors
                for j = 1:prog.num_legs
                  for m = 1:prog.M
                    A = prog.c_approx_A{k,n,i,j,m};
                    b = prog.c_approx_b{k,n,i,j,m};
                    c = prog_prev.vars.b.value(i,n,j).*(prog_prev.vars.p.value(k,n,j) + prog_prev.vars.r_hip.value(k,n,j));
                    x = [prog_prev.vars.r_hip.value(k,n,j), ...
                      prog_prev.vars.p.value(k,n,j), ...
                      prog_prev.vars.b.value(i,n,j), ...
                      c]';
                    prog.vars.B.start(k,n,j,m) = all(A*x <= b + sqrt(eps));
                  end
                end
              end
            end
          end
        otherwise
          prog.vars.(name).start = prog_prev.vars.(name).value;
      end
    end
  end
  params = struct();
  params.outputflag = 1;
  params.threads = 12;
  %params.mipgap = mipgap(M);
  %params.mipgap = 0.1;
  %params.timelimit = 30;
  [prog, solvertime, objval] = solveGurobi(prog, params);
  prog_prev = prog;
end
end
%%
r_data = leg_length*prog_prev.vars.r.value;
r_hip_data = reshape(leg_length*prog_prev.vars.r_hip.value, [prog_prev.dim, prog_prev.N, prog_prev.num_legs]);
p_data = reshape(leg_length*prog_prev.vars.p.value, [prog_prev.dim, prog_prev.N, prog_prev.num_legs]);
th_data = prog_prev.vars.th.value;

leg_pitch_data = zeros(1, prog_prev.N, prog_prev.num_legs);
force_angle_data = zeros(1, prog_prev.N, prog_prev.num_legs);
for j = 1:prog_prev.num_legs
  leg_pitch_data(:,:,j) = -atan2(p_data(1,:,j), -p_data(2,:,j));
  force_angle_data(:,:,j) = atan2(prog_prev.vars.F.value(1,:,j), prog_prev.vars.F.value(2,:,j));
  %force_angle_data(sqrt(sum(prog_prev.vars.F.value(:,:,j).^2)) < 1e-6) = 0;
end

q_data = zeros(rbm_vis.getNumPositions(), prog_prev.N);
q_data([1,3], :) = r_data;
q_data(5, :) = th_data;
for j = 1:prog_prev.num_legs
  q_data(6 + 12*(j-1) + [1,3], :) = r_data + r_hip_data(:,:,j) + p_data(:,:,j);
  %q_data([7,9], :) = r_data + p_data;
  q_data(6 + 12*(j-1) + 5, :) = leg_pitch_data(:,:,j);
  q_data(6 + 12*(j-1) + [7,9], :) = r_data + r_hip_data(:,:,j);
end

t = sqrt(leg_length/9.81)*(0:dt:(N-1)*dt);

qtraj = PPTrajectory(foh(t, q_data));
qtraj = qtraj.setOutputFrame(rbm_vis.getPositionFrame());
Ftraj = PPTrajectory(zoh(t, reshape(permute(prog_prev.vars.F.value, [1, 3, 2]),[],prog_prev.N)));
%rHipTraj = PPTrajectory(zoh(t, leg_length*prog_prev.vars.r_hip.value));
