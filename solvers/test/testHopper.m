N = 20;
tf = 2*6;
dt = tf/N;
leg_length = 0.3;
step_height = 0.20;
r0 = [0; leg_length];
th0 = 0;
rf = [2; leg_length];
v0 = [0; 0];
w0 = 0;

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
platform2_start = 0.5;
platform2_end = 1.5;
platform3_start = 1.7;
platform3_end = 2.5;
platform1 = RigidBodyBox([platform1_end-platform1_start; 1; 0.1], [(platform1_end+platform1_start)/2; 0; -0.05], [0; 0; 0]);
platform1.c = [0.2; 0.2; 0.2];
platform2 = RigidBodyBox([platform2_end-platform2_start; 1; 0.1], [(platform2_end+platform2_start)/2; 0; step_height-0.05], [0; 0; 0]);
platform2.c = [0.2; 0.2; 0.2];
platform3 = RigidBodyBox([platform3_end-platform3_start; 1; 0.1], [(platform3_end+platform3_start)/2; 0; -0.05], [0; 0; 0]);
platform3.c = [0.2; 0.2; 0.2];
rbm_vis = rbm_vis.addVisualGeometryToBody(1, platform1);
rbm_vis = rbm_vis.addVisualGeometryToBody(1, platform2);
rbm_vis = rbm_vis.addVisualGeometryToBody(1, platform3);
colormap('lines')
colors = colormap';
colormap('default')
options.collision = false;
leg = RigidBodyCapsule(0.01, leg_length, [0; 0; leg_length/2], [0; 0; 0]);
for j = 1:1
  rbm_vis = rbm_vis.addRobotFromURDF(particle_urdf, [], [], options);
  body = rbm_vis.body(end);
  body.visual_geometry{1} = body.visual_geometry{1}.setColor(colors(:,j));
  body.visual_geometry{1}.radius = 0.02;
  rbm_vis = rbm_vis.setBody(rbm_vis.getNumBodies(), body);
  rbm_vis = rbm_vis.addVisualGeometryToBody(rbm_vis.getNumBodies(), leg);
end
rbm_vis = rbm_vis.compile();
v = rbm_vis.constructVisualizer();
m = rbm.getMass();
I = rbm.body(2).inertia(2,2);
Istar = I/(m*leg_length^2);
mipgap = linspace(1-1e-4, 1e-4, 5);
  
for M = 1%1:5
  prog = MixedIntegerHopperPlanner(Istar, N, dt);
  prog.M = M;
  prog.rotation_max = pi/6;
  %prog.position_max = 10;
  prog.velocity_max = 5;
  prog.force_max = 1.5;
  prog.moment_max = prog.force_max;
  %prog = prog.addRegion([0, -1], 0.0, [], [], [], []);
  %prog = prog.addRegion([], [], [0, 1], 0, [0; 1], 1);
  prog = prog.addRegion([0, -1; 1, 0], 1/leg_length*[0.0; platform2_start - 0.2], [], [], [], []);
  prog = prog.addRegion([0, -1], -1/leg_length*(step_height), [], [], [], []);
  prog = prog.addRegion([0, -1; -1, 0], 1/leg_length*[0.0; -(platform2_end- 0.2)], [], [], [], []);

  prog = prog.addRegion([-1, 0;  1, 0], 1/leg_length*[-platform1_start; platform1_end], [0, 1], 0, [0; 1], 1);
  prog = prog.addRegion([-1, 0;  1, 0], 1/leg_length*[-platform2_start; platform2_end], [0, 1], 1/leg_length*step_height, [0; 1], 1);
  prog = prog.addRegion([-1, 0;  1, 0], 1/leg_length*[-platform3_start; platform3_end], [0, 1], 0, [0; 1], 1);
  prog = prog.setupProblem();
  prog = prog.addPositionConstraint(1, 1/leg_length*r0, 1/leg_length*r0);
  prog = prog.addPositionConstraint(N, 1/leg_length*rf, 1/leg_length*rf);
  prog = prog.addOrientationConstraint(1, th0, th0);
  prog = prog.addOrientationConstraint(N, th0, th0);
  prog = prog.addVelocityConstraint(1, v0, v0);
  prog = prog.addVelocityConstraint(N, v0, v0);
  prog.vars.r.lb(2,2:end-1) = leg_length/2;
  prog.vars.w.lb(:,1) = 0;
  prog.vars.w.ub(:,1) = 0;
  prog.vars.w.lb(:,N) = 0;
  prog.vars.w.ub(:,N) = 0;
  prog.vars.p.lb(:,1) = [0; -0.5];
  prog.vars.p.ub(:,1) = [0; -0.5];
  prog.vars.p.lb(:,N) = [0; -0.5];
  prog.vars.p.ub(:,N) = [0; -0.5];
  % prog = prog.addAngularVelocityConstraint(1, w0, w0);
  if 0%M > 1
    for field = fieldnames(prog_prev.vars)
      name = field{1};
      if ~(strcmp(name, 'B') || strcmp(name, 'Bz'))
        prog.vars.(name).start = prog_prev.vars.(name).value;
      end
    end
  end
  params.outputflag = 1;
  %params.mipgap = mipgap(M);
  [prog, solvertime, objval] = solveGurobi(prog, params);
  prog_prev = prog;
end
%%
r_data = leg_length*prog.vars.r.value;
p_data = leg_length*prog.vars.p.value;
th_data = prog.vars.th.value;

leg_pitch_data = -atan2(p_data(1,:), -p_data(2,:));

q_data = zeros(rbm_vis.getNumPositions(), prog.N);
q_data([1,3], :) = r_data;
q_data(5, :) = th_data;
q_data([7,9], :) = r_data + p_data;
q_data(11, :) = leg_pitch_data;

t = sqrt(leg_length/9.81)*(0:dt:(N-1)*dt);

qtraj = PPTrajectory(foh(t, q_data));
qtraj = qtraj.setOutputFrame(rbm_vis.getPositionFrame());
