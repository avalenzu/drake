N = 20;
tf = 3;
dt = tf/N;
r0 = [0; 1];
th0 = 0;
rf = [1; 1];
v0 = [0; 0];
w0 = 0;

options = struct();
options.floating = true;
options.use_new_kinsol = true;
urdf = fullfile(getDrakePath(), 'solvers', 'test', 'littleBrick.urdf');
particle_urdf = fullfile(getDrakePath(), 'systems', 'plants', 'test', 'PointMass.urdf');
rbm = RigidBodyManipulator(urdf, options);
rbm_vis = rbm;
colormap('lines')
colors = colormap';
colormap('default')
options.collision = false;
for j = 1:1
  rbm_vis = rbm_vis.addRobotFromURDF(particle_urdf, [], [], options);
  body = rbm_vis.body(end);
  body.visual_geometry{1} = body.visual_geometry{1}.setColor(colors(:,j));
  body.visual_geometry{1}.radius = 0.02;
  rbm_vis = rbm_vis.setBody(rbm_vis.getNumBodies(), body);
end
rbm_vis = rbm_vis.compile();
v = rbm_vis.constructVisualizer();
m = rbm.getMass();
leg_length = 0.3;
I = rbm.body(2).inertia(2,2);
Istar = I/(m*leg_length^2);
  
prog = MixedIntegerHopperPlanner(Istar, N, dt);
prog.rotation_max = pi/6;
prog = prog.addRegion([0, -1], -0.05, [], [], [], []);
prog = prog.addRegion([], [], [0, 1], 0, [0; 1], 1);
prog = prog.setupProblem();
prog = prog.addPositionConstraint(1, r0, r0);
prog = prog.addPositionConstraint(N, rf, rf);
prog = prog.addOrientationConstraint(1, th0, th0);
prog = prog.addOrientationConstraint(N, th0, th0);
prog = prog.addVelocityConstraint(1, v0, v0);
prog = prog.addVelocityConstraint(N, v0, v0);
prog.vars.p.lb(:,1) = [0; -1];
prog.vars.p.ub(:,1) = [0; -1];
% prog = prog.addAngularVelocityConstraint(1, w0, w0);
params.outputflag = 1;
[prog, solvertime, objval] = solveGurobi(prog, params);
%%
r_data = leg_length*prog.vars.r.value;
p_data = leg_length*prog.vars.p.value;
th_data = prog.vars.th.value;

q_data = zeros(rbm_vis.getNumPositions(), prog.N);
q_data([1,3], :) = r_data;
q_data(5, :) = th_data;
q_data([7,9], :) = r_data + p_data;

t = sqrt(leg_length/9.81)*0:dt:(N-1)*dt;

qtraj = PPTrajectory(foh(t, q_data));
qtraj = qtraj.setOutputFrame(rbm_vis.getPositionFrame());