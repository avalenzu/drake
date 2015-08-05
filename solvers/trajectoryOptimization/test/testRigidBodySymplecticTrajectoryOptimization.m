function testRigidBodySymplecticTrajectoryOptimization(visualize, random_seed)
import drakeFunction.geometry.*
import drakeFunction.*

if nargin < 1, visualize = false; end
if nargin < 2, random_seed = false; end
t0 = 0;
tf = 1.5;
N = 28;
r0 = [0; 0; 0.15];
z0 = rpy2quat([0; 0; 0]);
v0 = [0; 0; 0];
w0 = 0*[1e-1; 2*pi; 0];
x0 = [r0; z0; w0; v0];

urdf = fullfile(getDrakePath(), 'solvers', 'test', 'littleBrick.urdf');
particle_urdf = fullfile(getDrakePath(), 'systems', 'plants', 'test', 'PointMass.urdf');
%urdf = fullfile(getDrakePath(), 'systems', 'plants', 'test', 'FallingBrick.urdf');
options.floating = 'quat';
options.use_new_kinsol = true;
rbm = RigidBodyManipulator(urdf, options);

rbm_vis = rbm;
options.collision = false;
colors = [0, 0, 1; 0, 1, 0; 1, 0, 0; 0.5, 0.5, 0.5]';
for j = 1:4
  rbm_vis = rbm_vis.addRobotFromURDF(particle_urdf, [], [], options);
  body = rbm_vis.body(end);
  body.visual_geometry{1} = body.visual_geometry{1}.setColor(colors(:,j));
  body.visual_geometry{1}.radius = 0.02;
  rbm_vis = rbm_vis.setBody(rbm_vis.getNumBodies(), body);
end
rbm_vis = rbm_vis.compile();

m = rbm.getMass();
I = inertiaCalculator('box', m, rbm.body(2).visual_geometry{1}.size);
% I = diag([1; 4; 8]);
%
T = Identity(3);
w = Identity(3);
F_minus_mg = Affine(eye(3), -m*[0; 0; -9.81]);
v = Identity(3);
h = Identity(1);
power = h.*(F_minus_mg.dot(v) + T.dot(w));

dim_x = rbm.body(2).visual_geometry{1}.size(1);
dim_y = rbm.body(2).visual_geometry{1}.size(2);
dim_z = rbm.body(2).visual_geometry{1}.size(3);
feet.LH.pt_in_body = [-dim_x/2;  dim_y/2;  -dim_z/2];
feet.RH.pt_in_body = [-dim_x/2; -dim_y/2;  -dim_z/2];
feet.LF.pt_in_body = [ dim_x/2;  dim_y/2;  -dim_z/2];
feet.RF.pt_in_body = [ dim_x/2; -dim_y/2;  -dim_z/2];
stride_length = dim_x;
xf = x0; xf(1) = xf(1) + 2*stride_length;
leg_length = dim_x/2;
for field = fields(feet)'
  foot = field{1};
  feet.(foot).relative_position_fcn{1} = Identity(3) - Identity(3) - compose(RotateVectorByQuaternion(), [Identity(4); Affine(zeros(3,0), feet.(foot).pt_in_body)]);
  feet.(foot).relative_position_fcn{2} = Identity(3) - Identity(3) - compose(RotateVectorByQuaternion(), [Identity(4); Affine(zeros(3,0), feet.(foot).pt_in_body+[0; 0; -leg_length])]);
  feet.(foot).relative_velocity_fcn = Identity(3) - compose(RotateVectorByQuaternion(), ...
                                         [Identity(4); ...
                                          Linear(vectorToSkewSymmetric(feet.(foot).pt_in_body))]);
end

M = floor(N/7);

contact_wrench_struct(1).foot = 'LH';
contact_wrench_struct(1).knots = 1:2*M;
contact_wrench_struct(1).num_forces = 1;
contact_wrench_struct(1).pt_in_world = [-dim_x/2; dim_y/2; 0];
contact_wrench_struct(1).normal = [0; 0; 1];
contact_wrench_struct(1).mu = 1;

contact_wrench_struct(2).foot = 'RH';
contact_wrench_struct(2).knots = 1:2*M;
contact_wrench_struct(2).num_forces = 1;
contact_wrench_struct(2).pt_in_world = [-dim_x/2; -dim_y/2; 0];
contact_wrench_struct(2).normal = [0; 0; 1];
contact_wrench_struct(2).mu = 1;

contact_wrench_struct(3).foot = 'LF';
contact_wrench_struct(3).knots = 1:M;
contact_wrench_struct(3).num_forces = 1;
contact_wrench_struct(3).pt_in_world = [dim_x/2; dim_y/2; 0];
contact_wrench_struct(3).normal = [0; 0; 1];
contact_wrench_struct(3).mu = 1;

contact_wrench_struct(4).foot = 'RF';
contact_wrench_struct(4).knots = 1:M;
contact_wrench_struct(4).num_forces = 1;
contact_wrench_struct(4).pt_in_world = [dim_x/2; -dim_y/2; 0];
contact_wrench_struct(4).normal = [0; 0; 1];
contact_wrench_struct(4).mu = 1;

contact_wrench_struct(5).foot = 'LF';
contact_wrench_struct(5).knots = 2*M:4*M;
contact_wrench_struct(5).num_forces = 1;
contact_wrench_struct(5).pt_in_world = [dim_x/2; dim_y/2; 0] + [stride_length; 0; 0];
contact_wrench_struct(5).normal = [0; 0; 1];
contact_wrench_struct(5).mu = 1;

contact_wrench_struct(6).foot = 'RF';
contact_wrench_struct(6).knots = 2*M:4*M;
contact_wrench_struct(6).num_forces = 1;
contact_wrench_struct(6).pt_in_world = [dim_x/2; -dim_y/2; 0] + [stride_length; 0; 0];
contact_wrench_struct(6).normal = [0; 0; 1];
contact_wrench_struct(6).mu = 1;

contact_wrench_struct(7).foot = 'LH';
contact_wrench_struct(7).knots = 3*M:5*M;
contact_wrench_struct(7).num_forces = 1;
contact_wrench_struct(7).pt_in_world = [-dim_x/2; dim_y/2; 0] + [1.5*stride_length; 0; 0];
contact_wrench_struct(7).normal = [0; 0; 1];
contact_wrench_struct(7).mu = 1;

contact_wrench_struct(8).foot = 'RH';
contact_wrench_struct(8).knots = 3*M:5*M;
contact_wrench_struct(8).num_forces = 1;
contact_wrench_struct(8).pt_in_world = [-dim_x/2; -dim_y/2; 0] + [1.5*stride_length; 0; 0];
contact_wrench_struct(8).normal = [0; 0; 1];
contact_wrench_struct(8).mu = 1;

contact_wrench_struct(9).foot = 'LF';
contact_wrench_struct(9).knots = 5*M:7*M;
contact_wrench_struct(9).num_forces = 1;
contact_wrench_struct(9).pt_in_world = [dim_x/2; dim_y/2; 0] + [2*stride_length; 0; 0];
contact_wrench_struct(9).normal = [0; 0; 1];
contact_wrench_struct(9).mu = 1;

contact_wrench_struct(10).foot = 'RF';
contact_wrench_struct(10).knots = 5*M:7*M;
contact_wrench_struct(10).num_forces = 1;
contact_wrench_struct(10).pt_in_world = [dim_x/2; -dim_y/2; 0] + [2*stride_length; 0; 0];
contact_wrench_struct(10).normal = [0; 0; 1];
contact_wrench_struct(10).mu = 1;

contact_wrench_struct(11).foot = 'LH';
contact_wrench_struct(11).knots = 6*M:7*M;
contact_wrench_struct(11).num_forces = 1;
contact_wrench_struct(11).pt_in_world = [-dim_x/2; dim_y/2; 0] + [2*stride_length; 0; 0];
contact_wrench_struct(11).normal = [0; 0; 1];
contact_wrench_struct(11).mu = 1;

contact_wrench_struct(12).foot = 'RH';
contact_wrench_struct(12).knots = 6*M:7*M;
contact_wrench_struct(12).num_forces = 1;
contact_wrench_struct(12).pt_in_world = [-dim_x/2; -dim_y/2; 0] + [2*stride_length; 0; 0];
contact_wrench_struct(12).normal = [0; 0; 1];
contact_wrench_struct(12).mu = 1;

for i = 1:numel(contact_wrench_struct)
  lb = contact_wrench_struct(i).pt_in_world;
  ub = lb;
  contact_wrench_struct(i).constraint{1} = BoundingBoxConstraint(lb, ub);
  contact_wrench_struct(i).vars{1} = {'p'};

  lb = 0;%(leg_length/2)^2;
  ub = leg_length^2;
  contact_wrench_struct(i).constraint{2} = DrakeFunctionConstraint(lb, ub, ...
    compose(euclidean.NormSquared(3),feet.(contact_wrench_struct(i).foot).relative_position_fcn{1}));
  contact_wrench_struct(i).vars{2} = {'p','r','z'};
  lb = 0;%(leg_length/2)^2;
  ub = leg_length^2;
  contact_wrench_struct(i).constraint{3} = DrakeFunctionConstraint(lb, ub, ...
    compose(euclidean.NormSquared(3),feet.(contact_wrench_struct(i).foot).relative_position_fcn{2}));
  contact_wrench_struct(i).vars{3} = {'p','r','z'};

  % dot(f, normal) > 0
  lb = 0;
  ub = Inf;
  normal = contact_wrench_struct(i).normal;
  mu = contact_wrench_struct(i).mu;
  constraint = LinearConstraint(lb, ub, normal');
  contact_wrench_struct(i).constraint{4} = constraint;
  contact_wrench_struct(i).vars{4} = {'f'};

  % dot(f, normal)^2 - mu^2 ||f||^2 > 0
  % f'*normal*normal'*f - f'*mu^2*eye(3)*f > 0
  % f'*(normal*normal' - mu^2*eye(3))*f > 0
  Q = normal*normal' - 1/(mu^2+1)*eye(3);
  constraint = QuadraticConstraint(lb, ub, Q, zeros(3,1));
  contact_wrench_struct(i).constraint{5} = constraint;
  contact_wrench_struct(i).vars{5} = {'f'};
end

options = struct();
options.time_option = 1;
prog = RigidBodySymplecticTrajectoryOptimization(m, I, contact_wrench_struct, ...
                                                 N, [tf/4; 4*tf], options);
% prog = prog.setCheckGrad(true);
%prog = prog.addConstraint(ConstantConstraint(x0), prog.x_inds(:, 1));
tol = eps;
prog = prog.addConstraint(BoundingBoxConstraint(x0-tol, x0+tol), prog.x_inds(:, 1));
prog = prog.addConstraint(BoundingBoxConstraint(xf-tol, xf+tol), prog.x_inds(:, N));
%prog = prog.addConstraint(BoundingBoxConstraint(tf/(2*N)*ones(size(prog.h_inds)), 2*tf/N*ones(size(prog.h_inds))), prog.h_inds);
f_inds = [];
for n = 1:prog.N
  f_inds = [f_inds; prog.contact_inds(n).forces(:)]; %#ok
end
xinds = [];
for n = 1:prog.N-1
  xinds = [xinds; prog.h_inds(n); prog.F_inds(:,n); prog.v_inds(:,n); prog.T_inds(:,n); prog.w_inds(:,n)];
end
power_cost = compose(Linear(ones(1,prog.N-1)), duplicate(compose(ConstantPower(power.dim_output, 2), power), prog.N-1));
prog = prog.addCost(DrakeFunctionConstraint(-Inf, Inf, power_cost), xinds);
%prog = prog.addCost(QuadraticConstraint(-Inf, Inf, eye(numel(f_inds)), zeros(numel(f_inds),1)), f_inds(:));
%prog = prog.addCost(QuadraticConstraint(-Inf, Inf, [zeros(3*prog.N), eye(3*prog.N); eye(3*prog.N), zeros(3*prog.N)], zeros(6*prog.N,1)), [prog.F_inds(:); prog.v_inds(:)]);
%prog = prog.addCost(QuadraticConstraint(-Inf, Inf, eye(numel(prog.h_inds)), zeros(numel(prog.h_inds),1)), prog.h_inds(:));
% prog = prog.addConstraint(ConstantConstraint(zeros(size(prog.r_inds(2,:)))), prog.r_inds(2,:));
 prog = prog.addConstraint(BoundingBoxConstraint(leg_length/2*ones(prog.N,1), Inf(prog.N,1)), prog.r_inds(3, :));
 %prog = prog.addConstraint(ConstantConstraint(x0), prog.x_inds(:, prog.N));
 prog = prog.addConstraint(ConstantConstraint(0), prog.v_inds(3, prog.N));
% prog = prog.addConstraint(ConstantConstraint(0), prog.F_inds(3, 1));
% prog = prog.addConstraint(ConstantConstraint(0), prog.F_inds(3, prog.N));
%for n = contact_wrench_struct(1).knots
  %ind = n;
  %prog = prog.addConstraint(fixed_point_constraint_A, [prog.z_inds(:,n); prog.r_inds(:,n)]);
  %prog = prog.addConstraint(fixed_point_vel_constraint_A, [prog.v_inds(:,ind); prog.z_inds(:,ind); prog.w_inds(:,ind)]);
%end
%for n = contact_wrench_struct(2).knots
  %ind = n;
  %prog = prog.addConstraint(fixed_point_constraint_B, [prog.z_inds(:,n); prog.r_inds(:,n)]);
  %prog = prog.addConstraint(fixed_point_vel_constraint_B, [prog.v_inds(:,ind); prog.z_inds(:,ind); prog.w_inds(:,ind)]);
%end
prog = prog.setSolverOptions('snopt', 'SuperbasicsLimit', 2000);
%prog = prog.setSolverOptions('snopt', 'LinesearchTolerance', 0.99);
prog = prog.setSolverOptions('snopt', 'IterationsLimit', 1e5);
prog = prog.setSolverOptions('snopt', 'MajorIterationsLimit', 5e2);
prog = prog.setSolverOptions('snopt', 'MajorOptimalityTolerance', 1e-3);
prog = prog.setSolverOptions('snopt', 'print', 'snopt.out');

if visualize
  vis = rbm.constructVisualizer();
  prog = prog.addDisplayFunction(@(z) displayCallback(z, prog, vis));
end

t_init = linspace(t0, tf, N);
if random_seed
  traj_init.x = DTTrajectory(t_init, randn(numel(x0), N));
else
  traj_init.x = PPTrajectory(foh(t_init, linspacevec(x0, xf, N)));
  traj_init.u = ConstantTrajectory(zeros(6,1));
end
 %profile off; profile on
 solver_time = tic;
[xtraj,utraj,z,F,info,infeasible_constraint_name] = solveTraj(prog,t_init,traj_init);
toc(solver_time);
 %profile off; profile viewer
  xtraj = xtraj.setOutputFrame(rbm.getStateFrame());

  x_data = xtraj.eval(xtraj.getBreaks());
  r_data = x_data(1:3, :);
  z_data = x_data(4:7, :);
  w_data = x_data(8:10, :);
  v_data = x_data(11:13, :);
  t = cumsum([0; z(prog.h_inds(:))]);
  
  k_data = zeros(size(w_data));
  KE_data = zeros(1, N);
  for n = 1:N
    k_data(:, n) = quatRotateVec(z_data(:, n), I*w_data(:, n));
    KE_data(n) = 0.5*w_data(:, n)'*prog.I*w_data(:, n) + 0.5*prog.m*v_data(:, n)'*v_data(:, n);
  end
keyboard
end

function displayCallback(z, prog, vis)
  persistent h
  t = cumsum([0; z(prog.h_inds(:))]);
  x = z(prog.x_inds);
%   subplot(2, 1, 1);
  try
    set(h(1), 'XData', t, 'YData', x(1,:)');
    set(h(2), 'XData', t, 'YData', x(2,:)');
    set(h(3), 'XData', t, 'YData', x(3,:)');
  catch ex
    h = plot(t, x(1:3,:)', '.-');
  end
  drawnow;
  %vis.playback_speed = 2;
   %xtraj = PPTrajectory(zoh(t, x(1:7, :)));
   %xtraj = xtraj.setOutputFrame(vis.getInputFrame());
   %vis.playback(xtraj);
end
