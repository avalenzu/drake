function testRigidBodySymplecticTrajectoryOptimization(visualize, random_seed)
import drakeFunction.geometry.*
import drakeFunction.*

if nargin < 1, visualize = false; end
if nargin < 2, random_seed = false; end
t0 = 0;
tf = 3;
N = 20;
r0 = [0.5; 0; 1];
z0 = rpy2quat([pi/2; 0; 0]);
v0 = [0; 0; 0];
w0 = 0*[1e-1; 2*pi; 0];
x0 = [r0; z0; w0; v0];

urdf = fullfile(getDrakePath(), 'solvers', 'test', 'littleBrick.urdf');
urdf = fullfile(getDrakePath(), 'systems', 'plants', 'test', 'FallingBrick.urdf');
options.floating = 'quat';
options.use_new_kinsol = true;
rbm = RigidBodyManipulator(urdf, options);

m = rbm.getMass();
I = inertiaCalculator('box', m, rbm.body(2).visual_geometry{1}.size);
% I = diag([1; 4; 8]);

pt_in_body = [-1; 0; 0];
relative_pos_fcn = compose(RotateVectorByQuaternion(), [Identity(4); Affine(zeros(3,0), [-1; 0; 0])]) + Identity(3);
point0 = quatRotateVec(z0, pt_in_body) + [1; 0; 0];
fixed_point_constraint = DrakeFunctionConstraint(point0, point0, ...
                                                 relative_pos_fcn);
relative_vel_fcn = Identity(3) - compose(RotateVectorByQuaternion(), ...
                                         [Identity(4); ...
                                          Linear(vectorToSkewSymmetric(pt_in_body))]);
fixed_point_vel_constraint = DrakeFunctionConstraint(zeros(3,1), ...
                                                     zeros(3,1), ...
                                                     relative_vel_fcn);
options = struct();
options.time_option = 2;
wrench.knots = ceil(N/4):N;
wrench.num_forces = 1;
lb = zeros(3,1);
ub = zeros(3,1);
wrench.constraint{1} = BoundingBoxConstraint(lb, ub);
wrench.vars{1} = {'p'};
contact_wrench_struct(1) = wrench;
prog = RigidBodySymplecticTrajectoryOptimization(m, I, contact_wrench_struct, ...
                                                 N, [tf; tf], options);
% prog = prog.setCheckGrad(true);
prog = prog.addConstraint(ConstantConstraint(x0), prog.x_inds(:, 1));
prog = prog.addConstraint(BoundingBoxConstraint(tf/(2*N)*ones(size(prog.h_inds)), 2*tf/N*ones(size(prog.h_inds))), prog.h_inds);
prog = prog.addCost(QuadraticConstraint(-Inf, Inf, eye(numel(prog.F_inds)), repmat(-[0; 0; -9.81], prog.N, 1)), prog.F_inds(:));
prog = prog.addCost(QuadraticConstraint(-Inf, Inf, eye(numel(prog.h_inds)), zeros(numel(prog.h_inds),1)), prog.h_inds(:));
prog = prog.addConstraint(ConstantConstraint(zeros(size(prog.r_inds(2,:)))), prog.r_inds(2,:));
% prog = prog.addConstraint(ConstantConstraint(r0), prog.r_inds(:, prog.N));
% prog = prog.addConstraint(ConstantConstraint(0), prog.x_inds(13, prog.N));
% prog = prog.addConstraint(ConstantConstraint(0), prog.F_inds(3, 1));
% prog = prog.addConstraint(ConstantConstraint(0), prog.F_inds(3, prog.N));
for n = wrench.knots
  prog = prog.addConstraint(fixed_point_constraint, [prog.z_inds(:,n); prog.r_inds(:,n)]);
  if n < prog.N
    prog = prog.addConstraint(fixed_point_vel_constraint, [prog.v_inds(:,n+1); prog.z_inds(:,n+1); prog.w_inds(:,n+1)]);
  end
end
prog = prog.setSolverOptions('snopt', 'SuperbasicsLimit', 2000);
prog = prog.setSolverOptions('snopt', 'LinesearchTolerance', 0.99);
prog = prog.setSolverOptions('snopt', 'IterationsLimit', 1e5);

if visualize
  v = rbm.constructVisualizer();
  prog = prog.addDisplayFunction(@(z) displayCallback(z, prog, v));
end

t_init = linspace(t0, tf, N);
if random_seed
  traj_init.x = DTTrajectory(t_init, randn(numel(x0), N));
else
  traj_init.x = ConstantTrajectory(x0);
  traj_init.u = ConstantTrajectory(zeros(6,1));
end
% profile off; profile on
[xtraj,utraj,z,F,info,infeasible_constraint_name] = solveTraj(prog,t_init,traj_init);
% profile off; profile viewer
  xtraj = xtraj.setOutputFrame(rbm.getStateFrame());

  x_data = xtraj.eval(xtraj.getBreaks());
  r_data = x_data(1:3, :);
  z_data = x_data(4:7, :);
  w_data = x_data(8:10, :);
  v_data = x_data(11:13, :);
  
  k_data = zeros(size(w_data));
  KE_data = zeros(1, N);
  for n = 1:N
    k_data(:, n) = quatRotateVec(z_data(:, n), I*w_data(:, n));
    KE_data(n) = 0.5*w_data(:, n)'*prog.I*w_data(:, n) + 0.5*prog.m*v_data(:, n)'*v_data(:, n);
  end
keyboard
end

function displayCallback(z, prog, v)
  t = cumsum([0; z(prog.h_inds(:))]);
  x = z(prog.x_inds);
%   subplot(2, 1, 1);
  plot(t, x(1:3,:)', '.-');
  drawnow;
%   xtraj = PPTrajectory(zoh(t, x(1:7, :)));
%   xtraj = xtraj.setOutputFrame(v.getInputFrame());
%   v.playback(xtraj);
end
