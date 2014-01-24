%% Load data
load staticData

%% Create manipulator
S = warning('OFF','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('OFF','Drake:RigidBodyManipulator:UnsupportedJointLimits');
warning('OFF','Drake:RigidBody:SimplifiedCollisionGeometry');
rbm = PlanarRigidBodyManipulator('urdf/simple_quarter_atlas.urdf',struct('floating',true));
warning(S);
foot = length(rbm.body);
foot_frame = RigidBodyFrame(foot,zeros(3,1),zeros(3,1),'foot_origin');
rbm = addFrame(rbm,foot_frame);
rbm = rbm.compile();
tsrbm = TimeSteppingRigidBodyManipulator(rbm,0.001);
tsrbm = addSensor(tsrbm,FullStateFeedbackSensor());
tsrbm = addSensor(tsrbm,ContactForceTorqueSensor(tsrbm,foot_frame));
tsrbm = compile(tsrbm);
nq = rbm.getNumDOF();
nu = rbm.getNumInputs();
nx = tsrbm.getOutputFrame().dim; % NOT 2*nq
nf = tsrbm.getOutputFrame().getFrameByNum(2).dim;

%% Remove empty cells
idx_empty = cellfun('isempty',x_traj_data);
x_traj_data = x_traj_data(~idx_empty);

%% Split data into fall/not-fall
z_vel_norm = cellfun(@(traj) norm(traj.xx(nq+1,:)),x_traj_data);
idx_fall = z_vel_norm > 50;

x_traj_data_fall = x_traj_data(idx_fall);
x_traj_data_not_fall = x_traj_data(~idx_fall);
n = length(x_traj_data_not_fall);

%% Take mean of latter half of trajectories
x_data = zeros(nx,n);
for i = 1:n
  x_data(:,i) = mean(x_traj_data_not_fall{i}.xx(:,ceil(end/2):end),2);
end
q_data = x_data(1:nq,:);
qd_data = x_data(nq+(1:nq),:);
ft_data = x_data(2*nq+(1:nf),:);

%% Save data
save(['staticDataProcessed' datestr(now,'yyyy_mm_dd_HHMM')],'q_data','qd_data','ft_data','x_traj_data_not_fall');