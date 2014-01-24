rng(3)
S = warning('OFF','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('OFF','Drake:RigidBodyManipulator:UnsupportedJointLimits');
warning('OFF','Drake:RigidBody:SimplifiedCollisionGeometry');
rbm = PlanarRigidBodyManipulator('urdf/simple_quarter_atlas.urdf',struct('floating',true));
rbm_w_params = PlanarRigidBodyManipulator('urdf/simple_quarter_atlas_param.urdf',struct('floating',true));
warning(S);
foot = length(rbm.body);
foot_frame = RigidBodyFrame(foot,zeros(3,1),zeros(3,1),'foot_origin');
rbm = addFrame(rbm,foot_frame);
rbm = rbm.compile();
tsrbm = TimeSteppingRigidBodyManipulator(rbm,0.001);
nq = rbm.getNumDOF();
nu = rbm.getNumInputs();
% encoders = RigidBodyJointSensor(tsrbm.getManipulator());
% encoders.dof_mask = 4:nq;
% tsrbm = addSensor(tsrbm,encoders);
%% Add sensors
tsrbm = addSensor(tsrbm,FullStateFeedbackSensor());
tsrbm = addSensor(tsrbm,ContactForceTorqueSensor(tsrbm,foot_frame));
tsrbm = compile(tsrbm);
nx = tsrbm.getOutputFrame().getFrameByNum(1).dim;
nf = tsrbm.getOutputFrame().getFrameByNum(2).dim;
%%
% v = tsrbm.constructVisualizer();
% v.debug = true;
foot_pts = rbm.body(foot).getContactPoints();
qsc = QuasiStaticConstraint(rbm);
qsc = qsc.addContact(foot,foot_pts);
qsc = qsc.setActive(true);
qsc = qsc.setShrinkFactor(0.7);
kc_foot = WorldPositionConstraint(rbm,foot,foot_pts,[nan(2,4);zeros(1,4)],[nan(2,4);zeros(1,4)]);
above_ground_kc = WorldCoMConstraint(rbm,[0;0;0],[0;Inf;Inf]);
%%
[joint_limits_min,joint_limits_max] = rbm.getJointLimits();
% joint_limits_min(4:6) = -pi/10;
% joint_limits_max(4:6) = pi/10;
ikoptions = IKoptions(rbm);
ikoptions = ikoptions.setMex(true);
cost = Point(rbm.getStateFrame(),1);
cost.base_z = 0;
cost.base_relative_pitch = 0;
ikoptions = ikoptions.setQ(diag(cost(1:nq)));
%%
Kp = diag([5e3,5e3,1e4,1e4]);
Kd = 1e-1*eye(nu);
%%
sys = tsrbm.pdcontrol(Kp,Kd);
%%
n = 1000;
t_data = zeros(1,n);
q_data = zeros(nq,n);
ft_data = zeros(3,n);
x_traj_data = cell(1,n);
i = 1;
while i < n
  tic;
  fprintf('Iteration %d\n',i);
  q_nom = Point(rbm.getStateFrame(), ...
                [rand(nq,1).*(joint_limits_max - joint_limits_min) + joint_limits_min; ...
                zeros(nq,1)]);
  q_nom.base_x = 0;
  q_nom.base_z = 1;
  q_nom.base_relative_pitch = 0;
%     v.draw(0,q_nom);
  
  %[q,info] = inverseKin(rbm,q_nom,q_nom,qsc,kc_foot,above_ground_kc,ikoptions);
  [q,info] = inverseKin(rbm,q_nom(1:nq),q_nom(1:nq),qsc,kc_foot,ikoptions);
  x0 = inFrame(Point(rbm.getStateFrame(),[q;zeros(size(q))]),tsrbm.getStateFrame());
  xstar = x0;
  if info == 1
    success = true;
%     try
%       [xstar,ustar,success] = findFixedPoint(tsrbm,x0,zeros(nu,1));
%     catch ex
%       success = false;
%     end
  else
    success = false;
  end
  if success
    sys_sim = cascade(ConstantTrajectory(Point(sys.getInputFrame,xstar(tsrbm.getActuatedJoints))),sys);
    try
      [x_traj,y_traj] = simulate(sys_sim,[0,5],[xstar]);
%       x_traj_vis = PPTrajectory(foh(x_traj.tt,x_traj.xx));
%       x_traj_vis = x_traj_vis.setOutputFrame(v.getInputFrame());
%       v.playback(x_traj_vis)
    catch ex
      success = false;
    end
  end
  if success
    x_traj_data{i} = x_traj;
  else
    disp('Failed to find fixed point near this seed.');
  end
  save('staticData','x_traj_data');
  toc;
end;
