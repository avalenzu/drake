function relativeQuaternionTest()
  import drakeFunction.*
  import drakeFunction.euclidean.*
  import drakeFunction.kinematic.*

  %% Initial Setup
  % Create the robot
  w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
  options.floating = true;
  options.use_new_kinsol = true;
  urdf = fullfile(getDrakePath(),'examples','Atlas','urdf','atlas_minimal_contact.urdf');
  rbm = RigidBodyManipulator(urdf,options);
  warning(w);
  nq = rbm.getNumPositions();

  % Load nominal posture
  S = load(fullfile(getDrakePath(),'examples','Atlas','data','atlas_fp.mat'));
  q_nom = S.xstar(1:nq);
  q0 = q_nom;

  %% Test a basic RelativePosition object
  % Create a DrakeFunction that computes the relative orientation between the
  % hands
  hand_pts_fcn = RelativeQuaternion(rbm,'l_hand','r_hand');

  % Check the gradients of the DrakeFunction
  [f,df] = geval(@(q) eval(hand_pts_fcn,q),q0,struct('grad_method',{{'user','numerical'}},'tol',1e-6));
end
