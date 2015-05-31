function testRevoluteJointResiduals()
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

  %% Test a basic RevoluteJointResiduals object
  child = rbm.getBody(rbm.findLinkId('l_scap'));
  test_fcn = RevoluteJointResiduals(rbm,'l_scap','l_clav', child.Ttree, ...
                                        child.joint_axis);

  % Check the gradients of the DrakeFunction
  [f,df] = geval(@(x) eval(test_fcn, x),[q0; q0(child.position_num)],struct('grad_method',{{'user','numerical'}},'tol',1e-6));
end
