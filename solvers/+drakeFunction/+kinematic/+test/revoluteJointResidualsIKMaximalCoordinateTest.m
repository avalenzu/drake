function revoluteJointResidualsNonlinearProgramTest()
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
  n_bodies = rbm.getNumBodies();

  % Load nominal posture
  S = load(fullfile(getDrakePath(),'examples','Atlas','data','atlas_fp.mat'));
  q_star = S.xstar(1:nq);

  prog = InverseKinematicsMaximalCoordinates(rbm, q_star);
  v_max = prog.robot_max.constructVisualizer();

  prog = prog.addDisplayFunction(@(q) displayFun(v_max,q), prog.body_pose_idx);

  x0 = zeros(prog.num_vars, 1);
  [q, F, info] = prog.solve(q_star);

  keyboard

end

function displayFun(v, q)
  q = reshape(q, 6, []);
  xyz = q(1:3, :);
  exp_map = q(4:6, :);
  quat = zeros(4, size(exp_map, 2));
  for i = 1:size(exp_map, 2)
    quat(:, i) = expmap2quat(exp_map(:, i));
  end
  v.draw(0,reshape([xyz; quat], [], 1));
end
