function revoluteJointResidualsIKMaximalCoordinateTest(visualize)
  import drakeFunction.*
  import drakeFunction.euclidean.*
  import drakeFunction.kinematic.*
  
  if nargin < 1
    visualize = false;
  end

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
  q_star = S.xstar(1:nq);

  prog = InverseKinematicsMaximalCoordinates(rbm, q_star);
  if visualize
    v_max = prog.robot_max.constructVisualizer();
    
    prog = prog.addDisplayFunction(@(q) displayFun(v_max,q), prog.body_pose_idx);
  end
%   prog = prog.addConstraint(ConstantConstraint([0;0;0;1;0;0;0]), prog.body_pose_idx(:,1));
  prog = prog.addConstraint(ConstantConstraint([0;0;0]), prog.body_pose_idx(1:3,1));

  [q, F, info] = prog.solve(q_star);

end

function displayFun(v, q)
%   q = reshape(q, 6, []);
%   xyz = q(1:3, :);
%   exp_map = q(4:6, :);
%   quat = zeros(4, size(exp_map, 2));
%   for i = 1:size(exp_map, 2)
%     quat(:, i) = expmap2quat(exp_map(:, i));
%   end
%   v.draw(0,reshape([xyz; quat], [], 1));
v.draw(0, q);
end
