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

  options.floating = 'quat';
  options.rotation_type = 2;
  options.base_or_frame_id = 1;
  %options.terrain = RigidBodyFlatTerrain();
  options.ignore_self_collisions = true;
  r_max = RigidBodyManipulator([], options);
  q_nom = zeros(7, n_bodies-1);
  [r_max, robotnum] = r_max.addEmptyRobot('test');
  kinsol = rbm.doKinematics(q_star);
  parent_name = cell(n_bodies-1, 1);
  child_name = cell(n_bodies-1, 1);
  child_Ttree = cell(n_bodies-1, 1);
  child_joint_axis = cell(n_bodies-1, 1);
  for i = 2:n_bodies
    body = rbm.getBody(i);
    body.robotnum = robotnum;
    r_max = r_max.addLink(body);
    r_max = r_max.addJoint([body.linkname, '_pose'], 'floating_quat', 1, i, [0;0;0], [0;0;0]);
    q_nom(:,i-1) = rbm.forwardKin(kinsol, i, [0;0;0], options);
    parent_name{i-1} = rbm.getLinkName(body.parent); 
    child_name{i-1} = body.linkname; 
    child_Ttree{i-1} = body.Ttree;
    child_joint_axis{i-1} = body.joint_axis;
    child_position_num{i-1} = body.position_num;
  end
  r_max = r_max.compile();

  q_nom = q_nom(:);
  q0 = getRandomConfiguration(r_max);

  v_max = r_max.constructVisualizer();
  v_max.draw(0,q0)

  prog = InverseKinematics(r_max, q0);
  prog = prog.addDisplayFunction(@(q) displayFun(v_max,q), prog.q_idx);
  prog = prog.setQ(0*prog.Q);
  prog = prog.addDecisionVariable(rbm.getNumPositions(), rbm.getPositionFrame().coordinates);
  joint_angle_idx = (prog.num_vars-rbm.getNumPositions()+1):prog.num_vars;

  quat_idx = reshape(prog.q_idx, 7, []);
  quat_idx(1:3,:) = [];
  norm_squared = NormSquared(drakeFunction.frames.Quaternion());
  [jlmin, jlmax] = rbm.getJointLimits();
  for i = 1:(n_bodies-1)
    prog = prog.addConstraint(DrakeFunctionConstraint(1, 1, norm_squared), quat_idx(:,i));
    if rbm.findLinkId(parent_name{i}) ~= 1
      joint_residuals = RevoluteJointResiduals(r_max, parent_name{i}, ...
                                                  child_name{i}, ...
                                                  child_Ttree{i}, ...
                                                  child_joint_axis{i});
      joint_residual_constraint = DrakeFunctionConstraint(zeros(4,1), ...
                                                          zeros(4,1), ...
                                                          joint_residuals);
 
      prog = prog.addConstraint(joint_residual_constraint, ...
                                [prog.q_idx; joint_angle_idx(child_position_num{i})]);
      prog = prog.addConstraint( ...
                BoundingBoxConstraint(jlmin(child_position_num{i}), ...
                                      jlmax(child_position_num{i})), ...
                joint_angle_idx(child_position_num{i}));
    end
  end

  prog = prog.addCost(QuadraticConstraint(-Inf, Inf, eye(nq), -q_star), joint_angle_idx);

  [q, F, info] = prog.solve(q0);

end

function displayFun(v, q)
  v.draw(0, q);
end
