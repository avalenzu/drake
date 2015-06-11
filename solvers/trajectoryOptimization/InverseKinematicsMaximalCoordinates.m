classdef InverseKinematicsMaximalCoordinates < InverseKinematics
  properties
    robot_max
    body_pose_idx
  end
  methods
    function obj = InverseKinematicsMaximalCoordinates(robot, q_nom)
      obj = obj@InverseKinematics(robot, q_nom);
      obj.robot_max = obj.convertToMaximalModel(robot);
      obj = obj.addBodyPoseDecisionVariables();
      obj = obj.addJointConstraints();
    end

    function obj = addBodyPoseDecisionVariables(obj)
      var_names = {};
      for i = 2:obj.robot_max.getNumBodies()
        var_names = [var_names, ...
                     cellStrCat(obj.robot_max.getLinkName(i), ...
                                {'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'})]; %#ok
      end
      old_num_vars = obj.num_vars;
      obj = obj.addDecisionVariable(numel(var_names), var_names);
      obj.body_pose_idx = reshape((old_num_vars + 1):obj.num_vars, 7, []);
    end
    
    function obj = addJointConstraints(obj)
      import drakeFunction.frames.*
      import drakeFunction.kinematic.*
      R3 = realCoordinateSpace(3);
      R1 = realCoordinateSpace(1);
      poseExp2poseQuat = [drakeFunction.Identity(R3); drakeFunction.geometry.Exp2Quat()];
      postureExp2postureQuat = poseExp2poseQuat.duplicate(obj.robot_max.getNumBodies() - 1);
      postureExp2postureQuat = compose(drakeFunction.Linear(postureExp2postureQuat.output_frame, obj.robot_max.getPositionFrame(), eye(obj.robot_max.getNumPositions())), postureExp2postureQuat);
      postureExp2postureQuat = [postureExp2postureQuat;drakeFunction.Identity(R1)];
      for i = 2:obj.robot.getNumBodies()
        body = obj.robot.getBody(i);
        parent_name = obj.robot.getLinkName(body.parent);
        child_name = body.linkname;
        parent_idx = obj.robot_max.findLinkId(parent_name) - 1;
        child_idx = obj.robot_max.findLinkId(child_name) - 1;
        child_Ttree = body.Ttree;
        child_joint_axis = body.joint_axis;
        child_position_num = body.position_num;
        if ~strcmp(body.jointname, 'base')
          joint_residual = RevoluteJointResiduals(child_Ttree, ...
                                                  child_joint_axis);
%           joint_residual = joint_residual(postureExp2postureQuat);
          joint_residual_constraint = DrakeFunctionConstraint(zeros(4,1), ...
            zeros(4,1), ...
            joint_residual);

          obj = obj.addConstraint(joint_residual_constraint, ...
            [reshape(obj.body_pose_idx(:,[parent_idx, child_idx]),[],1); obj.q_idx(child_position_num)]); 
        end
        obj = obj.addConstraint(QuadraticConstraint(1, 1, 2*eye(4), zeros(4,1)), ...
          obj.body_pose_idx(4:7, child_idx));
      end
        obj = obj.addConstraint(BoundingBoxConstraint(-ones(numel(obj.body_pose_idx(4:7, :)),1), ones(numel(obj.body_pose_idx(4:7, :)),1)), obj.body_pose_idx(4:7, :));
    end
    
    function [x,F,info,infeasible_constraint] = solve(obj,q_seed)
      x0 = rand(obj.num_vars,1);
      x0(obj.body_pose_idx(4,:), 1);
      x0(obj.q_idx) = q_seed;
      if(~isempty(obj.qsc_weight_idx))
        x0(obj.qsc_weight_idx) = 1/length(obj.qsc_weight_idx);
      end
      [x,F,info,infeasible_constraint] = solve@NonlinearProgram(obj,x0);
      q = x(obj.q_idx);
      q = max([obj.x_lb(obj.q_idx) q],[],2);
      q = min([obj.x_ub(obj.q_idx) q],[],2);
    end
  end

  methods (Static)
    function r_max = convertToMaximalModel(r)
      options.floating = 'quat';
      options.use_new_kinsol = true;
      r_max = RigidBodyManipulator([], options);
      [r_max, robotnum] = r_max.addEmptyRobot('test');
      for i = 2:r.getNumBodies()
        body = r.getBody(i);
        body.robotnum = robotnum;
        r_max = r_max.addLink(body);
        r_max = r_max.addJoint([body.linkname, '_pose'], 'floating_quat', 1, i, [0;0;0], [0;0;0]);
      end
      r_max = r_max.compile();
    end
  end
end
