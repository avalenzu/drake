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
                                {'x', 'y', 'z', 'v1', 'v2', 'v3'})]; %#ok
      end
      old_num_vars = obj.num_vars;
      obj = obj.addDecisionVariable(numel(var_names), var_names);
      obj.body_pose_idx = reshape((old_num_vars + 1):obj.num_vars, 6, []);
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
        if ~strcmp(body.jointname, 'base')
          parent_name = obj.robot.getLinkName(body.parent); 
          child_name = body.linkname; 
          child_Ttree = body.Ttree;
          child_joint_axis = body.joint_axis;
          child_position_num = body.position_num;
          joint_residual = RevoluteJointResiduals(obj.robot_max, parent_name, ...
            child_name, ...
            child_Ttree, ...
            child_joint_axis);
          joint_residual = joint_residual(postureExp2postureQuat);
          joint_residual_constraint = DrakeFunctionConstraint(zeros(4,1), ...
            zeros(4,1), ...
            joint_residual);

          obj = obj.addConstraint(joint_residual_constraint, ...
            [obj.body_pose_idx(:); obj.q_idx(child_position_num)]); 
          obj = obj.addConstraint(BoundingBoxConstraint(-pi*ones(3,1), ...
                                                        pi*ones(3,1)), ...
                                  obj.body_pose_idx(4:6, i-1)); 
        end
      end
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
