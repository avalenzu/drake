classdef SE3MotionPlanningTree < CompositeVertexArrayTree
  properties
    rbm
    min_distance = 0.2;
    min_distance_penalty
  end

  methods
    function obj = SE3MotionPlanningTree()
      TA = R3MotionPlanningTree();
      TB = SO3MotionPlanningTree();
      obj = obj@CompositeVertexArrayTree(TA, TB);
      obj = obj.setLCMGL('SE3MotionPlanningTree',[0, 0, 0]);
      urdf = fullfile(getDrakePath, 'systems', 'plants', 'test', 'FallingBrick.urdf');
      options.floating = true;
      obj.rbm = RigidBodyManipulator(urdf, options);
      obj.rbm = obj.rbm.removeCollisionGroupsExcept({});
      %obj.rbm = RigidBodyManipulator();
      %obj.rbm.name{1} = 'robot';
      %body = RigidBody();
      %body = body.setInertial(1, zeros(3,1), eye(3));
      %body.linkname = 'body';
      %body.robotnum = 1;
      %obj.rbm = obj.rbm.addLink(body);
      %obj.rbm = obj.rbm.addFloatingBase(1, 2, [0;0;0], [0;0;0],'quat'); 
      %obj.rbm = obj.rbm.addFloatingBase(1, 2, [0;0;0], [0;0;0]); 
      obj = obj.compile();
      %obj = obj.addConstraint(FunctionHandleConstraint(0, 1e-8, 7, ...
                                                       %@obj.isCollisionFree));
    end

    function obj = compile(obj)
      obj.rbm = obj.rbm.compile();
    end

    %function [c, dc] = isCollisionFree(obj, q)
      %xyz = q(1:3);
      %quat = q(4:7); 
      %rpy = quat2rpy(quat);
      %kinsol = obj.rbm.doKinematics([xyz; rpy]);
      %[c, dc] = smoothDistancePenaltymex(obj.rbm.getMexModelPtr, obj.min_distance);
    %end

    function valid = checkConstraints(obj, q)
      xyz = q(1:3);
      quat = q(4:7); 
      rpy = quat2rpy(quat);
      valid = checkConstraints@CompositeVertexArrayTree(obj, q);
      valid = valid && isempty(obj.rbm.allCollisions([xyz; rpy], obj.min_distance));
    end

    function obj = addGeometryToRobot(obj, geom)
      obj.rbm = obj.rbm.addGeometryToBody(2, geom);
      obj = obj.compile();
    end

    function obj = addGeometryToWorld(obj, geom)
      obj.rbm = obj.rbm.addGeometryToBody(1, geom);
      obj = obj.compile();
    end

    function obj = setOrientationWeight(obj, orientation_weight)
      obj.weightB = orientation_weight;
    end

    function obj = setTranslationBounds(obj, lb, ub)
      obj.TA.lb = lb;
      obj.TA.ub = ub;
    end

    function obj = setLCMGL(obj, varargin)
      obj = setLCMGL@CompositeVertexArrayTree(obj, varargin{:});
      obj.TA = obj.TA.setLCMGL(varargin{:});
    end

    function obj = drawTree(obj, varargin)
      obj.TA.drawTree(varargin{:});
    end

    function drawPath(objA, path_ids_A, objB, path_ids_B)
      if nargin > 2
        drawPath(objA.TA, path_ids_A, objB.TA, path_ids_B);
      else
        drawPath(objA.TA, path_ids_A);
      end
    end
  end
end
