classdef ConfigurationSpaceMotionPlanningTree < CartesianMotionPlanningTree
  properties
    rbm
    visualization_point = struct('body', 1, 'pt', [0;0;0]);
    min_distance = 0.1;
    active_collision_options = [];
    v
    xyz_vis
  end
  methods
    function obj = ConfigurationSpaceMotionPlanningTree(rbm)
      obj = obj@CartesianMotionPlanningTree(rbm.getNumPositions());
      obj.rbm = rbm;
      [obj.lb, obj.ub] = rbm.getJointLimits();
      obj.lb(isinf(obj.lb)) = -10;
      obj.ub(isinf(obj.ub)) = 10;
      %obj.v = obj.rbm.constructVisualizer(struct('use_collision_geometry',true));
      obj = obj.compile();
    end

    function obj = compile(obj)
      %collision_constraint = DrakeFunctionConstraint(0,1e-8,drakeFunction.kinematic.SmoothDistancePenalty(obj.rbm,obj.min_distance,obj.active_collision_options));
      obj.constraints = {};
      obj.x_inds = {};
      %obj = obj.addConstraint(collision_constraint);
      obj.xyz_vis = NaN(3, obj.N);
    end

    function valid = checkConstraints(obj, q)
      valid = checkConstraints@CartesianMotionPlanningTree(obj, q);
      valid = valid && isempty(obj.rbm.allCollisions(q, obj.min_distance));
    end

    function obj = addGeometryToWorld(obj, geom)
      obj.rbm = obj.rbm.addGeometryToBody(1, geom);
      obj = obj.compile();
    end

    function q = randomConfig(obj)
      q = randomConfig@CartesianMotionPlanningTree(obj);
    end

    function obj = drawTree(obj, n_at_last_draw, draw_now)
      if nargin < 2, n_at_last_draw = 1; end
      if nargin < 3, draw_now = true; end
      obj.lcmgl.glColor3f(obj.line_color(1), obj.line_color(2), obj.line_color(3));
      for i = 1:obj.n
        if i >= n_at_last_draw
          kinsol = obj.rbm.doKinematics(obj.getVertex(i));
          obj.xyz_vis(:,i) = obj.rbm.forwardKin(kinsol, obj.visualization_point.body, obj.visualization_point.pt);
        end
        xyz_current = obj.xyz_vis(:, i);
        xyz_parent = obj.xyz_vis(:, obj.parent(i));
        xyz = [xyz_parent, xyz_current];
        obj.lcmgl.plot3(xyz(1,:), xyz(2,:), xyz(3,:));
      end
      if draw_now
        obj.lcmgl.switchBuffers();
      end
    end

    function drawPath(obj, varargin)
      q_path = extractPath(obj, varargin{:});
      path_length = size(q_path, 2);
      xyz = NaN(3, path_length);
      for i = 1:path_length
        kinsol = obj.rbm.doKinematics(q_path(:,i));
        xyz(:,i) = obj.rbm.forwardKin(kinsol, obj.visualization_point.body, obj.visualization_point.pt);
      end
      obj.lcmgl.glLineWidth(2);
      obj.lcmgl.glColor3f(0,1,0);
      obj.lcmgl.plot3(xyz(1,:), xyz(2,:), xyz(3,:));
      obj.lcmgl.switchBuffers();
    end
  end
end
