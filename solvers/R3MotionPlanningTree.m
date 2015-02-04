classdef R3MotionPlanningTree < CartesianMotionPlanningTree
  methods
    function obj = R3MotionPlanningTree()
      obj = obj@CartesianMotionPlanningTree(3);
    end

    function drawTree(obj, ~, draw_now)
      if nargin < 3, draw_now = true; end
      obj.lcmgl.glColor3f(obj.line_color(1), obj.line_color(2), obj.line_color(3));
      for jj = 2:obj.n
        obj.lcmgl.plot3(obj.V(1, [obj.parent(jj),jj]), obj.V(2, [obj.parent(jj),jj]), obj.V(3, [obj.parent(jj),jj]));
      end
      if draw_now
        obj.lcmgl.switchBuffers();
      end
    end

    function drawPath(obj, varargin)
      q_path = extractPath(obj, varargin{:});
      obj.lcmgl.glLineWidth(2);
      obj.lcmgl.glColor3f(0,1,0);
      obj.lcmgl.plot3(q_path(1,:), q_path(2,:), q_path(3,:));
      obj.lcmgl.switchBuffers();
    end
  end
end
