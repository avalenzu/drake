classdef MotionPlanningTree
  properties (Constant)
    TRAPPED = 0;
    REACHED = 1;
    ADVANCED = 2;
  end
  
  properties
    constraint_fcn = @(q)true;
    N = 1e4; % Number of nodes for which memory should be pre-allocated
    n = 0; % Number of nodes in the tree
    lcmgl;
    max_edge_length = 0.1;
    max_length_between_constraint_checks = 0.01;
    line_color;
  end

  methods (Abstract)
    q = randomConfig(T);
    id_near = nearestNeighbor(T, q);
    T = addVertex(T, q, id_parent);
    is_valid = isValidConfiguration(T, q);
    q = getVertex(obj, id);
    q = interpolate(obj, q1, q2, interpolation_factors);
  end

  methods
    function obj = init(obj, q_init)
      obj.n = 0;
    end

    function [q_new, id_near] = newConfig(obj, q)
      [d, id_near] = nearestNeighbor(obj, q);
      if d < obj.max_edge_length
        q_new = q;
      else
        alpha = obj.max_edge_length/d;
        q_new = obj.interpolate(obj.getVertex(id_near), q, alpha);
        d = obj.max_edge_length;
      end
      if d > obj.max_length_between_constraint_checks
        num_interpolated_checks = 2+ceil(d/obj.max_length_between_constraint_checks);
        interpolation_factors = linspace(0, 1, num_interpolated_checks);
        xss = obj.interpolate(obj.getVertex(id_near), q, interpolation_factors);
        valid = true;
        for i=2:num_interpolated_checks-1
          if ~obj.constraint_fcn(xss(:,i)),
            valid=false;
            break;
          end
        end
        if ~valid
          q_new = [];
        end
      end
    end
    function [T, status, id_new] = extend(T, q)
      [q_new, id_near] = newConfig(T, q);
      if ~isempty(q_new) && T.constraint_fcn(q_new)
        [T, id_new] = T.addVertex(q_new, id_near);
        if q_new == q
          status = T.REACHED;
        else
          status = T.ADVANCED;
        end
      else
        id_new = [];
        status = T.TRAPPED;
      end
    end

    function [T, status, id_last] = connect(T, q)
      status = T.ADVANCED;
      while status == T.ADVANCED
        [T, status, id_last] = extend(T, q);
      end
    end

    function obj = setLCMGL(obj, name, color)
      obj.lcmgl = LCMGLClient(name);
      sizecheck(color, 3);
      obj.line_color = color;
    end

    function drawTree(~, ~)
    end
  end
end
