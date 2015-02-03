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
    q = randomConfig(obj);
    d = distanceMetric(obj, q1, q_array)
    is_valid = isValidConfiguration(obj, q);
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
    function [obj, status, id_new] = extend(obj, q)
      [q_new, id_near] = newConfig(obj, q);
      if ~isempty(q_new) && obj.constraint_fcn(q_new)
        [obj, id_new] = obj.addVertex(q_new, id_near);
        if q_new == q
          status = obj.REACHED;
        else
          status = obj.ADVANCED;
        end
      else
        id_new = [];
        status = obj.TRAPPED;
      end
    end

    function [obj, status, id_last] = connect(obj, q)
      status = obj.ADVANCED;
      while status == obj.ADVANCED
        [obj, status, id_last] = extend(obj, q);
      end
    end

    function q_path = extractPath(TA, path_ids_A, TB, path_ids_B)
      if nargin > 2
        q_path = [TA.getVertex(path_ids_A), TB.getVertex(fliplr(path_ids_B(1:end-1)))];
      else
        q_path = TA.getVertex(path_ids_A);
      end
    end
    function obj = setLCMGL(obj, name, color)
      obj.lcmgl = LCMGLClient(name);
      sizecheck(color, 3);
      obj.line_color = color;
    end

    function [d, id_near] = nearestNeighbor(obj, q)
      d_all = obj.distanceMetric(q, obj.getVertex(1:obj.n));
      [d, id_near] = min(d_all);
    end

    function obj = addVertex(obj)
      obj.n = obj.n + 1;
    end

    function drawTree(~, ~)
    end
  end
end
