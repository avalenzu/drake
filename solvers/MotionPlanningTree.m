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


    function [obj_new, id_last] = recursiveConnectSmoothing(obj, path_ids, n_iterations)
      if nargin < 3
        path_length = numel(path_ids);
        obj_start = obj.init(obj.getVertex(path_ids(1)));
        if path_length == 2
          % DEBUG
          %fprintf('DONE: path_length is 2\n');
          % END_DEBUG
          [obj_new, id_last] = obj_start.addVertex(obj.getVertex(path_ids(2)), 1);
        else
          [obj_new, status, id_last] = obj_start.connect(obj.getVertex(path_ids(end)));
          if status ~= obj.REACHED
            % DEBUG
            %fprintf('RECURSE: connect failed');
            % END_DEBUG
            %mid_idx = floor(path_length/2);
            mid_idx = randi([2,path_length-1]);
            [obj_new, id_last] = obj.recursiveConnectSmoothing(path_ids(1:mid_idx));
            obj_new2 = obj.recursiveConnectSmoothing(path_ids(mid_idx+1:end));
            for i = 1:obj_new2.n
              [obj_new, id_last] = obj_new.addVertex(obj_new2.getVertex(i), id_last);
            end
          end
          % DEBUG
          %fprintf('DONE: connect succeeded\n');
          % END_DEBUG
        end
      else
        for i = 1:n_iterations
          [obj, id_last] = recursiveConnectSmoothing(obj, path_ids);
          path_ids = obj.getPathToVertex(id_last);
          % DEBUG
          %path_ids
          % END_DEBUG
        end
        obj_new = obj;
      end
    end

    function obj = setLCMGL(obj, name, color)
      obj.lcmgl = LCMGLClient(name);
      sizecheck(color, 3);
      obj.line_color = color;
      obj.lcmgl.switchBuffers();
    end

    function [d, id_near] = nearestNeighbor(obj, q)
      d_all = obj.distanceMetric(q, obj.getVertex(1:obj.n));
      [d, id_near] = min(d_all);
    end

    function [obj, id] = addVertex(obj, q, parent_id)
      obj.n = obj.n + 1;
      id = obj.n;
    end

    function obj = drawTree(obj, ~)
    end
  end
end
