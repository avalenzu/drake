classdef VertexArrayTree < MotionPlanningTree
  properties
    dim
    V
    parent
  end

  methods (Abstract)
    d = distanceMetric(obj, q1, q_array)
  end

  methods
    function obj = VertexArrayTree(dim)
      obj = obj@MotionPlanningTree();
      obj.dim = dim;
    end

    function obj = init(obj, q_init)
      obj = init@MotionPlanningTree(obj);
      sizecheck(q_init, 'colvec');
      obj.V = NaN(obj.dim, obj.N);
      obj.parent = NaN(1, obj.N);
      obj = obj.addVertex(q_init, 1);
    end

    function [obj, id] = addVertex(obj, q, id_parent)
      obj.n = obj.n + 1;
      obj.V(:,obj.n) = q; 
      obj.parent(obj.n) = id_parent; 
      id = obj.n;
    end

    function [d, id_near] = nearestNeighbor(obj, q)
      d_all = obj.distanceMetric(q, obj.V(:,1:obj.n));
      [d, id_near] = min(d_all);
    end

    function q = getVertex(obj, id)
      q = obj.V(:, id);
    end

    function path_ids = getPathToVertex(obj, leaf_id)
      path_ids = leaf_id;
      while path_ids(1) > 1
        path_ids = [obj.parent(path_ids(1)),path_ids];
      end
    end

    function q_path = extractPath(TA, path_ids_A, TB, path_ids_B)
      if nargin > 2
        q_path = [TA.V(:,path_ids_A), fliplr(TB.V(:,path_ids_B))];
      else
        q_path = TA.V(:,path_ids_A);
      end
    end

    function is_valid = isValidConfiguration(obj, q)
      is_valid = sizecheck(q, [obj.dim, 1]);
    end

  end
end
