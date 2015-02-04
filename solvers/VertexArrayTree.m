classdef VertexArrayTree < MotionPlanningTree
  properties
    dim
    V
    parent
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
      [obj, id] = addVertex@MotionPlanningTree(obj, q, id_parent);
      obj.V(:,obj.n) = q; 
      obj.parent(obj.n) = id_parent; 
    end

    function [obj, id_last] = addPath(obj, q, id_parent)
      path_length = size(q, 2);
      for i = 1:path_length
        [obj, id_parent] = obj.addVertex(q(:,i), id_parent);
      end
      id_last = id_parent;
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

    function is_valid = isValidConfiguration(obj, q)
      is_valid = sizecheck(q, [obj.dim, 1]);
    end

  end
end
