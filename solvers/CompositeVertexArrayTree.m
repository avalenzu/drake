classdef CompositeVertexArrayTree < VertexArrayTree
  properties
    TA
    TB
    weightB
    idxA
    idxB
  end
  methods
    function obj = CompositeVertexArrayTree(TA, TB)
      typecheck(TA, 'VertexArrayTree');
      typecheck(TB, 'VertexArrayTree');
      obj = obj@VertexArrayTree(TA.dim + TB.dim);
      obj.TA = TA;
      obj.TB = TB;
      obj.idxA = 1:obj.TA.dim;
      obj.idxB = obj.TA.dim + (1:obj.TB.dim);
      obj.weightB = 1;
    end

    function d = distanceMetric(obj, q1, q_array)
      d = obj.TA.distanceMetric(q1(obj.idxA,:), q_array(obj.idxA, :));
      if obj.weightB > 0
        d = d + obj.weightB*obj.TB.distanceMetric(q1(obj.idxB,:), q_array(obj.idxB, :));
      end
    end

    function q = interpolate(obj, q1, q2, interpolation_factors)
      q = zeros(obj.TA.dim + obj.TB.dim, numel(interpolation_factors));
      q(obj.idxA,:) = obj.TA.interpolate(q1(obj.idxA), q2(obj.idxA), interpolation_factors);
      q(obj.idxB,:) = obj.TB.interpolate(q1(obj.idxB), q2(obj.idxB), interpolation_factors);
    end

    %function q = getVertex(obj, id)
      %q = zeros(obj.TA.dim + obj.TB.dim, numel(id));
      %q(obj.idxA,:) = obj.TA.getVertex(id);
      %q(obj.idxB,:) = obj.TB.getVertex(id);
    %end

    function [obj, id_last] = init(obj, q_init)
      %obj.n = 1;
      obj = init@VertexArrayTree(obj, q_init);
      [obj.TA, id_last] = obj.TA.init(q_init(obj.idxA));
      obj.TB = obj.TB.init(q_init(obj.idxB));
    end

    function valid = isValidConfiguration(obj, q)
      valid = isValidConfiguration@VertexArrayTree(obj, q) && ...
              obj.TA.isValidConfiguration(q(obj.idxA)) && ...
              obj.TB.isValidConfiguration(q(obj.idxB));
    end

    function [obj, id_new] = addVertex(obj, q, id_parent)
      obj = addVertex@VertexArrayTree(obj,q,id_parent);
      [obj.TA, id_new] = obj.TA.addVertex(q(obj.idxA), id_parent);
      obj.TB = obj.TB.addVertex(q(obj.idxB), id_parent);
    end

    function q = randomConfig(obj)
      q = zeros(obj.TA.dim + obj.TB.dim, 1);
      q(obj.idxA) = obj.TA.randomConfig();
      q(obj.idxB) = obj.TB.randomConfig();
    end

    %function path_ids = getPathToVertex(obj, leaf_id)
      %path_ids = leaf_id;
      %while path_ids(1) > 1
        %path_ids = [obj.TA.parent(path_ids(1)),path_ids];
      %end
    %end

    function q_path = extractPath(objA, path_ids_A, objB, path_ids_B)
      path_length = numel(path_ids_A);
      if nargin > 2
        path_length = path_length + numel(path_ids_B) - 1;
      end
      q_path = zeros(objA.TA.dim + objA.TB.dim, path_length);
      if nargin > 2
        q_path(objA.idxA, :) = extractPath(objA.TA, path_ids_A, ...
                                           objB.TA, path_ids_B);
        q_path(objA.idxB, :) = extractPath(objA.TB, path_ids_A, ...
                                           objB.TB, path_ids_B);
      else
        q_path(objA.idxA, :) = extractPath(objA.TA, path_ids_A);
        q_path(objA.idxB, :) = extractPath(objA.TB, path_ids_A);
      end
    end
  end
end
