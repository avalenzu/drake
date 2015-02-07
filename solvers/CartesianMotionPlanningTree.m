classdef CartesianMotionPlanningTree < VertexArrayTree
  properties
    lb = 0;
    ub = 1;
  end
  methods
    function obj = CartesianMotionPlanningTree(dim)
      obj = obj@VertexArrayTree(dim);
    end

    function q = randomConfig(obj)
      q = obj.lb + (obj.ub-obj.lb).*rand(obj.dim,1);
    end

    function q = interpolate(obj, q1, q2, f)
      q = bsxfun(@times,1-f,q1) + bsxfun(@times,f,q2);
    end

    function obj = drawTree(obj, n_at_last_draw)
      line([obj.V(1,(n_at_last_draw+1):obj.n);obj.V(1,obj.parent((n_at_last_draw+1):obj.n))],[obj.V(2,n_at_last_draw+1:obj.n);obj.V(2,obj.parent(n_at_last_draw+1:obj.n))],'Color',0.3*[1 1 1],'Marker','.');
      axis equal
    end

    function d = distanceMetric(obj, q1, q_array)
      d = sqrt(sum(bsxfun(@minus, q1, q_array).^2,1));
      %d = sqrt(sum((q_array-repmat(q1,1,size(q_array,2))).^2,1));
    end
  end
end
