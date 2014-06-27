classdef MinDistanceConstraint < SingleTimeKinematicConstraint
  % Constrains the closest distance any pair of bodies to be greater
  % than  min_distance
  % @param min_distance    -- a scalar, the lower bound of the distance
  % @param tspan   -- a 1x2 vector, the time span of the constraint being
  %                   active
  properties
    min_distance
  end
  
  methods(Access=protected)
   function [c,dc] = evalValidTime(obj,kinsol)
     if isstruct(kinsol)
       q = kinsol.q;
     else
       q = kinsol;
     end
     kinsol = doKinematics(obj.robot,q,true);
     [dist,ddist_dq] = closestDistance(obj.robot,kinsol);
     [scaled_dist,dscaled_dist_ddist] = scaleDistance(obj,dist);
     [pairwise_costs,dpairwise_cost_dscaled_dist] = penalty(obj,scaled_dist);
     c = sum(pairwise_costs);
     dcost_dscaled_dist = sum(dpairwise_cost_dscaled_dist,1);
     dc = dcost_dscaled_dist*dscaled_dist_ddist*ddist_dq;
     %if c > 1e-6
       %fprintf('%6.3e\n',c);
     %end
    end
  end

  methods
    function obj = MinDistanceConstraint(robot,min_distance,tspan)
      if(nargin == 2)
        tspan = [-inf inf];
      end
      sizecheck(min_distance,[1,1]);
      ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.MinDistanceConstraintType,robot.getMexModelPtr,min_distance,tspan);
      obj = obj@SingleTimeKinematicConstraint(robot,tspan);
      obj.type = RigidBodyConstraint.MinDistanceConstraintType;
      obj.min_distance = min_distance;
      obj.mex_ptr = ptr;
      obj.num_constraint = 1;
    end

    function [scaled_dist,dscaled_dist_ddist] = scaleDistance(obj,dist)
      recip_min_dist = 1/obj.min_distance;
      scaled_dist = recip_min_dist*dist - 1;
      dscaled_dist_ddist = recip_min_dist*eye(size(dist,1));
    end

    function [cost, dcost_ddist] = penalty(obj,dist)
      idx_neg = find(dist < 0);
      cost = zeros(size(dist));
      dcost_ddist = zeros(numel(dist));
      exp_recip_dist = exp(dist(idx_neg).^(-1));
      cost(idx_neg) = -dist(idx_neg).*exp_recip_dist;
      dcost_ddist(sub2ind(size(dcost_ddist),idx_neg,idx_neg)) = ...
        exp_recip_dist.*(dist(idx_neg).^(-1) - 1);
    end

    function num = getNumConstraint(obj,t)
      if obj.isTimeValid(t)
        num = 1;
      else
        num = 0;
      end
    end

    function obj = updateRobot(obj,robot)
      obj.robot = robot;
      obj.mex_ptr = updatePtrRigidBodyConstraintmex(obj.mex_ptr,'robot',robot.getMexModelPtr);
    end

    function [lb,ub] = bounds(obj,t)
      if obj.isTimeValid(t)
        lb = 0;
        ub = 0;
      else
        lb = [];
        ub = [];
      end
    end

    function name_str = name(obj,t)
      name_str = repmat({'Minimum distance constraint'},obj.getNumConstraint(t),1);
    end
  end
end
