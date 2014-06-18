classdef DummyDynamicsFullKinematicsPlanner < SimpleDynamicsFullKinematicsPlanner
  methods
    function obj = DummyDynamicsFullKinematicsPlanner(varargin)
      obj = obj@SimpleDynamicsFullKinematicsPlanner(varargin{:});
    end
    function obj = addDynamicConstraints(obj)
      % obj = addDynamicConstraints(obj,cnstr) adds a dynamic constraint
      % to the planner.
      % @param cnstr  -- Dynamics constraint
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      N = obj.N;
      
      dyn_inds = cell(N-1,1);      
      n_vars = 2*nX + nU + 1;

      cnstr = NonlinearConstraint(zeros(nX,1),zeros(nX,1),n_vars,@constraint_fun);
      
      for i=1:obj.N-1,        
        dyn_inds{i} = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i)};
        obj = obj.addNonlinearConstraint(cnstr, dyn_inds{i});
      end

      function [f,df] = constraint_fun(h,x0,x1,u)
        f = x1-x0;
        df = [zeros(nX,1),-eye(nX),eye(nX),zeros(nX,nU)];
      end
    end
    function obj = addRunningCost(obj,running_cost)
      for i=1:obj.N-1,
        h_ind = obj.h_inds(i);
        x_ind = obj.x_inds(:,i);
        u_ind = obj.u_inds(:,i);
        
        obj = obj.addCost(running_cost,{h_ind;x_ind;u_ind});
      end
    end
  end
end
