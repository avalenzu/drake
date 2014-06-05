classdef WholeBodyPlanner < InverseKinTraj
  properties
    dynamics_constraint;
    tracking_error;
    qd_idx;
    v_nom;
  end
  methods
    function obj = WholeBodyPlanner(robot,t,q_nom_traj, ...
                                    fix_initial_state,x0,varargin)
      obj = obj@InverseKinTraj(robot,t,q_nom_traj,fix_initial_state, ...
                               x0,varargin{:});
      obj.Qv = eye(obj.nq);
      qd_name = cell(obj.nq,obj.nT-2);
      for j = 1:(obj.nT-2)
        for i = 1:obj.nq
          qd_name{i,j} = sprintf('qd%d[%d]',i,j+1);
        end
      end
      qd_name = reshape(qd_name,obj.nq*(obj.nT-2),1);
      obj.qd_idx = [obj.qd0_idx, ...
                    reshape(obj.num_vars+(1:obj.nq*(obj.nT-2)), ...
                            obj.nq,obj.nT-2), ...
                    obj.qdf_idx];
      obj = obj.addDecisionVariable(obj.nq*(obj.nT-2),qd_name);
      obj = obj.setLinearDynamics();
      obj = obj.setTrackingError(obj.Q,obj.Qv,obj.Qa);
    end

    function obj = setLinearDynamics(obj)
      obj.dynamics_constraint = ...
        LinearDynamics(obj.t_knot,obj.nq, 1:obj.nq*obj.nT, ...
                       obj.nq*obj.nT+(1:obj.nq*obj.nT));
      xind = [obj.q_idx(:);obj.qd_idx(:)];
      obj = obj.addLinearConstraint(obj.dynamics_constraint,xind);
    end

    function obj = setTrackingError(obj,Q,Qv,Qa)
      if ~isempty(Q)
        obj.Q = (Q+Q')/2;
      end
      if ~isempty(Qv)
        obj.Qv = (Qv+Qv')/2;
      end
      if ~isempty(Qa)
        obj.Qa = (Qa+Qa')/2;
      end
      obj.tracking_error = WeightedTrackingError(obj.t_knot, ...
        obj.Q,obj.q_nom,obj.Qv,zeros(size(obj.q_nom)),obj.Qa,obj.q_idx, ...
        obj.qd_idx);
      xind = [obj.q_idx(:);obj.qd_idx(:)];
      if isempty(obj.cost)
        obj = obj.addCost(obj.tracking_error,[],xind,xind);
      else
        obj = obj.replaceCost(obj.tracking_error,1,[],xind,xind);
      end
    end

    function [xtraj,F,info,infeasible_constraint] = solve(obj,qtraj_seed)
      % @param qtraj_seed.   A Trajectory object. The initial guess of posture trajectory.
      % @retval xtraj.       A Cubic spline trajectory. The solution of state trajectory,
      % x = [q;qdot]
      q_seed = qtraj_seed.eval(obj.t_knot);
      v_seed = qtraj_seed.fnder().eval(obj.t_knot);
      x0 = zeros(obj.num_vars,1);
      x0(obj.q_idx(:)) = q_seed(:);
      x0(obj.qd_idx) = v_seed;
      for i = 1:length(obj.qsc_weight_idx)
        if(~isempty(obj.qsc_weight_idx{i}))
          x0(obj.qsc_weight_idx{i}) = 1/length(obj.qsc_weight_idx{i});
        end
      end
      [x,F,info] = solve@NonlinearProgramWConstraintObjects(obj,x0);
      xtraj = PPTrajectory(foh(obj.t_knot, ...
        x([reshape(obj.q_idx(:),obj.nq,obj.nT); ...
           reshape(obj.qd_idx(:),obj.nq,obj.nT)])));
      xtraj = xtraj.setOutputFrame(obj.robot.getStateFrame);
      [info,infeasible_constraint] = infeasibleConstraintName(obj,x,info);
    end
  end
end
