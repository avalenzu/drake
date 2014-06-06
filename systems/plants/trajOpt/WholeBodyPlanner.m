classdef WholeBodyPlanner < NonlinearProgramWKinsol
  properties
    nv
    q_idx
    v_idx
    vd_idx
    qsc_weight_idx
    Q
    Qv
    Qa
    t_knot
    fix_initial_state
    dynamics_constraint;
    position_error;
    velocity_cost;
    acceleration_cost;
    q_nom_traj;
    v_nom_traj;
    vd_nom_traj;
  end
  methods
    function obj = WholeBodyPlanner(robot,t,q_nom_traj, ...
                                    fix_initial_state,x0,varargin)
      t = unique(t(:)');
      obj = obj@NonlinearProgramWKinsol(robot,numel(t));
      obj.t_knot = t;
      sizecheck(fix_initial_state,[1,1]);
      obj.nv = obj.robot.getNumDOF();
      obj.q_idx = reshape((1:obj.nq*obj.nT),obj.nq,obj.nT);

      v_name = cell(obj.nv,obj.nT-1);
      for j = 1:(obj.nT-1)
        for i = 1:obj.nq
          v_name{i,j} = sprintf('v%d[%d]',i,j);
        end
      end
      v_name = reshape(v_name,obj.nv*(obj.nT-1),1);
      obj.v_idx = reshape(obj.num_vars+(1:obj.nv*(obj.nT-1)), ...
                            obj.nv,obj.nT-1);
                  
      obj = obj.addDecisionVariable(obj.nv*(obj.nT-1),v_name);

      vd_name = cell(obj.nv,obj.nT-2);
      for j = 1:(obj.nT-2)
        for i = 1:obj.nq
          vd_name{i,j} = sprintf('v%d[%d]',i,j);
        end
      end
      vd_name = reshape(vd_name,obj.nv*(obj.nT-2),1);
      obj.vd_idx = reshape(obj.num_vars+(1:obj.nv*(obj.nT-2)), ...
                            obj.nv,obj.nT-2);
                  
      obj = obj.addDecisionVariable(obj.nv*(obj.nT-1),v_name);
      obj = obj.addDecisionVariable(obj.nv*(obj.nT-1),vd_name);
      obj = obj.setFixInitialState(fix_initial_state,x0);
      obj = obj.setLinearDynamics();
      obj.q_nom_traj = q_nom_traj;
      obj.v_nom_traj = ConstantTrajectory(zeros(obj.nv,1));
      obj.vd_nom_traj = ConstantTrajectory(zeros(obj.nv,1));

      %qd_name = cell(obj.nq,obj.nT-2);
      %for j = 1:(obj.nT-2)
        %for i = 1:obj.nq
          %qd_name{i,j} = sprintf('qd%d[%d]',i,j+1);
        %end
      %end
      %qd_name = reshape(qd_name,obj.nq*(obj.nT-2),1);
      %obj.qd_idx = [obj.qd0_idx, ...
                    %reshape(obj.num_vars+(1:obj.nq*(obj.nT-2)), ...
                            %obj.nq,obj.nT-2), ...
                    %obj.qdf_idx];
      %obj = obj.addDecisionVariable(obj.nq*(obj.nT-2),qd_name);
      %obj = obj.setLinearDynamics();
      obj.Q = eye(obj.nq);
      obj.Qv = eye(obj.nq);
      obj.Qa = 0*eye(obj.nq);
      obj = obj.setTrackingError(obj.Q,obj.Qv);
      obj.qsc_weight_idx = cell(1,obj.nT);
      num_rbcnstr = nargin-5;
      [q_lb,q_ub] = obj.robot.getJointLimits();
      obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint( ...
        reshape(bsxfun(@times,q_lb,ones(1,obj.nT)),[],1),...
        reshape(bsxfun(@times,q_ub,ones(1,obj.nT)),[],1)),obj.q_idx(:));
      if(obj.fix_initial_state)
        t_start = 2;
      else
        t_start = 1;
      end
      for i = 1:num_rbcnstr
        if(~isa(varargin{i},'RigidBodyConstraint'))
          error('Drake:WholeBodyPlanner:NonRBConstraint', ...
                'The input should be a RigidBodyConstraint');
        end
        if(isa(varargin{i},'SingleTimeKinematicConstraint'))
          for j = t_start:obj.nT
            if(varargin{i}.isTimeValid(obj.t_knot(j)))
              cnstr = varargin{i}.generateConstraint(obj.t_knot(j));
              obj = obj.addNonlinearConstraint(cnstr{1},j,[],obj.q_idx(:,j));
            end
          end
        elseif(isa(varargin{i},'PostureConstraint'))
          for j = t_start:obj.nT
            if(varargin{i}.isTimeValid(obj.t_knot(j)))
              cnstr = varargin{i}.generateConstraint(obj.t_knot(j));
              obj = obj.addBoundingBoxConstraint(cnstr{1},obj.q_idx(:,j));
            end
          end
        elseif(isa(varargin{i},'QuasiStaticConstraint'))
          for j = t_start:obj.nT
            if(varargin{i}.isTimeValid(obj.t_knot(j)) && varargin{i}.active)
              if(~isempty(obj.qsc_weight_idx{j}))
                error('Drake:InverseKinTraj:currently only support at most one QuasiStaticConstraint at an individual time');
              end
              cnstr = varargin{i}.generateConstraint(obj.t_knot(j));
              qsc_weight_names = cell(varargin{i}.num_pts,1);
              for k = 1:varargin{i}.num_pts
                qsc_weight_names{k} = sprintf('qsc_weight%d',k);
              end
              obj.qsc_weight_idx{j} = obj.num_vars+(1:varargin{i}.num_pts)';
              obj = obj.addDecisionVariable(varargin{i}.num_pts,qsc_weight_names);
              obj = obj.addNonlinearConstraint(cnstr{1},j,obj.qsc_weight_idx{j},[obj.q_idx(:,j);obj.qsc_weight_idx{j}]);
              obj = obj.addLinearConstraint(cnstr{2},obj.qsc_weight_idx{j});
              obj = obj.addBoundingBoxConstraint(cnstr{3},obj.qsc_weight_idx{j});
            end
          end
        elseif(isa(varargin{i},'SingleTimeLinearPostureConstraint'))
          for j = t_start:obj.nT
            if(varargin{i}.isTimeValid(obj.t_knot(j)))
              cnstr = varargin{i}.generateConstraint(obj.t_knot(j));
              obj = obj.addLinearConstraint(cnstr{1},obj.q_idx(:,j));
            end
          end
        elseif(isa(varargin{i},'MultipleTimeKinematicConstraint'))
          valid_t_flag = varargin{i}.isTimeValid(obj.t_knot(t_start:end));
          t_idx = (t_start:obj.nT);
          valid_t_idx = t_idx(valid_t_flag);
          cnstr = varargin{i}.generateConstraint(obj.t_knot(valid_t_idx));
          obj = obj.addNonlinearConstraint(cnstr{1},valid_t_idx,[],reshape(obj.q_idx(:,valid_t_idx),[],1));
        elseif(isa(varargin{i},'MultipleTimeLinearPostureConstraint'))
          cnstr = varargin{i}.generateConstraint(obj.t_knot(t_start:end));
          obj = obj.addLinearConstraint(cnstr{1},reshape(obj.q_idx(:,t_start:end),[],1));
        end
      end
      obj = obj.setSolverOptions('snopt','majoroptimalitytolerance',1e-6);
      obj = obj.setSolverOptions('snopt','superbasicslimit',2000);
      obj = obj.setSolverOptions('snopt','majorfeasibilitytolerance',1e-6);
      obj = obj.setSolverOptions('snopt','iterationslimit',10000);
      obj = obj.setSolverOptions('snopt','majoriterationslimit',200);
    end

    function obj = setLinearDynamics(obj)
      obj.dynamics_constraint{1} = ...
        LinearDynamics(obj.t_knot,obj.nq, obj.nv, 1:obj.nq*obj.nT, ...
                       obj.nq*obj.nT+(1:obj.nv*(obj.nT-1)));
      xind = [obj.q_idx(:);obj.v_idx(:)];
      obj = obj.addLinearConstraint(obj.dynamics_constraint{1},xind);
      t_mid = (obj.t_knot(1:end-1)+obj.t_knot(2:end))/2;
      obj.dynamics_constraint{2} = ...
        LinearDynamics(t_mid,obj.nv, obj.nv, 1:obj.nv*(obj.nT-1), ...
                       obj.nv*(obj.nT-1)+(1:obj.nv*(obj.nT-2)));
      xind = [obj.v_idx(:);obj.vd_idx(:)];
      obj = obj.addLinearConstraint(obj.dynamics_constraint{2},xind);
    end

    function obj = setTrackingError(obj,Q,Qv,Qa)
      if nargin > 1 && ~isempty(Q)
        obj.Q = (Q+Q')/2;
      end
      if nargin > 2 && ~isempty(Qv)
        obj.Qv = (Qv+Qv')/2;
      end
      if nargin > 3 && ~isempty(Qa)
        obj.Qa = (Qa+Qa')/2;
      end
      %q_nom_traj = PPTrajectory(foh(obj.t_knot,obj.q_nom));
      %v_nom_traj = PPTrajectory(foh(obj.t_knot,0*obj.q_nom));
      obj.position_error = WeightedSquaredError(obj.t_knot, obj.Q, ...
                                                obj.q_nom_traj);
      obj.velocity_cost = WeightedSquaredError(obj.t_knot(1:end-1),obj.Qv, ...
        obj.v_nom_traj);

      obj.acceleration_cost = WeightedSquaredError(obj.t_knot(1:end-2),obj.Qa, ...
        obj.v_nom_traj);
      if isempty(obj.cost)
        obj = obj.addCost(obj.position_error,[],obj.q_idx(:),obj.q_idx(:));
      else
        obj = obj.replaceCost(obj.position_error,1,[],obj.q_idx(:),obj.q_idx(:));
      end
      if numel(obj.cost) < 2
        obj = obj.addCost(obj.velocity_cost,[],obj.v_idx(:),obj.v_idx(:));
      else
        obj = obj.replaceCost(obj.velocity_cost,2,[],obj.v_idx(:),obj.v_idx(:));
      end
      if numel(obj.cost) < 3
        obj = obj.addCost(obj.acceleration_cost,[],obj.vd_idx(:),obj.vd_idx(:));
      else
        obj = obj.replaceCost(obj.acceleration_cost,3,[],obj.vd_idx(:),obj.vd_idx(:));
      end
    end

    function obj = setFixInitialState(obj,flag,x0)
      % set obj.fix_initial_state = flag. If flag = true, then fix the initial state to x0
      % @param x0   A 2*obj.nq x 1 double vector. x0 = [q0;qdot0]. The initial state
      sizecheck(flag,[1,1]);
      flag = logical(flag);
      if(isempty(obj.bbcon))
        obj.fix_initial_state = flag;
        if(obj.fix_initial_state)
          obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(x0(1:obj.nq),x0(1:obj.nq)),obj.q_idx(:,1));
        else
          obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(-inf(obj.nq,1),inf(obj.nq,1)),obj.q_idx(:,1));
        end
      elseif(obj.fix_initial_state ~= flag)
        obj.fix_initial_state = flag;
        if(obj.fix_initial_state)
          obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(x0(1:obj.nq),x0(1:obj.nq)),obj.q_idx(:,1));
        else
          obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(-inf(obj.nq,1),inf(obj.nq,1)),obj.q_idx(:,1));
        end
      end
    end

    function [xtraj,F,info,infeasible_constraint] = solve(obj,qtraj_seed)
      % @param qtraj_seed.   A Trajectory object. The initial guess of posture trajectory.
      % @retval xtraj.       A Cubic spline trajectory. The solution of state trajectory,
      % x = [q;qdot]
      q_seed = qtraj_seed.eval(obj.t_knot);
      x0 = zeros(obj.num_vars,1);
      x0(obj.q_idx(:)) = q_seed(:);
      for i = 1:length(obj.qsc_weight_idx)
        if(~isempty(obj.qsc_weight_idx{i}))
          x0(obj.qsc_weight_idx{i}) = 1/length(obj.qsc_weight_idx{i});
        end
      end
      [x,F,info] = solve@NonlinearProgramWConstraintObjects(obj,x0);
      q_sol = x(obj.q_idx);
      dt = reshape(diff(obj.t_knot),1,[]);
      v_tmp = bsxfun(@rdivide,diff(q_sol,1,2),dt);
      v_sol = (v_tmp(:,[1,1:end])+v_tmp(:,[1:end,end]))/2;
      xtraj = PPTrajectory(foh(obj.t_knot, ...
        [q_sol;v_sol]));
      xtraj = xtraj.setOutputFrame(obj.robot.getStateFrame);
      [info,infeasible_constraint] = infeasibleConstraintName(obj,x,info);
    end
  end
end
