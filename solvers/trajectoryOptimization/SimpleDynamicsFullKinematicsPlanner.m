classdef SimpleDynamicsFullKinematicsPlanner < DirectTrajectoryOptimization
  properties
    nv % number of velocities in state
    q_idx % An nq x obj.N matrix. x(q_idx(:,i)) is the posture at i'th knot
    v_idx % An nv x obj.N matrix. x(v_idx(:,i)) is the velocity at i'th knot 
    qsc_weight_idx
    Q
    t_seed
    fix_initial_state
    dynamics_constraint;
    position_error;
    q_nom_traj;
    v_nom_traj;
    vd_nom_traj;
    dt_idx  % A 1 x obj.N-1 array. x(dt_idx(i)) is the decision variable dt(i) = t(i+1)-t(i)
    unique_contact_bodies % A set of body indices, whose body has a ContactWrenchConstraint
    num_unique_contact_bodies % An integer. num_unique_contact_bodies = length(unique_contact_bodies)
    F_idx % A cell array of size 1 x num_unique_contact_bodies, F_idx{i} is a F_size(1) x F_size(2) x obj.N matrix
  end
  methods
    function obj = SimpleDynamicsFullKinematicsPlanner(robot,t_seed,q_nom_traj, ...
                                    fix_initial_state,x0,varargin)
      t_seed = unique(t_seed(:)');
      obj = obj@DirectTrajectoryOptimization(robot,numel(t_seed),[t_seed(1),t_seed(end)]);
      obj.t_seed = t_seed;
      sizecheck(fix_initial_state,[1,1]);
      obj.nv = obj.plant.getNumDOF();
      obj.q_idx = reshape((1:obj.plant.getNumPositions()*obj.N),obj.plant.getNumPositions(),obj.N);
      obj.dt_idx = obj.num_vars+(1:obj.N-1);
      x_name = cell(obj.N-1,1);
      for i = 1:obj.N-1
        x_name{i} = sprintf('dt[%d]',i);
      end
      obj = obj.addDecisionVariable(obj.N-1,x_name);
      %v_name = cell(obj.nv,obj.nT_v);
      %for j = 1:obj.nT_v
        %for i = 1:obj.nv
          %v_name{i,j} = sprintf('v%d[%d]',i,j);
        %end
      %end
      %v_name = reshape(v_name,obj.nv*obj.nT_v,1);
      %obj.v_idx = reshape(obj.num_vars+(1:obj.nv*obj.nT_v), ...
                            %obj.nv,obj.nT_v);
      %obj = obj.addDecisionVariable(obj.nv*obj.nT_v,v_name);

      %vd_name = cell(obj.nv,obj.nT_vd);
      %for j = 1:obj.nT_vd
        %for i = 1:obj.nv
          %vd_name{i,j} = sprintf('v%d[%d]',i,j);
        %end
      %end
      %vd_name = reshape(vd_name,obj.nv*obj.nT_vd,1);
      %obj.vd_idx = reshape(obj.num_vars+(1:obj.nv*obj.nT_vd), ...
                            %obj.nv,obj.nT_vd);
      %obj = obj.addDecisionVariable(obj.nv*obj.nT_vd,vd_name);

      obj = obj.setFixInitialState(fix_initial_state,x0);
      obj.q_nom_traj = q_nom_traj;
      obj.v_nom_traj = ConstantTrajectory(zeros(obj.nv,1));
      obj.vd_nom_traj = ConstantTrajectory(zeros(obj.nv,1));

      obj.Q = eye(obj.plant.getNumPositions());
      obj.qsc_weight_idx = cell(1,obj.N);
      num_rbcnstr = nargin-6;
      [q_lb,q_ub] = obj.plant.getJointLimits();
      obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint( ...
        reshape(bsxfun(@times,q_lb,ones(1,obj.N)),[],1),...
        reshape(bsxfun(@times,q_ub,ones(1,obj.N)),[],1)),obj.q_idx(:));
      if(obj.fix_initial_state)
        t_start = 2;
      else
        t_start = 1;
      end
      obj.unique_contact_bodies = [];
      obj.num_unique_contact_bodies = 0;
      for i = 1:num_rbcnstr
        if(~isa(varargin{i},'RigidBodyConstraint'))
          error('Drake:WholeBodyPlanner:NonRBConstraint', ...
                'The input should be a RigidBodyConstraint');
        end
        if(isa(varargin{i},'SingleTimeKinematicConstraint'))
          for j = t_start:obj.N
            if(varargin{i}.isTimeValid(obj.t_seed(j)))
              cnstr = varargin{i}.generateConstraint(obj.t_seed(j));
              obj = obj.addNonlinearConstraint(cnstr{1},j,[],obj.q_idx(:,j));
            end
          end
        elseif(isa(varargin{i},'PostureConstraint'))
          for j = t_start:obj.N
            if(varargin{i}.isTimeValid(obj.t_seed(j)))
              cnstr = varargin{i}.generateConstraint(obj.t_seed(j));
              obj = obj.addBoundingBoxConstraint(cnstr{1},obj.q_idx(:,j));
            end
          end
        elseif(isa(varargin{i},'QuasiStaticConstraint'))
          for j = t_start:obj.N
            if(varargin{i}.isTimeValid(obj.t_seed(j)) && varargin{i}.active)
              if(~isempty(obj.qsc_weight_idx{j}))
                error('Drake:InverseKinTraj:currently only support at most one QuasiStaticConstraint at an individual time');
              end
              cnstr = varargin{i}.generateConstraint(obj.t_seed(j));
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
          for j = t_start:obj.N
            if(varargin{i}.isTimeValid(obj.t_seed(j)))
              cnstr = varargin{i}.generateConstraint(obj.t_seed(j));
              obj = obj.addLinearConstraint(cnstr{1},obj.q_idx(:,j));
            end
          end
        elseif(isa(varargin{i},'MultipleTimeKinematicConstraint'))
          valid_t_flag = varargin{i}.isTimeValid(obj.t_seed(t_start:end));
          t_idx = (t_start:obj.N);
          valid_t_idx = t_idx(valid_t_flag);
          cnstr = varargin{i}.generateConstraint(obj.t_seed(valid_t_idx));
          obj = obj.addNonlinearConstraint(cnstr{1},valid_t_idx,[],reshape(obj.q_idx(:,valid_t_idx),[],1));
        elseif(isa(varargin{i},'MultipleTimeLinearPostureConstraint'))
          cnstr = varargin{i}.generateConstraint(obj.t_seed(t_start:end));
          obj = obj.addLinearConstraint(cnstr{1},reshape(obj.q_idx(:,t_start:end),[],1));
        elseif(isa(varargin{i},'ContactWrenchConstraint'))
          % I am supposing that a body only has one type of contact.
          if(~any(varargin{i}.body == obj.unique_contact_bodies))
            obj.unique_contact_bodies = [obj.unique_contact_bodies varargin{i}.body];
            obj.num_unique_contact_bodies = obj.num_unique_contact_bodies+1;
            obj.F_idx = [obj.F_idx,{zeros(varargin{i}.F_size(1),varargin{i}.F_size(2),obj.N)}];
            num_F = prod(varargin{i}.F_size);
            for j = 1:obj.N
              x_name = repmat({sprintf('%s_contact_F[%d]',varargin{i}.body_name,j)},num_F,1);
              obj.F_idx{end}(:,:,j) = obj.num_vars+reshape(num_F,varargin{i}.F_size(1),varargin{i}.F_size(2));
              obj = obj.addDecisionVariables(num_F,x_name);
              if(varargin{i}.isTimeValid(obj.t_seed(j)))
                cnstr = varargin{i}.generateConstraint(obj.t_seed(j));
                obj = obj.addNonlinearConstraint(cnstr{1},[obj.q_idx(:,j);reshape(obj.F_idx{end}(:,:,j),[],1)]);
                obj = obj.addBoundingBoxConstraint(cnstr{2},reshape(obj.F_idx{end}(:,:,j),[],1));
              else
                obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(zeros(num_F,1),zeros(num_F,1)),reshape(obj.F_idx{end}(:,:,j),[],1));
              end
            end
          end
        end
      end
      obj = obj.setSolverOptions('snopt','majoroptimalitytolerance',1e-4);
      obj = obj.setSolverOptions('snopt','superbasicslimit',2000);
      obj = obj.setSolverOptions('snopt','majorfeasibilitytolerance',1e-6);
      obj = obj.setSolverOptions('snopt','iterationslimit',1e5);
      obj = obj.setSolverOptions('snopt','majoriterationslimit',300);
    end

    function obj = setFixInitialState(obj,flag,x0)
      % set obj.fix_initial_state = flag. If flag = true, then fix the initial state to x0
      % @param x0   A 2*obj.plant.getNumPositions() x 1 double vector. x0 = [q0;qdot0]. The initial state
      sizecheck(flag,[1,1]);
      flag = logical(flag);
      if(isempty(obj.bbcon))
        obj.fix_initial_state = flag;
        if(obj.fix_initial_state)
          obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(x0(1:obj.plant.getNumPositions()),x0(1:obj.plant.getNumPositions())),obj.q_idx(:,1));
        else
          obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(-inf(obj.plant.getNumPositions(),1),inf(obj.plant.getNumPositions(),1)),obj.q_idx(:,1));
        end
      elseif(obj.fix_initial_state ~= flag)
        obj.fix_initial_state = flag;
        if(obj.fix_initial_state)
          obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(x0(1:obj.plant.getNumPositions()),x0(1:obj.plant.getNumPositions())),obj.q_idx(:,1));
        else
          obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(-inf(obj.plant.getNumPositions(),1),inf(obj.plant.getNumPositions(),1)),obj.q_idx(:,1));
        end
      end
    end
  end
end
