classdef SimpleDynamicsFullKinematicsPlanner < DirectTrajectoryOptimization
  properties(SetAccess = protected)
    nq % number of positions in state
    nv % number of velocities in state
    q_inds % An nq x obj.N matrix. x(q_inds(:,i)) is the posture at i'th knot
    v_inds % An nv x obj.N matrix. x(v_inds(:,i)) is the velocity at i'th knot 
    qsc_weight_idx
    Q
    t_seed
    fix_initial_state = false
    g
    dynamics_constraint;
    position_error;
    q_nom_traj;
    dt_inds  % A 1 x obj.N-1 array. x(dt_inds(i)) is the decision variable dt(i) = t(i+1)-t(i)

    % N-element vector of indices into the shared_data, where
    % shared_data{kinsol_dataind(i)} is the kinsol for knot point i
    kinsol_dataind 

    contact_wrench_cnstr % A cell of ContactWrenchConstraint objects
    unique_contact_bodies; % An integer array. The indices of the unique contact bodies. It is in the order of obj.contact_wrench_cnstr. Namely if obj.contact_wrench_cnstr has body indices [3 2 4 3 1 2], theun unique_contact_bodies = [3 2 4 1]
    unique_body_contact_pts; % A length(unique_contact_bodies x 1) cell. unique_body_contact_pts{i} is a 3 x num_pts array, which contains all the contact points for unique_contact_bodies(i)
    lambda_inds % A length(unique_contact_bodies) x 1 cell. lambda_idx{i} is an N x size(unique_body_contact_pts{i},2) x N array. where N is the number of force parameters for one contact point
    lambda2contact_wrench_cnstr % A length(unique_contact_bodies) x 1 cell....
                 % lambda2contact_wrench_cnstr{i} is a size(unique_body_contact_pts{i},2) x obj.N matrix.
                 % lambda{i}(:,j,k) are the contact forces whose information are encoded
                 % in the ContactWrenchConstraint
                 % obj.contact_wrench_cnstr(obj.lambda2contact_wrench_cnstr{i}(j,k))
                 % If there is no active ContactWrenchConstraint, then the value of
                 % lambda2contact_wrench_cnstr is set to be 0.
                 % Furthermore, I guarantee that in obj.lambda2contact_wrench_cnstr{i},
                 % the same ContactWrenchConstraint indices are grouped together. Namely
                 % obj.lambda2contact_wrench_cnstr{2}(:,1) would be [1;1;1;2;2] rather than
                 % [1;2;1;1;2];
     num_lambda_knot % An integer. The number of external force parameters at any one knot point
     robot_mass  % A double scalar.
  end
  
  properties(Access = protected)
    add_dynamic_constraint_flag = false;% If this flag is false, then bypass the addDynamicConstraint function
  end
  
  methods
    function obj = SimpleDynamicsFullKinematicsPlanner(robot,t_seed,tf_range,q_nom_traj, ...
                                    fix_initial_state,x0,varargin)
      t_seed = unique(t_seed(:)');
      obj = obj@DirectTrajectoryOptimization(robot,numel(t_seed),tf_range);
      obj.nq = obj.plant.getNumPositions();
      obj.t_seed = t_seed;
      sizecheck(fix_initial_state,[1,1]);
      obj.nv = obj.plant.getNumDOF();
      obj.q_inds = obj.x_inds(1:obj.nq,:);
      obj.v_inds = obj.x_inds(obj.nq+(1:obj.nv),:);
      obj.dt_inds = obj.num_vars+(1:obj.N-1);
      x_name = cell(obj.N-1,1);
      for i = 1:obj.N-1
        x_name{i} = sprintf('dt[%d]',i);
      end
      obj = obj.addDecisionVariable(obj.N-1,x_name);
      obj = obj.setFixInitialState(fix_initial_state,x0);
      obj.q_nom_traj = q_nom_traj;
      obj.g = 9.81;
      % create shared data functions to calculate kinematics at the knot
      % points
      kinsol_dataind = zeros(obj.N,1);
      for i=1:obj.N,
        [obj,kinsol_dataind(i)] = obj.addSharedDataFunction(@obj.kinematicsData,{obj.q_inds(:,i)});
      end
      obj.kinsol_dataind = kinsol_dataind;

      obj.Q = eye(obj.nq);
      obj.qsc_weight_idx = cell(1,obj.N);
      num_rbcnstr = nargin-6;
      [q_lb,q_ub] = obj.plant.getJointLimits();
      obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint( ...
        reshape(bsxfun(@times,q_lb,ones(1,obj.N)),[],1),...
        reshape(bsxfun(@times,q_ub,ones(1,obj.N)),[],1)),obj.q_inds(:));
      if(obj.fix_initial_state)
        t_start = 2;
      else
        t_start = 1;
      end
      obj.contact_wrench_cnstr = {};
      for i = 1:num_rbcnstr
        if(~isa(varargin{i},'RigidBodyConstraint'))
          error('Drake:WholeBodyPlanner:NonRBConstraint', ...
                'The input should be a RigidBodyConstraint');
        end
        if(isa(varargin{i},'SingleTimeKinematicConstraint'))
          for j = t_start:obj.N
            if(varargin{i}.isTimeValid(obj.t_seed(j)))
              cnstr = varargin{i}.generateConstraint(obj.t_seed(j));
              obj = obj.addManagedKinematicConstraint(cnstr{1},j);
            end
          end
        elseif(isa(varargin{i},'PostureConstraint'))
          for j = t_start:obj.N
            if(varargin{i}.isTimeValid(obj.t_seed(j)))
              cnstr = varargin{i}.generateConstraint(obj.t_seed(j));
              obj = obj.addBoundingBoxConstraint(cnstr{1},obj.q_inds(:,j));
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
              obj = obj.addNonlinearConstraint(cnstr{1},{obj.q_inds(:,j);obj.qsc_weight_idx{j}},obj.kinsol_dataind(j));
              obj = obj.addLinearConstraint(cnstr{2},obj.qsc_weight_idx{j});
              obj = obj.addBoundingBoxConstraint(cnstr{3},obj.qsc_weight_idx{j});
            end
          end
        elseif(isa(varargin{i},'SingleTimeLinearPostureConstraint'))
          for j = t_start:obj.N
            if(varargin{i}.isTimeValid(obj.t_seed(j)))
              cnstr = varargin{i}.generateConstraint(obj.t_seed(j));
              obj = obj.addLinearConstraint(cnstr{1},obj.q_inds(:,j));
            end
          end
        elseif(isa(varargin{i},'MultipleTimeKinematicConstraint'))
          valid_t_flag = varargin{i}.isTimeValid(obj.t_seed(t_start:end));
          t_idx = (t_start:obj.N);
          valid_t_idx = t_idx(valid_t_flag);
          cnstr = varargin{i}.generateConstraint(obj.t_seed(valid_t_idx));
          obj = obj.addNonlinearConstraint(cnstr{1},valid_t_idx,[],reshape(obj.q_inds(:,valid_t_idx),[],1));
        elseif(isa(varargin{i},'MultipleTimeLinearPostureConstraint'))
          cnstr = varargin{i}.generateConstraint(obj.t_seed(t_start:end));
          obj = obj.addLinearConstraint(cnstr{1},reshape(obj.q_inds(:,t_start:end),[],1));
        elseif(isa(varargin{i},'ContactWrenchConstraint'))
          % I am supposing that a body only has one type of contact.
          obj.contact_wrench_cnstr{end+1} = varargin{i};
        end
      end
      obj.robot_mass = obj.plant.getMass();
      obj = obj.parseContactWrenchConstraint();
      obj = obj.addDynamicConstraints();
      
      obj = obj.setSolverOptions('snopt','majoroptimalitytolerance',1e-5);
      obj = obj.setSolverOptions('snopt','superbasicslimit',2000);
      obj = obj.setSolverOptions('snopt','majorfeasibilitytolerance',1e-6);
      obj = obj.setSolverOptions('snopt','iterationslimit',1e5);
      obj = obj.setSolverOptions('snopt','majoriterationslimit',200);
    end

    function data = kinematicsData(obj,q)
      data = doKinematics(obj.plant,q,false,false);
    end

    function obj = setFixInitialState(obj,flag,x0)
      % set obj.fix_initial_state = flag. If flag = true, then fix the initial state to x0
      % @param x0   A 2*obj.plant.getNumPositions() x 1 double vector. x0 = [q0;qdot0]. The initial state
      sizecheck(flag,[1,1]);
      flag = logical(flag);
      if(isempty(obj.bbcon))
        obj.fix_initial_state = flag;
        if(obj.fix_initial_state)
          obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(x0,x0),obj.x_inds(:,1));
        else
          obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(-inf(obj.plant.getNumStates(),1),inf(obj.plant.getNumStates(),1)),obj.x_inds(:,1));
        end
      elseif(obj.fix_initial_state ~= flag)
        obj.fix_initial_state = flag;
        if(obj.fix_initial_state)
          obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(x0,x0),obj.x_inds(:,1));
        else
          obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(-inf(obj.plant.getNumStates(),1),inf(obj.plant.getNumStates(),1)),obj.x_inds(:,1));
        end
      end
    end


    function obj = addManagedKinematicConstraint(obj,constraint,time_index)
      % Add a linear constraint that is a function of the state at the
      % specified time or times.
      % @param constraint  a ConstraintManager
      % @param time_index   a cell array of time indices
      %   ex1., time_index = {1, 2, 3} means the constraint is applied
      %   individually to knot points 1, 2, and 3
      %   ex2,. time_index = {[1 2], [3 4]} means the constraint is applied to knot
      %   points 1 and 2 together (taking the combined state as an argument)
      %   and 3 and 4 together.
      if ~isa(constraint,'ConstraintManager')
        constraint = ConstraintManager([],constraint);
      end
      if ~iscell(time_index)
        time_index = {time_index};
      end
      for j=1:length(time_index),
        kinsol_inds = obj.kinsol_dataind(time_index{j}); 
        cstr_inds = mat2cell(obj.q_inds(:,time_index{j}),size(obj.q_inds,1),ones(1,length(time_index{j})));
        
        % record constraint for posterity
        obj.constraints{end+1}.constraint = constraint;
        obj.constraints{end}.var_inds = cnstr_inds;
        obj.constraints{end}.kinsol_inds = kinsol_inds;
        obj.constraints{end}.time_index = time_index;
        
        obj = obj.addManagedConstraints(constraint,cnstr_inds,kinsol_inds);
      end
    end
    
    function obj = parseContactWrenchConstraint(obj)
      num_contact_wrench_cnstr = length(obj.contact_wrench_cnstr);
      obj.unique_contact_bodies = [];
      pt_num_F_contact_bodies = []; % pt_num_F_contact_bodies(i) is the field pt_num_F of the ContactWrenchConstraint for body unique_contact_bodies(i)
      obj.unique_body_contact_pts = {};
      body_contact_type = []; % body_contact_type(i) is the type of ContactWrenchConstraint for body unique_contact_bodies(i)
      
      unique_body2contact_wrench_cnstr = {}; % unique_body2contact_wrench_cnstr{i} contains all the indices of the ContactWrenchConstraint that impose constraint on unique_contact_bodies(i)
      for i = 1:num_contact_wrench_cnstr
        repeat_body = obj.contact_wrench_cnstr{i}.body == obj.unique_contact_bodies;
        if(~any(repeat_body))
          obj.unique_contact_bodies(end+1) =  obj.contact_wrench_cnstr{i}.body;
          pt_num_F_contact_bodies(end+1) = obj.contact_wrench_cnstr{i}.pt_num_F;
          obj.unique_body_contact_pts{end+1} = obj.contact_wrench_cnstr{i}.body_pts;
          body_contact_type(end+1) = obj.contact_wrench_cnstr{i}.type;
          unique_body2contact_wrench_cnstr{end+1} = i;
        else
          if(sum(repeat_body)>1)
            error('Drake:SimpleDynamicsFullKinematics:parseContactWrenchConstraint: there should be at most one existing contact body with the same body index');
          end
          if(obj.contact_wrench_cnstr{i}.type ~= body_contact_type(repeat_body))
            error('Drake:SimpleDynamicsFullKinematics:parseContactWrenchConstraint: Currently we only support one type of ContactWrenchConstraint for one contact body');
          end
          if(obj.contact_wrench_cnstr{i}.pt_num_F ~= pt_num_F_contact_bodies(repeat_body))
            error('Drake:SimpleDynamicsFullKinematics:parseContactWrenchConstraint: The number of force parameters on the same body should be the same');
          end
          obj.unique_body_contact_pts{repeat_body} = unique([obj.unique_body_contact_pts{repeat_body} obj.contact_wrench_cnstr{i}.body_pts]','rows','stable')';
          unique_body2contact_wrench_cnstr{repeat_body}(end+1) = i;
        end
      end
      % set up lambda2contact_wrench_cnstr
      obj.lambda2contact_wrench_cnstr = cell(length(obj.unique_contact_bodies),1);
      for i = 1:length(obj.unique_contact_bodies)
        obj.lambda2contact_wrench_cnstr{i} = zeros(size(obj.unique_body_contact_pts{i},2),obj.N);
        for cwc_idx = unique_body2contact_wrench_cnstr{i}
          for t_idx = 1:obj.N
            if(obj.contact_wrench_cnstr{cwc_idx}.isTimeValid(obj.t_seed(t_idx)))
              [~,pt_idx] = intersect(obj.unique_body_contact_pts{i}',obj.contact_wrench_cnstr{cwc_idx}.body_pts','rows','stable'); 
              if(any(obj.lambda2contact_wrench_cnstr{i}(pt_idx,t_idx) ~= 0))
                pt_coord = obj.unique_body_contact_pts{i}(:,pt_idx(1));
                error('Drake:SimpleDynamicsFullKinematics:parseContactWrenchConstraint: at knot %d, %s pt [%5.2f %5.2f %5.2f] is active for more than 1 ContactWrenchConstraint, one of which is obj.contact_wrench_cnstr{%d}',...
                  t_idx,obj.contact_wrench_cnstr{cwc_idx}.body_name,pt_coord(1),pt_coord(2),pt_coord(3),cwc_idx);
              end
              obj.lambda2contact_wrench_cnstr{i}(pt_idx,t_idx) = cwc_idx;
            end
          end
        end
      end
      obj.num_lambda_knot = 0;
      for i = 1:length(obj.unique_contact_bodies)
        num_lambda_i = pt_num_F_contact_bodies(i)*size(obj.unique_body_contact_pts{i},2)*obj.N;
        obj.num_lambda_knot = obj.num_lambda_knot+num_lambda_i;
        obj.lambda_inds{i} = obj.num_vars+reshape(1:num_lambda_i,pt_num_F_contact_bodies(i),size(obj.unique_body_contact_pts{i},2),obj.N);
        x_name = cell(num_lambda_i,1);
        for j = 1:obj.N
          for k = 1:size(obj.unique_body_contact_pts{i},2)
            contact_wrench_cnstr_idx = obj.lambda2contact_wrench_cnstr{i}(k,j);
            if(contact_wrench_cnstr_idx>0)
              body_name = obj.contact_wrench_cnstr{contact_wrench_cnstr_idx}.body_name;
              for l = 1:pt_num_F_contact_bodies(i)
                x_name{(j-1)*(size(obj.unique_body_contact_pts{i},2)*pt_num_F_contact_bodies(i))+(k-1)*pt_num_F_contact_bodies(i)+l}...
                  = sprintf('%s_pt%d_F[%d]',body_name,k,j);
              end
            else
              x_name((j-1)*(size(obj.unique_body_contact_pts{i},2)*pt_num_F_contact_bodies(i))+(k-1)*pt_num_F_contact_bodies(i)+(1:pt_num_F_contact_bodies(i)))...
                = repmat({'null contact force'},pt_num_F_contact_bodies(i),1);
            end
          end
        end
        obj = obj.addDecisionVariable(num_lambda_i,x_name);
        % Now add the constraint from ContactWrenchConstraint
        for k = 1:obj.N
          contact_wrench_idx = obj.lambda2contact_wrench_cnstr{i}(:,k);
          unique_contact_wrench_idx = unique(contact_wrench_idx,'stable');
          for j = reshape(unique_contact_wrench_idx,1,[])
            lambda_idx_ijk = obj.lambda_inds{i}(:,contact_wrench_idx==j,k); % lambda_idx_jk is the indices of lambda used for i'th body, with j'th ContactWrenchConstraint, at k'th knot point
            if(j > 0)
              cnstr_tmp = obj.contact_wrench_cnstr{j}.generateConstraint(obj.t_seed(k));
              cnstr_name = repmat({sprintf('contact_wrench_cnstr%d_nlcon[%d]',j,k)},cnstr_tmp{1}.num_cnstr,1);
              cnstr_tmp{1} = cnstr_tmp{1}.setName(cnstr_name);
              obj = obj.addNonlinearConstraint(cnstr_tmp{1},[{obj.q_inds(:,k)};{reshape(lambda_idx_ijk,[],1)}],obj.kinsol_dataind(k));
              obj = obj.addBoundingBoxConstraint(cnstr_tmp{2},reshape(lambda_idx_ijk,[],1));
            else
              obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(zeros(numel(lambda_idx_ijk),1),zeros(numel(lambda_idx_ijk),1)),lambda_idx_ijk);
            end
          end
        end
      end
    end
    
    function obj = addDynamicConstraints(obj)
      % First I find out the order of the contact_wrench_cnstr such that it is in the same
      % order of lambda
      if(obj.add_dynamic_constraint_flag)
        tLeft_contact_wrench_cnstr_idx = [];
        tLeft_lambda_idx = zeros(obj.num_lambda_knot,1);
        tLeft_lambda_count = 0;
        for j = 1:length(obj.unique_contact_bodies)
          tLeft_valid_pt_idx = obj.lambda2contact_wrench_cnstr{j}(:,1) ~= 0;
          tLeft_lambda_idx_j_count = sum(tLeft_valid_pt_idx)*size(obj.lambda_inds{j},1);
          tLeft_lambda_idx(tLeft_lambda_count+(1:tLeft_lambda_idx_j_count))...
            = reshape(obj.lambda_inds{j}(:,tLeft_valid_pt_idx,1),[],1);
          tLeft_lambda_count = tLeft_lambda_count+tLeft_lambda_idx_j_count;
          tLeft_contact_wrench_cnstr_idx = [tLeft_contact_wrench_cnstr_idx unique(obj.lambda2contact_wrench_cnstr{j}(tLeft_valid_pt_idx,1)','stable')];
        end
        tLeft_lambda_idx = tLeft_lambda_idx(1:tLeft_lambda_count);
        obj = obj.addContactDynamicConstraints(1,tLeft_contact_wrench_cnstr_idx,tLeft_lambda_idx);
        for i = 2:obj.N
          tRight_contact_wrench_cnstr_idx = [];
          tRight_lambda_idx = zeros(obj.num_lambda_knot,1);
          tRight_lambda_count = 0;
          for j = 1:length(obj.unique_contact_bodies)
            tRight_valid_pt_idx = obj.lambda2contact_wrench_cnstr{j}(:,i) ~= 0;
            tRight_lambda_idx_j_count = sum(tRight_valid_pt_idx)*size(obj.lambda_inds{j},1);
            tRight_lambda_idx(tRight_lambda_count+(1:tRight_lambda_idx_j_count))...
              =reshape(obj.lambda_inds{j}(:,tRight_valid_pt_idx,i),[],1);
            tRight_lambda_count = tRight_lambda_count+tRight_lambda_idx_j_count;
            tRight_contact_wrench_cnstr_idx = [tRight_contact_wrench_cnstr_idx unique(obj.lambda2contact_wrench_cnstr{j}(tRight_valid_pt_idx,i)','stable')];
          end
          tRight_lambda_idx = tRight_lambda_idx(1:tRight_lambda_count);
          obj = obj.addContactDynamicConstraints(i,tRight_contact_wrench_cnstr_idx,tRight_lambda_idx);
          obj = obj.addContactDynamicConstraints([i-1,i],[{tLeft_contact_wrench_cnstr_idx},{tRight_contact_wrench_cnstr_idx}],[{tLeft_lambda_idx},{tRight_lambda_idx}]);
          tLeft_contact_wrench_cnstr_idx = tRight_contact_wrench_cnstr_idx;
          tLeft_lambda_idx = tRight_lambda_idx;
          tLeft_lambda_count = tRight_lambda_count;
        end
      end
    end
  end
  
  methods(Abstract)
    obj = addContactDynamicConstraints(obj,num_knot,contact_wrench_cnstr_idx, knot_lambda_idx)
  end
end
