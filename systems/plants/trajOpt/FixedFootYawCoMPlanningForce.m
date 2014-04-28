classdef FixedFootYawCoMPlanningForce
  % 'F-step' of the alternative planning for CoM trajectory. Fix the contact location and
  % CoM trajectory, bust search for the contact force. This is a SOCP problem
  properties(SetAccess = protected)
    robot_mass  % The mass of the robot
    t_knot  % The time knot for planning
    g % gravitational acceleration
    nT % The length of obj.t_knot
    F_idx % A cell array. x(F_idx{i}{j}(:,k)) is the contact force parameter (the weights for the friction cone extreme rays) at time t_knot(i), for the j'th FootStepRegionContactConstraint, at k'th contact point
    H_idx; %A 3 x obj.nT matrix. x(H_idx(:,i)) is the centroidal angular momentum at time t_knot(i)
    Hdot_idx % A 3 x obj.nT matrix. x(Hdot_idx(:,i)) is the rate of centroidal angular momentum at time t_knot(i)
    margin_idx % A 1 x obj.nT vector. x(margin_idx(i)) is the force margin at time t_knot(i). We want to maximize this margin for better robustness
    epsilon_idx % A 3 x obj.nT matrix. x(epsilon_idx(:,i)) is the residue of the PD law Hdot[i] = lambda*H[i]+epsilon[i]
    tau_idx % An integer scalar. x(tau_idx) is the index of tau, which is the dummy variable used to bound the PD residue: sum_n epsilon[n]'*epsilon[n] <= tau  
    sdotsquare_idx  % A 1 x obj.nT positve vector. x(sdotsquare_idx(i)) is the square of the time scaling function s at the i'th knot point.
    num_vars % The total number of decision variables in the SOCP 
    x_names % A obj.num_vars x 1 double vector. The lower bound for the decision variable x
    x_lb % A obj.num_vars x 1 double vector. The lower bound for the decision variable x
    x_ub % A obj.num_vars x 1 double vector. The upper bound for the decision variable x
    Q_cost % A obj.num_vars x obj.num_vars sparse PSD matrix. The Hessian of the quadratic cost
    f_cost % A obj.num_vars x 1 vector. The linear componenet of the cost
    A_H,A_H_bnd % A_H * x = A_H_bnd is the constraint on the euler integraton of angular momentum
    A_angular_PD,A_angular_PD_bnd % A_angular_PD * x = A_angular_PD_bnd is the constraint Hdot[n] = lambda*H[n]+epsilon[n]
    num_fsrc_cnstr  % A scalar. The total number of FootStepRegionContactConstraint
    fsrc_cnstr % A cell array. All the FootStepRegionContactConstraint object
    
    F2fsrc_map % A cell array. obj.fsrc_cnstr{F2fsrc_map{i}(j)} is the FootStepContactRegionConstraint corresponds to the force x(obj.F_idx{i}{j})
    yaw % A 1 x num_fsrc_cnstr double vector. yaw(i) is the yaw angle for obj.fsrc_cnstr{i}
    
    A_newton % A 3*obj.nT x obj.num_vars matrix. force(:) = A_newton*x, where force(:,i) is the total force at i'th knot point
    A_margin % A obj.nT x obj.num_vars matrix. 
    A_margin_bnd % A obj.nT x 1 vector. A_margin*x<=A_margin_bnd represents margin(i)<= min(F_i), where F_i are all the force weights at the k'th knot point
  end
  
  properties(Access = protected)
    A_force % A cell array . A_force{i} = obj.fsrc_cnstr{i}.force, which is a 3 x obj.fsrc_cnstr[i}.num_edges matrix
    A_xy,b_xy,rotmat % A_xy is 3 x 2 x obj.num_fsrc_cnstr matrix. b_xy is 3 x 1 x obj.num_fsrc_cnstr matrix. rotmat is 3 x 3 x obj.num_fsrc_cnstr matrix. [rotmat(:,:,i),A_xy(:,:,i),b_xy(:,:,i)] = obj.fsrc_cnstr{i}.bodyTransform(obj.yaw(i)); 
    num_force_weight % A scalar. The total number of force weights.
  end
  
  methods
    function obj = FixedFootYawCoMPlanningForce(robot_mass,t,lambda,c_margin,varargin)
      % obj =
      % FixedFootYawCoMPlanningForce(robot_mass,t,lambda,c_margin,foot_step_region_contact_cnstr1,yaw1,foot_step_region_contact_cnstr2,yaw2,...)
      % @param robot_mass  The mass of the robot
      % @param t   The time knot for planning. This indicates which
      % FootStepRegionContactConstraint is active at a given time knot. The actual time is
      % determined by the scaling function.
      % @param lambda        A 3 x 3 Hurwitz matrix. It tries to drive the angular
      % momentum stays at 0 by putting the constraint Hdot[n] = lambda*H[n]+epsilon[n]
      % @param c_margin  A positive scalar. The weight of the force margin in the
      % objective. The objective function is sum_n lambda[n]'*lambda[n]-c_weight*margin[n]
      % @param foot_step_region_contact_cnstr    A FootStepRegionContactConstraint
      % @param yaw           A double scalar. The yaw angle of the foot
      if(~isnumeric(robot_mass))
        error('Drake:FixedFootYawCoMPlanningForce:robot mass should be numeric');
      end
      sizecheck(robot_mass,[1,1]);
      if(robot_mass<=0)
        error('Drake:FixedFootYawCoMPlanningForce:robot mass should be positive');
      end
      obj.robot_mass = robot_mass;
      if(~isnumeric(t))
        error('Drake:FixedFootYawCoMPlanningForce:t should be numeric');
      end
      obj.t_knot = reshape(unique(t),1,[]);
      obj.nT = length(obj.t_knot);
      obj.g = 9.81;
      if(~isnumeric(lambda))
        error('Drake:FixedFootYawCoMPlanningForce:lambda should be numeric');
      end
      sizecheck(lambda,[3,3]);
      if(any(eig(lambda)>=0))
        error('Drake:FixedFootYawCoMPlanningForce:lambda should be a Hurwitz matrix. Namely all its eigen values should be negative');
      end
      if(~isnumeric(c_margin))
        error('Drake:FixedFootYawCoMPlanningForce:c_margin should be numeric');
      end
      sizecheck(c_margin,[1,1]);
      if(c_margin<0)
        error('Drake:FixedFootYawCoMPlanningForce:c_margin should be non-negative');
      end
      obj.num_vars = 0;
      obj.H_idx = reshape(obj.num_vars+(1:3*obj.nT),3,obj.nT);
      obj.x_names = cell(obj.num_vars,1);
      for i = 1:obj.nT
        obj.x_names(obj.num_vars+(i-1)*3+(1:3)) = {sprintf('H_x[%d]',i);sprintf('H_y[%d]',i);sprintf('H_z[%d]',i)};
      end
      obj.num_vars = obj.num_vars+3*obj.nT;
      obj.Hdot_idx = reshape(obj.num_vars+(1:3*obj.nT),3,obj.nT);
      obj.x_names = [obj.x_names;cell(3*obj.nT,1)];
      for i = 1:obj.nT
        obj.x_names(obj.num_vars+(i-1)*3+(1:3)) = {sprintf('Hdot_x[%d]',i);sprintf('Hdot_y[%d]',i);sprintf('Hdot_z[%d]',i)};
      end
      obj.num_vars = obj.num_vars+3*obj.nT;
      obj.epsilon_idx = reshape(obj.num_vars+(1:3*obj.nT),3,obj.nT);
      obj.x_names = [obj.x_names;cell(3*obj.nT,1)];
      for i = 1:obj.nT
        obj.x_names(obj.num_vars+(i-1)*3+(1:3)) = repmat({sprintf('epsilon[%d]',i)},3,1);
      end
      obj.num_vars = obj.num_vars+3*obj.nT;
      obj.tau_idx = obj.num_vars+1;
      obj.x_names = [obj.x_names;{'tau'}];
      obj.num_vars = obj.num_vars+1;
      obj.sdotsquare_idx = obj.num_vars+(1:obj.nT);
      obj.x_names = [obj.x_names;cell(obj.nT,1)];
      for i = 1:obj.nT
        obj.x_names{obj.num_vars+i} = sprintf('sdotsquare[%d]',i);
      end
      obj.num_vars = obj.num_vars+obj.nT;
      obj.margin_idx = obj.num_vars+(1:obj.nT);
      obj.x_names = [obj.x_names;cell(obj.nT,1)];
      for i = 1:obj.nT
        obj.x_names{obj.num_vars+i} = sprintf('force_margin[%d]',i);
      end
      obj.num_vars = obj.num_vars+obj.nT;
      
      obj.num_fsrc_cnstr = length(varargin)/2;
      obj.fsrc_cnstr = cell(1,obj.num_fsrc_cnstr);
      obj.F2fsrc_map = cell(1,obj.nT);
      obj.yaw = zeros(1,obj.num_fsrc_cnstr);
      obj.F_idx = cell(1,obj.nT);
      obj.A_xy = zeros(3,2,obj.num_fsrc_cnstr);
      obj.b_xy = zeros(3,1,obj.num_fsrc_cnstr);
      obj.rotmat = zeros(3,3,obj.num_fsrc_cnstr);
      obj.num_force_weight = 0;
      for i = 1:obj.num_fsrc_cnstr
        if(~isa(varargin{2*i-1},'FootStepRegionContactConstraint'))
          error('Drake:FixedFootYawCoMPlanningForce:The input should be a FootStepRegionContactConstraint');
        end
        if(~isnumeric(varargin{2*i}))
          error('Drake:FixedFootYawCoMPlanningForce:The input yaw angle should be a double');
        end
        obj.fsrc_cnstr{i} = varargin{2*i-1};
        sizecheck(varargin{2*i},[1,1]);
        obj.yaw(i) = varargin{2*i};
        obj.A_force{i} = varargin{2*i-1}.force(varargin{2*i});
        [obj.rotmat(:,:,i),obj.A_xy(:,:,i),obj.b_xy(:,:,i)] = varargin{2*i-1}.foot_step_region_cnstr.bodyTransform(varargin{2*i});
        is_fsrc_active = false;
        for j = 1:obj.nT
          if(varargin{2*i-1}.foot_step_region_cnstr.isTimeValid(obj.t_knot(j)))
            obj.F_idx{j} = [obj.F_idx{j} {obj.num_vars+reshape((1:varargin{2*i-1}.num_force_weight),varargin{2*i-1}.num_edges,varargin{2*i-1}.num_contact_pts)}];
            obj.F2fsrc_map{j} = [obj.F2fsrc_map{j} i];
            is_fsrc_active = true;
            obj.num_vars = obj.num_vars+varargin{2*i-1}.num_force_weight;
            obj.num_force_weight = obj.num_force_weight+obj.fsrc_cnstr{i}.num_force_weight;
          end
        end
        if(~is_fsrc_active)
          error('Drake:FixedFootYawCoMPlanningForce:The %dth FootStepRegionContactConstraint is not active for any t_knot');
        end
      end
      obj.x_lb = -inf(obj.num_vars,1);
      obj.x_ub = inf(obj.num_vars,1);
      for i = 1:obj.nT
        for j = 1:length(obj.F_idx{i})
          obj.x_lb(obj.F_idx{i}{j}(:)) = 0;
        end
      end
      delta_s = 1/(obj.nT-1);
      % linear constraint that H[n]-H[n-1] = Hdot[n]*delta_s
      iA_H = [(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';];
      jA_H = [reshape(obj.H_idx(:,2:end),[],1);reshape(obj.H_idx(:,1:end-1),[],1); reshape(obj.Hdot_idx(:,2:end),[],1)];
      Aval_H = [ones(3*(obj.nT-1),1);-ones(3*(obj.nT-1),1);-reshape(bsxfun(@times,ones(3,1),delta_s*ones(1,obj.nT-1)),[],1)];
      obj.A_H = sparse(iA_H,jA_H,Aval_H,3*(obj.nT-1),obj.num_vars);
      obj.A_H_bnd = zeros(3*(obj.nT-1),1);
      % linear constraint that Hdot[n] = lambda*H[n]+epsilon[n]
      iA_angular_PD = [(1:3*obj.nT)';reshape(repmat(reshape((1:3*obj.nT),3,obj.nT),3,1),[],1);(1:3*obj.nT)'];
      jA_angular_PD = [obj.Hdot_idx(:);reshape(bsxfun(@times,obj.H_idx(:)',ones(3,1)),[],1);obj.epsilon_idx(:)];
      Aval_angular_PD = [ones(3*obj.nT,1);reshape(repmat(-lambda,1,obj.nT),[],1);-ones(3*obj.nT,1)];
      obj.A_angular_PD = sparse(iA_angular_PD,jA_angular_PD,Aval_angular_PD,3*obj.nT,obj.num_vars);
      obj.A_angular_PD_bnd = zeros(3*obj.nT,1);
      
      % compute the summation of force, and the constraint that force_margin<= min_i
      % (F_i), where F_i are all force weights at i'th knot
      obj.A_newton = zeros(3*obj.nT,obj.num_vars);
      obj.A_margin = zeros(obj.num_force_weight,obj.num_vars);
      obj.A_margin_bnd = zeros(obj.num_force_weight,1);
      force_weight_count = 0;
      for i = 1:obj.nT
        for j = 1:length(obj.F_idx{i})
          obj.A_newton((i-1)*3+(1:3),obj.F_idx{i}{j}(:)) = repmat(obj.A_force{obj.F2fsrc_map{i}(j)},1,size(obj.F_idx{i}{j},2));
          fsrc_idx = obj.F2fsrc_map{i}(j);
          num_force_weight_ij = obj.fsrc_cnstr{fsrc_idx}.num_force_weight;
          obj.A_margin(force_weight_count+(1:num_force_weight_ij),obj.F_idx{i}{j}(:)) = -eye(num_force_weight_ij);
          obj.A_margin(force_weight_count+(1:num_force_weight_ij),obj.margin_idx(i)) = 1;
          force_weight_count = force_weight_count+num_force_weight_ij;
        end
      end
      
      obj.x_lb(obj.sdotsquare_idx(:)) = 0;
      obj.Q_cost = sparse(obj.epsilon_idx(:),obj.epsilon_idx(:),ones(3*obj.nT,1),obj.num_vars,obj.num_vars);
      obj.f_cost = zeros(obj.num_vars,1);
      obj.f_cost(obj.margin_idx(:)) = -c_margin;
    end
    
    function [F,sdotsquare,Hdot,Hbar,tau] = solve(obj,com,comp,compp,foot_pos,tau)
      % @param com    - A 3 x obj.nT matrix. com(:,i) is the CoM position at i'th knot
      % @param comp   - A 3 x obj.nT matrix. comp(:,i) is the first derivative of CoM
      % w.r.t time scaling function s at i'th knot point
      % @param compp  - A 3 x obj.nT matrix. compp(:,i) is the second derivative of CoM
      % w.r.t time scaling function s at i'th knot point
      % @param foot_pos  - A 2 x obj.num_fsrc_cnstr matrix. foot_pos(:,i) is the xy
      % position of the contact body in obj.fsrc_cnstr{i}
      % @param tau    A scalar. The upper bound for the angular PD residue. sum_n
      % epsilon[n]'*epsilon[n] <= tau
      % @retval F     A cell array. F{i}{j} is the force parameters for
      % obj.fsrc_cnstr(obj.F2fsrc_map{i}(j))
      % @retval sdotsquare  A 1 x obj.nT vector. sdotsquare(i) is the square of the time
      % scaling function at time obj.t_knot(i)
      % @retval Hdot   A 3 x obj.nT matrix. Hdot[i] is the rate of the angular momentum at
      % i'th knot point
      % @retval Hbar   A 3 x obj.nT matrix. The integration of Hdot w.r.t the time scaling
      % function s.
      sizecheck(com,[3,obj.nT]);
      sizecheck(comp,[3,obj.nT]);
      sizecheck(compp,[3,obj.nT]);
      sizecheck(foot_pos,[2,obj.num_fsrc_cnstr]);
      sizecheck(tau,[1,1]);
      % compute the CoMddot from the comp,compp and time scaling function
      iAcomddot = reshape(repmat(reshape(1:3*obj.nT,3,obj.nT),2,1),[],1);
      jAcomddot = reshape([bsxfun(@times,ones(3,1),[obj.sdotsquare_idx(1:end-1) obj.sdotsquare_idx(end-1)])...
        bsxfun(@times,ones(3,1),[obj.sdotsquare_idx(2:end) obj.sdotsquare_idx(end)])],[],1);
      delta_s = 1/(obj.nT-1);
      Aval_comddot = reshape(obj.robot_mass*[compp-comp*2/delta_s;comp*2/delta_s],[],1);
      A_comddot = obj.A_newton-sparse(iAcomddot,jAcomddot,Aval_comddot,3*obj.nT,obj.num_vars);
      A_comddot_bnd = repmat([0;0;obj.robot_mass*obj.g],obj.nT,1);
      A_angular = zeros(3*obj.nT,obj.num_vars);
      A_angular_bnd = zeros(3*obj.nT,1);
      foot_contact_pts_pos = cell(1,obj.num_fsrc_cnstr);
      for i = 1:obj.num_fsrc_cnstr
        foot_contact_pts_pos{i} = bsxfun(@times,obj.A_xy(:,:,i)*foot_pos(:,i)+obj.b_xy(:,:,i),...
          ones(1,obj.fsrc_cnstr{i}.num_contact_pts))+obj.rotmat(:,:,i)*obj.fsrc_cnstr{i}.body_contact_pts;
      end
      for i = 1:obj.nT
        for j = 1:length(obj.F_idx{i})
          fsrc_idx = obj.F2fsrc_map{i}(j);
          num_pts_ij = obj.fsrc_cnstr{fsrc_idx}.num_contact_pts;
          foot_contact_pos_CoM = foot_contact_pts_pos{fsrc_idx}-bsxfun(@times,com(:,i),ones(1,num_pts_ij));
%           iA_cross = reshape(bsxfun(@times,[1;1;2;2;3;3],ones(1,num_pts_ij)),[],1);
%           jA_cross = reshape(bsxfun(@plus,[2;3;1;3;1;2],3*(0:num_pts_ij-1)),[],1);
%           Aval_cross = reshape([-foot_contact_pos_CoM(3,:);foot_contact_pos_CoM(2,:);foot_contact_pos_CoM(3,:);-foot_contact_pos_CoM(1,:);-foot_contact_pos_CoM(2,:);foot_contact_pos_CoM(1,:)],[],1);
%           Across = sparse(iA_cross,jA_cross,Aval_cross,3,3*num_pts_ij);
          A_angular((i-1)*3+(1:3),obj.F_idx{i}{j}(:)) = cross(reshape(repmat(foot_contact_pos_CoM,obj.fsrc_cnstr{fsrc_idx}.num_edges,1),3,[]),...
            repmat(obj.A_force{fsrc_idx},1,obj.fsrc_cnstr{fsrc_idx}.num_contact_pts));
        end
        A_angular((i-1)*3+(1:3),obj.Hdot_idx(:,i)) = -eye(3);
      end
      
      model.A = sparse([obj.A_margin;A_comddot;A_angular;obj.A_H;obj.A_angular_PD]);
      model.rhs = [obj.A_margin_bnd;A_comddot_bnd;A_angular_bnd;obj.A_H_bnd;obj.A_angular_PD_bnd];
      model.sense = [repmat('<',obj.num_force_weight,1);repmat('=',3*obj.nT+3*obj.nT+3*(obj.nT-1)+3*obj.nT,1)];
      model.Q = obj.Q_cost;
      model.obj = obj.f_cost;
      model.lb = obj.x_lb;
      model.ub = obj.x_ub;
      model.cone.index = [obj.tau_idx obj.epsilon_idx(:)'];
      
      params = struct();
      
      result = gurobi(model,params);
      if(strcmp(result.status,'OPTIMAL'))
        F = cell(1,obj.nT);
        for i = 1:obj.nT
          for j = 1:length(obj.F_idx{i})
            fsrc_idx = obj.F2fsrc_map{i}(j);
            F{i} = [F{i} {reshape(result.x(obj.F_idx{i}{j}),obj.fsrc_cnstr{fsrc_idx}.num_edges,obj.fsrc_cnstr{fsrc_idx}.num_contact_pts)}];
          end
        end
        sdotsquare = reshape(result.x(obj.sdotsquare_idx),1,[]);
        Hdot = reshape(result.x(obj.Hdot_idx(:)),3,obj.nT);
        Hbar = reshape(result.x(obj.H_idx(:)),3,obj.nT);
        epsilon = reshape(result.x(obj.epsilon_idx(:)),3,obj.nT);
        tau = sum(sum(epsilon.*epsilon));
      end
    end
  end
end