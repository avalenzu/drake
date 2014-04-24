classdef FixedFootYawCoMPlanningPosition
  % 'P-step' of the alternative planning for CoM trajectory and force trajectory. This is
  % a SOCP problem for linearized friction cone.
  % @properties robot_mass  The mass of the robot
  % @properties t_not     The time knot for planning
  % @properties g         The gravitational acceleration
  % @properties nT        The length of obj.t_knot
  % @properties com_idx   A 3 x obj.nT matrix. x(com_idx(:,i)) is the CoM position at time
  % t_knot(i) in the decision variable x
  % @properties comdot_idx   A 3 x obj.nT matrix. x(comdot_idx(:,i)) is the CoM velocity at time
  % t_knot(i) in the decision variable x
  % @properties comddot_idx   A 3 x obj.nT matrix. x(comddot_idx(:,i)) is the CoM acceleration at time
  % t_knot(i) in the decision variable x
  % @properties H_idx         A 3 x obj.nT matrix. x(H_idx(:,i)) is the centroidal angular
  % momentum at time t_knot(i)
  % @properties Hdot_idx      A 3 x obj.nT matrix. x(Hdot_idx(:,i)) is the rate of
  % centroidal angular momentum at time t_knot(i)
  % @properties epsilon_idx   A 3 x obj.nT matrix. x(epsilon_idx(:,i)) is the residue of
  % the PD law Hdot[i] = lambda*H[i]+epsilon[i]
  % @properties tau_idx      An integer scalar. x(tau_idx) is the index of tau, which is
  % the dummy variable used to bound the PD residue: sum_n epsilon[n]'*epsilon[n] <= tau
  % @properties fscr_cnstr    a 1 X obj.nT cell. fscr_cnstr{i} is a cell containing all
  % the FootStepContactRegionConstraint that is active at time t_knot(i)
  % @properties contact_body_pos_idx  A 1 x obj.nT cell. x(contact_body_pos_idx{i}(:,j)) is the xy position
  % of the contact body being active in obj.fscr_cnstr{i}{j} at time t_knot(i) 
  % @properties x_lb    A obj.num_vars x 1 double vector. The lower bound for the decision
  % variable x
  % @properties x_ub    A obj.num_vars x 1 double vector. The upper bound for the decision
  % variable x
  % @properties Q_cost  A obj.num_vars x obj.num_vars sparse PSD matrix. The Hessian of
  % the quadratic cost
  % @properties A_iris,b_iris  A_iris * x <= b_iris is the union of all half space
  % constraint on the foot location from iris
  % @properties A_com,A_com_bnd   A_com * x = A_com_bnd is the constraint on the euler
  % integration of CoM
  % @properties A_H,A_H_bnd      A_H * x = A_H_bnd is the constraint on the euler
  % integraton of angular momentum
  % @properties A_angular_PD,A_angular_PD_bnd   A_angular_PD * x = A_angular_PD_bnd is the
  % constraint Hdot[n] = lambda*H[n]+epsilon[n]
  % @properties yaw    A cell array. yaw{i}(j) is the yaw angle for obj.fsrc_cnstr{i}{j}
  properties(SetAccess = protected)
    robot_mass
    t_knot
    g
    nT
    com_idx;
    comdot_idx;
    comddot_idx;
    contact_body_pos_idx;
    H_idx;
    Hdot_idx
    epsilon_idx
    tau_idx
    num_vars
    x_names
    fsrc_cnstr
    x_lb
    x_ub
    Q_cost
    A_iris
    b_iris
    A_com
    A_com_bnd
    A_H
    A_H_bnd
    A_angular_PD
    A_angular_PD_bnd
    yaw
  end
  
  properties(Access = protected)
    A_force % A cell array. A_force{i}{j} is a 3 x obj.fsrc_cnstr.num_edges matrix. A_force{i}{j}*w is the force on the linearized friction cone where w is the force weight.
    A_xy,b_xy,rotmat  % A_xy,b_xy,rotmat are all cell arrays. [A_xy{i}{j},b_xy{i}{j},rotmat{i}{j}] = obj.fsrc_cnstr{i}{j}.bodyTransform(obj.yaw{i}(j))
  end
  
  methods
    function obj = FixedFootYawCoMPlanningPosition(robot_mass,t,lambda,Q_comddot,varargin)
      % obj =
      % FixedFootYawCoMPlanningPosition(robot_mass,t,Q_Hdot,Q_H,foot_step_region_contact_cnstr1,yaw1,foot_step_region_contact_cnstr2,yaw2,...)
      % @properties robot_mass    The mass of the robot
      % @properties t             The time knot for planning
      % @properties lambda        A 3 x 3 Hurwitz matrix. It tries to drive the angular
      % momentum stays at 0 by putting the constraint Hdot[n] = lambda*H[n]+epsilon[n]
      % @properties Q_comddot     A 3 x 3 PSD matrix. The cost is sum_n
      % comddot[n]'*Q_comddot*comddot[n]+epsilon[n]'*epsilon[n]
      % @properties foot_step_region_contact_cnstr    A FootStepRegionContactConstraint
      % @properties yaw           A double scalar. The yaw angle of the foot
      if(~isnumeric(robot_mass))
        error('Drake:FixedFootYawCoMPlanningPosition:robot mass should be numeric');
      end
      sizecheck(robot_mass,[1,1]);
      if(robot_mass<=0)
        error('Drake:FixedFootYawCoMPlanningPosition:robot mass should be positive');
      end
      obj.robot_mass = robot_mass;
      if(~isnumeric(t))
        error('Drake:FixedFootYawCoMPlanningPosition:t should be numeric');
      end
      obj.t_knot = reshape(unique(t),1,[]);
      obj.nT = length(obj.t_knot);
      obj.g = 9.81;
      if(~isnumeric(lambda))
        error('Drake:FixedFootYawCoMPlanningPosition:lambda should be numeric');
      end
      sizecheck(lambda,[3,3]);
      if(any(eig(lambda)>=0))
        error('Drake:FixedFootYawCoMPlanningPosition:lambda should be a Hurwitz matrix. Namely all its eigen values should be negative');
      end
      if(~isnumeric(Q_comddot))
        error('Drake:FixedFootYawCoMPlanningPosition:Q_comddot should be numeric');
      end
      sizecheck(Q_comddot,[3,3]);
      if(any(eig(Q_comddot)<0))
        error('Drake:FixedFootYawCoMPlanningPosition:Q_comddot should be a positive semi-definite matrix');
      end
      obj.num_vars = 0;
      obj.com_idx = reshape(obj.num_vars+(1:3*obj.nT),3,obj.nT);
      obj.x_names = cell(obj.num_vars,1);
      for i = 1:obj.nT
        obj.x_names((i-1)*3+(1:3)) = {sprintf('com_x[%d]',i);sprintf('com_y[%d]',i);sprintf('com_z[%d]',i)};
      end
      obj.num_vars = obj.num_vars+3*obj.nT;
      obj.comdot_idx = reshape(obj.num_vars+(1:3*obj.nT),3,obj.nT);
      obj.x_names = [obj.x_names;cell(3*obj.nT,1)];
      for i = 1:obj.nT
        obj.x_names(obj.num_vars+(i-1)*3+(1:3)) = {sprintf('comdot_x[%d]',i);sprintf('comdot_y[%d]',i);sprintf('comdot_z[%d]',i)};
      end
      obj.num_vars = obj.num_vars+3*obj.nT;
      obj.comddot_idx = reshape(obj.num_vars+(1:3*obj.nT),3,obj.nT);
      obj.x_names = [obj.x_names;cell(3*obj.nT,1)];
      for i = 1:obj.nT
        obj.x_names(obj.num_vars+(i-1)*3+(1:3)) = {sprintf('comddot_x[%d]',i);sprintf('comddot_y[%d]',i);sprintf('comddot_z[%d]',i)};
      end
      obj.num_vars = obj.num_vars+3*obj.nT;
      obj.H_idx = reshape(obj.num_vars+(1:3*obj.nT),3,obj.nT);
      obj.x_names = [obj.x_names;cell(3*obj.nT,1)];
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
      
      obj.fsrc_cnstr = cell(1,obj.nT);
      obj.contact_body_pos_idx = cell(1,obj.nT);
      obj.yaw = cell(1,obj.nT);
      obj.A_force = cell(1,obj.nT);
      obj.A_xy = cell(1,obj.nT);
      obj.b_xy = cell(1,obj.nT);
      obj.rotmat = cell(1,obj.nT);
      iA_iris = [];
      jA_iris = [];
      Aval_iris = [];
      obj.b_iris = [];
      num_halfspace_iris = 0;
      for i = 1:(nargin-4)/2
        if(~isa(varargin{2*i-1},'FootStepRegionContactConstraint'))
          error('Drake:FixedFootYawCoMPlanningPosition:The input should be a FootStepRegionContactConstraint');
        end
        if(~isnumeric(varargin{2*i}))
          error('Drake:FixedFootYawCoMPlanningPosition:The input yaw angle should be a double');
        end
        sizecheck(varargin{2*i},[1,1]);
        fsrc_pos_idx = obj.num_vars+(1:2)';
        is_fsrc_active = false;
        A_iris_i = varargin{2*i-1}.foot_step_region_cnstr.A;
        b_iris_i = varargin{2*i-1}.foot_step_region_cnstr.b;
        iA_iris_i = num_halfspace_iris+reshape([(1:size(A_iris_i,1))' (1:size(A_iris_i,1))'],[],1);
        jA_iris_i = [(obj.num_vars+1)*ones(size(A_iris_i,1),1);(obj.num_vars+2)*ones(size(A_iris_i,1),1)];
        Aval_iris_i = reshape(A_iris_i(:,1:2),[],1);
        iA_iris = [iA_iris;iA_iris_i];
        jA_iris = [jA_iris;jA_iris_i];
        Aval_iris = [Aval_iris;Aval_iris_i];
        obj.b_iris = [obj.b_iris;b_iris_i-varargin{2*i}*A_iris_i(:,3)];
        num_halfspace_iris = num_halfspace_iris+size(A_iris_i,1);
        obj.x_names = [obj.x_names;{sprintf('Foot x position for %d''th FootStepRegionContactConstraint',i);...
          sprintf('Foot y position for %d''th FootStepRegionContactConstraint',i)}];
        for j = 1:obj.nT
          if(varargin{2*i-1}.foot_step_region_cnstr.isTimeValid(obj.t_knot(j)))
            obj.fsrc_cnstr{j} = [obj.fsrc_cnstr{j} varargin(2*i-1)];
            obj.contact_body_pos_idx{j} = [obj.contact_body_pos_idx{j} fsrc_pos_idx];
            obj.yaw{j} = [obj.yaw{j} varargin{2*i}];
            obj.A_force{j} = [obj.A_force{j} {varargin{2*i-1}.force(varargin{2*i})}];
            [rotmat_ij,A_xy_ij,b_xy_ij] = varargin{2*i-1}.foot_step_region_cnstr.bodyTransform(varargin{2*i});
            obj.rotmat{j} = [obj.rotmat{j},{rotmat_ij}];
            obj.A_xy{j} = [obj.A_xy{j},{A_xy_ij}];
            obj.b_xy{j} = [obj.b_xy{j},{b_xy_ij}];
            is_fsrc_active = true;
          end
        end
        if(~is_fsrc_active)
          error('Drake:FixedFootYawCoMPlanningPosition:The %dth FootStepRegionContactConstraint is not active for any t_knot');
        end
        obj.num_vars = obj.num_vars+2;
      end
      obj.A_iris = sparse(iA_iris,jA_iris,Aval_iris,num_halfspace_iris,obj.num_vars);
      dt = reshape(diff(obj.t_knot),1,[]);
      % linear constraint that com[n]-com[n-1] = comdot[n]*dt and comdot[n]-comdot[n-1] =
      % comddot[n]*dt
      iAcom = [(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';];
      jAcom = [reshape(obj.com_idx(:,2:end),[],1);reshape(obj.com_idx(:,1:end-1),[],1); reshape(obj.comdot_idx(:,2:end),[],1)];
      Aval_com = [ones(3*(obj.nT-1),1);-ones(3*(obj.nT-1),1);-reshape(bsxfun(@times,ones(3,1),dt),[],1)];
      iAcom = [iAcom;3*(obj.nT-1)+iAcom];
      jAcom = [jAcom;reshape(obj.comdot_idx(:,2:end),[],1);reshape(obj.comdot_idx(:,1:end-1),[],1); reshape(obj.comddot_idx(:,2:end),[],1)];
      Aval_com = [Aval_com;Aval_com];
      obj.A_com = sparse(iAcom,jAcom,Aval_com,6*(obj.nT-1),obj.num_vars);
      obj.A_com_bnd = zeros(6*(obj.nT-1),1);
      % linear constraint that H[n]-H[n-1] = Hdot[n]*dt
      iA_H = [(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';];
      jA_H = [reshape(obj.H_idx(:,2:end),[],1);reshape(obj.H_idx(:,1:end-1),[],1); reshape(obj.Hdot_idx(:,2:end),[],1)];
      Aval_H = [ones(3*(obj.nT-1),1);-ones(3*(obj.nT-1),1);-reshape(bsxfun(@times,ones(3,1),dt),[],1)];
      obj.A_H = sparse(iA_H,jA_H,Aval_H,3*(obj.nT-1),obj.num_vars);
      obj.A_H_bnd = zeros(3*(obj.nT-1),1);
      % linear constraint that Hdot[n] = lambda*H[n]+epsilon[n]
      iA_angular_PD = [(1:3*obj.nT)';reshape(repmat(reshape((1:3*obj.nT),3,obj.nT),3,1),[],1);(1:3*obj.nT)'];
      jA_angular_PD = [obj.Hdot_idx(:);reshape(bsxfun(@times,obj.H_idx(:)',ones(3,1)),[],1);obj.epsilon_idx(:)];
      Aval_angular_PD = [ones(3*obj.nT,1);reshape(repmat(-lambda,1,obj.nT),[],1);-ones(3*obj.nT,1)];
      obj.A_angular_PD = sparse(iA_angular_PD,jA_angular_PD,Aval_angular_PD,3*obj.nT,obj.num_vars);
      obj.A_angular_PD_bnd = zeros(3*obj.nT,1);
      
      % cost sum comddot[n]'*comddot[n]+epsilon[n]'*epsilon[n]
      iQcost = [reshape(repmat(obj.comddot_idx,3,1),[],1);obj.epsilon_idx(:)];
      jQcost = [reshape(bsxfun(@times,obj.comddot_idx(:)',ones(3,1)),[],1);obj.epsilon_idx(:)];
      Qval_cost = [reshape(repmat(Q_comddot,1,obj.nT),[],1);ones(3*obj.nT,1)];
      obj.Q_cost = sparse(iQcost,jQcost,Qval_cost,obj.num_vars,obj.num_vars);
      % bounds on decision variables x
      obj.x_lb = -inf(obj.num_vars,1);
      obj.x_ub = inf(obj.num_vars,1);
    end
    
    function [com,comdot,comddot,foot_pos,Hdot,H,tau] = solve(obj,F,tau)
      % @properties F     A cell array. F{i}{j} is the force parameters for
      % obj.fsrc_cnstr{i}{j}
      % @properties tau   A scalar. The upper bound for sum_n
      % epsilon[n]'*epsilon[n]
      % @retval tau   A scalar. The updated value for sum_n epsilon[n]'*epsilon[n]
      if(~iscell(F) || length(F) ~= obj.nT)
        error('Drake:FixedFootYawCoMPlanningPosition:F should be a cell of length obj.nT');
      end
      A_angular = zeros(3*obj.nT,obj.num_vars);
      A_angular_bnd = zeros(3*obj.nT,1);
      for i = 1:obj.nT
        F_i = zeros(3,1);
        A_angular((i-1)*3+(1:3),obj.Hdot_idx(:,i)) = eye(3);
        for j = 1:length(obj.A_force{i})
          num_contact_pts_ij = obj.fsrc_cnstr{i}{j}.num_contact_pts;
          sizecheck(F{i}{j},[obj.fsrc_cnstr{i}{j}.num_edges,num_contact_pts_ij]);
          F_ij = obj.A_force{i}{j}*F{i}{j}; % F_ij(:,k) is the contact force at the k'th corner of the foot.
          F_i = F_i+sum(F_ij,2);
          A_angular((i-1)*3+(1:3),obj.contact_body_pos_idx{i}(:,j)) = ...
            [sum(cross(F_ij,bsxfun(@times,obj.A_xy{i}{j}(:,1),ones(1,num_contact_pts_ij))),2) sum(cross(F_ij,bsxfun(@times,obj.A_xy{i}{j}(:,2),ones(1,num_contact_pts_ij))),2)];
          A_angular_bnd((i-1)*3+(1:3)) = A_angular_bnd((i-1)*3+(1:3))-sum(cross(F_ij,bsxfun(@times,obj.b_xy{i}{j},ones(1,num_contact_pts_ij))+obj.rotmat{i}{j}*obj.fsrc_cnstr{i}{j}.body_contact_pts),2);
        end
        A_angular((i-1)*3+(1:3),obj.com_idx(:,i)) = -[0 -F_i(3) F_i(2);F_i(3) 0 -F_i(1);-F_i(2) F_i(1) 0];
        F_i(3) = F_i(3)-obj.robot_mass*obj.g;
        obj.x_lb(obj.comddot_idx(:,i)) = F_i/obj.robot_mass;
        obj.x_ub(obj.comddot_idx(:,i)) = F_i/obj.robot_mass;
      end
      model.A = sparse([obj.A_iris;obj.A_com;obj.A_H;obj.A_angular_PD;A_angular]);
      model.rhs = [obj.b_iris;obj.A_com_bnd;obj.A_H_bnd;obj.A_angular_PD_bnd;A_angular_bnd];
      model.sense = [repmat('<',size(obj.A_iris,1),1);repmat('=',6*(obj.nT-1)+3*(obj.nT-1)+3*obj.nT+3*obj.nT,1)];
      model.Q = obj.Q_cost;
      model.obj = zeros(1,obj.num_vars);
      obj.x_lb(obj.tau_idx) = tau;
      obj.x_ub(obj.tau_idx) = tau;
      model.lb = obj.x_lb;
      model.ub = obj.x_ub;
      model.cones.index = [obj.tau_idx obj.epsilon_idx(:)'];
      params = struct();

      result = gurobi(model,params);
      if(strcmp(result.status,'OPTIMAL'))
        com = reshape(result.x(obj.com_idx(:)),3,obj.nT);
        comdot = reshape(result.x(obj.comdot_idx(:)),3,obj.nT);
        comddot = reshape(result.x(obj.comddot_idx(:)),3,obj.nT);
        Hdot = reshape(result.x(obj.Hdot_idx(:)),3,obj.nT);
        H = reshape(result.x(obj.H_idx(:)),3,obj.nT);
        foot_pos = cell(1,obj.nT);
        epsilon = reshape(result.x(obj.epsilon_idx(:)),3,obj.nT);
        tau = sum(sum(epsilon.*epsilon));
        for i = 1:obj.nT
          foot_pos{i} = reshape(result.x(obj.contact_body_pos_idx{i}(:)),2,[]);
        end
      end
    end
    
    function obj = setVarBounds(obj,lb,ub,xind)
      % @properties lb,ub    The lower and uppper bound of the variables
      % @properties xind     The indices of the variables whose bounds are going to be set
      lb = lb(:);
      ub = ub(:);
      xind = xind(:);
      num_x = length(xind);
      if(num_x ~= length(lb) || num_x ~= length(ub))
        error('Drake:FixedFootYawCoMPlanningPosition: bound sizes do not match');
      end
      if(any(lb>ub))
        error('Drake:FixedFootYawCoMPlanningPosition: lower bound should be no larger than the upper bound');
      end
      obj.x_lb(xind) = lb;
      obj.x_ub(xind) = ub;
    end
  end
end