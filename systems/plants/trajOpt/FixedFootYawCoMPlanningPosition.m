classdef FixedFootYawCoMPlanningPosition
  % 'P-step' of the alternative planning for CoM trajectory and force trajectory. This is
  % a SOCP problem for linearized friction cone.            
  properties(SetAccess = protected)
    robot_mass % The mass of the robot
    t_knot % The time knot for planning
    g % The gravitational acceleration
    nT % The length of obj.t_knot
    com_idx; % A 3 x obj.nT matrix. x(com_idx(:,i)) is the CoM position at time t_knot(i) in the decision variable x
    comp_idx; % A 3 x obj.nT matrix. x(comp_idx(:,i)) is the first derivative of CoM w.r.t time scaling function s a i'th knot point
    compp_idx; % A 3 x obj.nT matrix. x(compp_idx(:,i)) is the second derivative of CoM w.r.t time scaling function s a i'th knot point
    H_idx; %A 3 x obj.nT matrix. x(H_idx(:,i)) is the centroidal angular momentum at time t_knot(i)
    Hdot_idx % A 3 x obj.nT matrix. x(Hdot_idx(:,i)) is the rate of centroidal angular momentum at time t_knot(i)
    epsilon_idx % A 3 x obj.nT matrix. x(epsilon_idx(:,i)) is the residue of the PD law Hdot[i] = lambda*H[i]+epsilon[i]
    sigma_idx % An integer scalar. x(sigma_idx) is the index of sigma, which is the dummy variable used to bound the PD residue: sum_n epsilon[n]'*epsilon[n] <= sigma  
    num_vars % The total number of decision variables in the SOCP 
    x_names % A obj.num_vars x 1 double vector. The lower bound for the decision variable x
    x_lb % A obj.num_vars x 1 double vector. The lower bound for the decision variable x
    x_ub % A obj.num_vars x 1 double vector. The upper bound for the decision variable x
    Q_cost % A obj.num_vars x obj.num_vars sparse PSD matrix. The Hessian of the quadratic cost
    A_iris,b_iris % A_iris * x <= b_iris is the union of all half space constraint on the foot location from iris
    A_com,A_com_bnd % A_com * x = A_com_bnd is the constraint on the euler integration of CoM
    A_H,A_H_bnd % A_H * x = A_H_bnd is the constraint on the euler integraton of angular momentum
    A_angular_PD,A_angular_PD_bnd % A_angular_PD * x = A_angular_PD_bnd is the constraint Hdot[n] = lambda*H[n]+epsilon[n]
    num_fsrc_cnstr % An integer. The total number of FootStepRegionContactConstraint
    fsrc_cnstr % A cell array. All the FootStepRegionContactConstraint object
    fsrc_body_pos_idx % A 2 x length(fsrc_cnstr) matrix. x(obj.fsrc_body_pos_idx(:,i)) is the body position for the i'th FootStepRegionContactConstraint in the decision variables.
    F2fsrc_map % A cell arry. obj..fsrc_cnstr{F2fsrc_map{i}(j)} is the FootStepContactRegionConstraint corresponds to the force x(obj.F_idx{i}{j})
    yaw % A 1 x num_fsrc_cnstr double vector. yaw(i) is the yaw angle for obj.fsrc_cnstr{i}
    A_kin,b_kin  % A_kin*x<=b_kin encodes the kinematic constraint on the contact points and CoM
    lb_comdot,ub_comdot % lb_comdot and ub_comdot are the lower and upper bound of the CoM velocities respectively
  end
  
  properties(Access = protected)
    A_force % A cell array.  A_force{i} = obj.fsrc_cnstr{i}.force, which is a 3 x obj.fsrc_cnstr[i}.num_edges matrix
    A_xy,b_xy,rotmat  % A_xy is 3 x 2 x obj.num_fsrc_cnstr matrix. b_xy is 3 x 1 x obj.num_fsrc_cnstr matrix. rotmat is 3 x 3 x obj.num_fsrc_cnstr matrix. [rotmat(:,:,i),A_xy(:,:,i),b_xy(:,:,i)] = obj.fsrc_cnstr{i}.bodyTransform(obj.yaw(i)); 
  end
  
  methods
    function obj = FixedFootYawCoMPlanningPosition(robot_mass,t,lambda,Q_comddot,fsrc_cnstr,yaw,F2fsrc_map,A_force,A_xy,b_xy,rotmat)
      % obj =
      % FixedFootYawCoMPlanningPosition(robot_mass,t,lambda,Q_comddot,foot_step_region_contact_cnstr1,yaw1,foot_step_region_contact_cnstr2,yaw2,...)
      % @param robot_mass    The mass of the robot
      % @param t             The time knot for planning. This indicates which
      % FootStepRegionContactConstraint is active at a given time knot. The actual time is
      % determined by the scaling function.
      % @param lambda        A 3 x 3 Hurwitz matrix. It tries to drive the angular
      % momentum stays at 0 by putting the constraint Hdot[n] = lambda*H[n]+epsilon[n]
      % @param Q_comddot     A 3 x 3 PSD matrix. The cost is sum_n
      % comddot[n]'*Q_comddot*comddot[n]+epsilon[n]'*epsilon[n]
      % @param fsrc_cnstr  A cell array. All the FootStepRegionContactConstraint object
      % @param yaw     A 1 x num_fsrc_cnstr double vector. yaw(i) is the yaw angle for obj.fsrc_cnstr{i}
      % @param A_xy,b_xy,rotmat   A_xy is 3 x 2 x obj.num_fsrc_cnstr matrix. b_xy is 3 x 1 x obj.num_fsrc_cnstr matrix. rotmat is 3 x 3 x obj.num_fsrc_cnstr matrix. [rotmat(:,:,i),A_xy(:,:,i),b_xy(:,:,i)] = obj.fsrc_cnstr{i}.bodyTransform(obj.yaw(i)); 
      % @param A_force    A_force{i} = obj.fsrc_cnstr{i}.force, which is a 3 x obj.fsrc_cnstr[i}.num_edges matrix
      obj.robot_mass = robot_mass;
      obj.t_knot = t;
      obj.nT = length(obj.t_knot);
      obj.g = 9.81;
      obj.num_fsrc_cnstr = length(fsrc_cnstr);
      obj.fsrc_cnstr = fsrc_cnstr;
      obj.yaw = yaw;
      obj.A_xy = A_xy;
      obj.b_xy = b_xy;
      obj.rotmat = rotmat;
      obj.F2fsrc_map = F2fsrc_map;
      obj.A_force = A_force;
      obj.num_vars = 0;
      obj.com_idx = reshape(obj.num_vars+(1:3*obj.nT),3,obj.nT);
      obj.x_names = cell(obj.num_vars,1);
      for i = 1:obj.nT
        obj.x_names((i-1)*3+(1:3)) = {sprintf('com_x[%d]',i);sprintf('com_y[%d]',i);sprintf('com_z[%d]',i)};
      end
      obj.num_vars = obj.num_vars+3*obj.nT;
      obj.comp_idx = reshape(obj.num_vars+(1:3*obj.nT),3,obj.nT);
      obj.x_names = [obj.x_names;cell(3*obj.nT,1)];
      for i = 1:obj.nT
        obj.x_names(obj.num_vars+(i-1)*3+(1:3)) = {sprintf('comp_x[%d]',i);sprintf('comp_y[%d]',i);sprintf('comp_z[%d]',i)};
      end
      obj.num_vars = obj.num_vars+3*obj.nT;
      obj.compp_idx = reshape(obj.num_vars+(1:3*obj.nT),3,obj.nT);
      obj.x_names = [obj.x_names;cell(3*obj.nT,1)];
      for i = 1:obj.nT
        obj.x_names(obj.num_vars+(i-1)*3+(1:3)) = {sprintf('compp_x[%d]',i);sprintf('compp_y[%d]',i);sprintf('compp_z[%d]',i)};
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
      obj.sigma_idx = obj.num_vars+1;
      obj.x_names = [obj.x_names;{'sigma'}];
      obj.num_vars = obj.num_vars+1;
      
      obj.num_fsrc_cnstr = length(fsrc_cnstr);
      obj.fsrc_body_pos_idx = zeros(2,obj.num_fsrc_cnstr);
      iA_iris = [];
      jA_iris = [];
      Aval_iris = [];
      obj.b_iris = [];
      num_halfspace_iris = 0;
      for i = 1:obj.num_fsrc_cnstr
        fsrc_pos_idx = obj.num_vars+(1:2)';
        obj.fsrc_body_pos_idx(:,i) = fsrc_pos_idx;
        A_iris_i = obj.fsrc_cnstr{i}.foot_step_region_cnstr.A;
        b_iris_i = obj.fsrc_cnstr{i}.foot_step_region_cnstr.b;
        iA_iris_i = num_halfspace_iris+reshape([(1:size(A_iris_i,1))' (1:size(A_iris_i,1))'],[],1);
        jA_iris_i = [(obj.num_vars+1)*ones(size(A_iris_i,1),1);(obj.num_vars+2)*ones(size(A_iris_i,1),1)];
        Aval_iris_i = reshape(A_iris_i(:,1:2),[],1);
        iA_iris = [iA_iris;iA_iris_i];
        jA_iris = [jA_iris;jA_iris_i];
        Aval_iris = [Aval_iris;Aval_iris_i];
        obj.b_iris = [obj.b_iris;b_iris_i-obj.yaw(i)*A_iris_i(:,3)];
        num_halfspace_iris = num_halfspace_iris+size(A_iris_i,1);
        obj.x_names = [obj.x_names;{sprintf('Foot x position for %d''th FootStepRegionContactConstraint',i);...
          sprintf('Foot y position for %d''th FootStepRegionContactConstraint',i)}];
        obj.num_vars = obj.num_vars+2;
      end
      obj.A_iris = sparse(iA_iris,jA_iris,Aval_iris,num_halfspace_iris,obj.num_vars);
      delta_s = 1/(obj.nT-1);
      % linear constraint that com[n]-com[n-1] = comdot[n]*dt and comdot[n]-comdot[n-1] =
      % comddot[n]*dt
      iAcom = [(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';];
      jAcom = [reshape(obj.com_idx(:,2:end),[],1);reshape(obj.com_idx(:,1:end-1),[],1); reshape(obj.comp_idx(:,2:end),[],1)];
      Aval_com = [ones(3*(obj.nT-1),1);-ones(3*(obj.nT-1),1);-reshape(bsxfun(@times,ones(3,1),delta_s*ones(1,obj.nT-1)),[],1)];
      iAcom = [iAcom;3*(obj.nT-1)+iAcom];
      jAcom = [jAcom;reshape(obj.comp_idx(:,2:end),[],1);reshape(obj.comp_idx(:,1:end-1),[],1); reshape(obj.compp_idx(:,2:end),[],1)];
      Aval_com = [Aval_com;Aval_com];
      obj.A_com = sparse(iAcom,jAcom,Aval_com,6*(obj.nT-1),obj.num_vars);
      obj.A_com_bnd = zeros(6*(obj.nT-1),1);
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
      
      % cost sum comddot[n]'*comddot[n]+epsilon[n]'*epsilon[n]
      iQcost = [reshape(repmat(obj.compp_idx,3,1),[],1);obj.epsilon_idx(:)];
      jQcost = [reshape(bsxfun(@times,obj.compp_idx(:)',ones(3,1)),[],1);obj.epsilon_idx(:)];
      Qval_cost = [reshape(repmat(Q_comddot,1,obj.nT),[],1);ones(3*obj.nT,1)];
      obj.Q_cost = sparse(iQcost,jQcost,Qval_cost,obj.num_vars,obj.num_vars);
      % bounds on decision variables x
      obj.x_lb = -inf(obj.num_vars,1);
      obj.x_ub = inf(obj.num_vars,1);
      
      % The kinematic constraint on the contact bodies will be set through function
      % addKinematicPolygon
      obj.A_kin = [];
      obj.b_kin = [];
      obj.lb_comdot = -inf(3,obj.nT);
      obj.ub_comdot = inf(3,obj.nT);
    end
    
    function [com,comp,compp,foot_pos,Hdot,Hbar,sigma,epsilon] = solve(obj,F,sdotsquare,sigma)
      % @param F     A cell array. F{i}{j} is the force parameters for
      % obj.fsrc_cnstr(obj.F2fsrc_map{i}(j))
      % @param sigma   A scalar. The upper bound for sum_n
      % epsilon[n]'*epsilon[n]
      % @param sdotsquare  A 1 x obj.nT vector. sdotsquare(i) is the square of the time
      % scaling function at time obj.t_knot(i)
      % @retval sigma   A scalar. The updated value for sum_n epsilon[n]'*epsilon[n]
      % @retval com   A 3 x obj.nT matrix. com(:,i) is the position of the robot at the
      % i'th knot point
      % @retval comp  A 3 x obj.nT matrix. comp(:,i) is the first derivative of com w.r.t
      % scaling function s at i'th knot point
      % @retval compp  A 3 x obj.nT matrix. compp(:,i) is the second derivative of com
      % w.r.t scaling function s at i'th knot point
      % @retval foot_pos   A 2 x obj.num_fsrc_cnstr matrix. foot_pos(:,i) is the xy
      % position of the contact body in obj.fsrc_cnstr{i}
      % @retval epsilon   A 3 x obj.nT matrix. The residue of the PD law on angular
      % momentum.
      if(~iscell(F) || length(F) ~= obj.nT)
        error('Drake:FixedFootYawCoMPlanningPosition:F should be a cell of length obj.nT');
      end
      sizecheck(sdotsquare,[1,obj.nT]);
      if(any(sdotsquare<=0))
        error('Drake:FixedFootYawCoMPlanningPosition:sdotsquare should all be positive');
      end
      sdotsquare_diff = diff(sdotsquare);
      sdotsquare_diff = [sdotsquare_diff sdotsquare_diff(end)];
      A_angular = zeros(3*obj.nT,obj.num_vars);
      A_angular_bnd = zeros(3*obj.nT,1);
      A_compp = zeros(3*obj.nT,obj.num_vars);
      A_compp_bnd = zeros(3*obj.nT,1);
      for i = 1:obj.nT
        F_i = zeros(3,1);
        A_angular((i-1)*3+(1:3),obj.Hdot_idx(:,i)) = eye(3);
        for j = 1:length(obj.F2fsrc_map{i})
          fsrc_idx = obj.F2fsrc_map{i}(j);
          num_contact_pts_ij = obj.fsrc_cnstr{fsrc_idx}.num_contact_pts;
          sizecheck(F{i}{j},[obj.fsrc_cnstr{fsrc_idx}.num_edges,num_contact_pts_ij]);
          F_ij = obj.A_force{fsrc_idx}*F{i}{j}; % F_ij(:,k) is the contact force at the k'th corner of the foot.
          F_i = F_i+sum(F_ij,2);
          A_angular((i-1)*3+(1:3),obj.fsrc_body_pos_idx(:,fsrc_idx)) = ...
            [sum(cross(F_ij,bsxfun(@times,obj.A_xy(:,1,fsrc_idx),ones(1,num_contact_pts_ij))),2) sum(cross(F_ij,bsxfun(@times,obj.A_xy(:,2,fsrc_idx),ones(1,num_contact_pts_ij))),2)];
          A_angular_bnd((i-1)*3+(1:3)) = A_angular_bnd((i-1)*3+(1:3))-sum(cross(F_ij,bsxfun(@times,obj.b_xy(:,:,fsrc_idx),ones(1,num_contact_pts_ij))+obj.rotmat(:,:,fsrc_idx)*obj.fsrc_cnstr{fsrc_idx}.body_contact_pts),2);
        end
        A_angular((i-1)*3+(1:3),obj.com_idx(:,i)) = -[0 -F_i(3) F_i(2);F_i(3) 0 -F_i(1);-F_i(2) F_i(1) 0];
        F_i(3) = F_i(3)-obj.robot_mass*obj.g;
        A_compp_bnd((i-1)*3+(1:3)) = F_i;
        A_compp((i-1)*3+(1:3),obj.compp_idx(:,i)) = obj.robot_mass*sdotsquare(i)*eye(3);
        A_compp((i-1)*3+(1:3),obj.comp_idx(:,i)) = obj.robot_mass*(obj.nT-1)/2*sdotsquare_diff(i)*eye(3);
      end
      model.A = sparse([obj.A_iris;obj.A_kin;obj.A_com;obj.A_H;obj.A_angular_PD;A_angular;A_compp]);
      model.rhs = [obj.b_iris;obj.b_kin;obj.A_com_bnd;obj.A_H_bnd;obj.A_angular_PD_bnd;A_angular_bnd;A_compp_bnd];
      model.sense = [repmat('<',size(obj.A_iris,1)+size(obj.A_kin,1),1);repmat('=',6*(obj.nT-1)+3*(obj.nT-1)+3*obj.nT+3*obj.nT+3*obj.nT,1)];
      model.Q = obj.Q_cost;
      model.obj = zeros(1,obj.num_vars);
      obj.x_lb(obj.sigma_idx) = sigma;
      obj.x_ub(obj.sigma_idx) = sigma;
      obj.x_lb(obj.comp_idx(:)) = reshape(obj.lb_comdot./bsxfun(@times,ones(3,1),sqrt(sdotsquare)),[],1);
      obj.x_ub(obj.comp_idx(:)) = reshape(obj.ub_comdot./bsxfun(@times,ones(3,1),sqrt(sdotsquare)),[],1);
      model.lb = obj.x_lb;
      model.ub = obj.x_ub;
      model.cones.index = [obj.sigma_idx obj.epsilon_idx(:)'];
      params = struct('OutputFlag',false);

      result = gurobi(model,params);
      if(strcmp(result.status,'OPTIMAL'))
        com = reshape(result.x(obj.com_idx(:)),3,obj.nT);
        comp = reshape(result.x(obj.comp_idx(:)),3,obj.nT);
        compp = reshape(result.x(obj.compp_idx(:)),3,obj.nT);
        Hdot = reshape(result.x(obj.Hdot_idx(:)),3,obj.nT);
        Hbar = reshape(result.x(obj.H_idx(:)),3,obj.nT);
        epsilon = reshape(result.x(obj.epsilon_idx(:)),3,obj.nT);
        sigma = sum(sum(epsilon.*epsilon));
        foot_pos = reshape(result.x(obj.fsrc_body_pos_idx),2,[]);
      else
        error('P-step is infeasible');
      end
    end
    
    function obj = setVarBounds(obj,lb,ub,xind)
      % @param lb,ub    The lower and uppper bound of the variables
      % @param xind     The indices of the variables whose bounds are going to be set
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
    
    function obj = setCoMVelocityBounds(obj,knot_idx,lb,ub)
      % set the lower and upper bounds for CoM velocities at the knots with indices
      % knot_ind
      % @param knot_idx   A 1 x m integer vector. The indices of the knots whose CoM
      % velocities will be bounded
      % @param lb     A 3 x m double vector. lb(:,i) is the lower bound of the CoM
      % velocity at the knot with index knot_idx(i)
      % @param ub     A 3 x m double vector. ub(:,i) is the upper bound of the CoM
      % velocity at the knot with index knot_idx(i)
      num_idx = numel(knot_idx);
      if(~isnumeric(knot_idx) || ~isnumeric(lb) || ~isnumeric(ub))
        error('Drake:FixedFootYawCoMPlanningPosition: input should be numeric');
      end
      sizecheck(knot_idx,[1,num_idx]);
      sizecheck(lb,[3,num_idx]);
      sizecheck(ub,[3,num_idx]);
      obj.lb_comdot(:,knot_idx) = lb;
      obj.ub_comdot(:,knot_idx) = ub;
    end
    
    function obj = addKinematicPolygon(obj,fsrc_idx,A,b)
      % add polygonal constraint A*[x1;y1;x2;y2;...;xN;yN]<=b on the contact bodies corresponding to
      % obj.fsrc_cnstr{fsrc_idx(1)}, obj.fsrc_cnstr{fsrc_idx(2)},...obj.fsrc_cnstr{fsrc_idx(N)}.
      % where [x1;y1] is the position of the body in obj.fsrc_cnstr{fsrc_idx(1)}, and so
      % on for [xi;yi]
      % @param fsrc_idx  An integer vector. The kinematic constraint is on the
      % bodies in obj.fsrc_cnstr{fsrc_idx(1)} obj.fsrc_cnstr{fsrc_idx(2)} and obj.fsrc_cnstr{fsrc_idx(N)}
      % @param A    A n x (2*length(fsrc_idx)) matrix.
      % @param b    A n x 1 vector
      if(~isnumeric(fsrc_idx))
        error('Drake:FixedFootYawCoMPlanningPosition:fsrc_idx1 and fsrc_idx2 should be numeric scalar');
      end
      num_fsrc = numel(fsrc_idx);
      sizecheck(fsrc_idx,[1,num_fsrc]);
      if(~isnumeric(A) || ~isnumeric(b))
        error('Drake:FixedFootYawCoMPlanningPosition:A and b should be numeric');
      end
      num_cnstr = numel(b);
      sizecheck(A,[num_cnstr,2*num_fsrc]);
      sizecheck(b,[num_cnstr,1]);
      iA = reshape(bsxfun(@times,(1:num_cnstr)',ones(1,2*num_fsrc)),[],1);
      jA = reshape(bsxfun(@times,reshape(obj.fsrc_body_pos_idx(:,fsrc_idx),1,[]),ones(num_cnstr,1)),[],1);
      obj.A_kin = [obj.A_kin;sparse(iA,jA,A(:),num_cnstr,obj.num_vars)];
      obj.b_kin = [obj.b_kin;b];
    end
    
    function checkSolution(obj,com,comp,compp,foot_pos,F,sdotsquare,Hdot,Hbar,epsilon)
      delta_s = 1/(obj.nT-1);
      valuecheck(diff(com,1,2)-comp(:,2:end)*delta_s,0,1e-4);
      valuecheck(diff(comp,1,2)-compp(:,2:end)*delta_s,0,1e-4);
      if(any(sdotsquare<0))
        error('sdotsquare cannot be negative');
      end
      if(any(sdotsquare>obj.sdot_max^2))
        error('sdot is larger than sdot_max');
      end
      % check the wrench
      sdot_diff = diff(sdotsquare);
      sdot_diff = [sdot_diff sdot_diff(end)];
      foot_contact_pts_pos = cell(1,obj.num_fsrc_cnstr);
      for i = 1:obj.num_fsrc_cnstr
        foot_contact_pts_pos{i} = bsxfun(@times,obj.A_xy(:,:,i)*foot_pos(:,i)+obj.b_xy(:,:,i),...
          ones(1,obj.fsrc_cnstr{i}.num_contact_pts))+obj.rotmat(:,:,i)*obj.fsrc_cnstr{i}.body_contact_pts;
      end
      for i = 1:obj.nT
        F_i = zeros(3,1);
        tau_i = zeros(3,1);
        for j = 1:length(obj.F_idx{i})
          fsrc_idx = obj.F2fsrc_map{i}(j);
          F_ij = obj.A_force{fsrc_idx}*F{i}{j};
          F_i = F_i+sum(F_ij,2);
          foot_contact_pts_CoM = foot_contact_pts_pos{fsrc_idx}-bsxfun(@times,com(:,i),ones(1,obj.fsrc_cnstr{fsrc_idx}.num_contact_pts));
          tau_i = tau_i+sum(cross(foot_contact_pts_CoM,F_ij),2);
        end
        F_i(3) = F_i(3)-obj.robot_mass*obj.g;
        mcomddot = obj.robot_mass*(compp(:,i)*sdotsquare(i)+comp(:,i)*2/delta_s*sdot_diff(i));
        valuecheck(F_i,mcomddot,1e-4);
        valuecheck(tau_i,Hdot(:,i),1e-4);
        valuecheck(diff(Hbar,1,2),Hdot(:,2:end)*delta_s,1e-4);
        valuecheck(Hdot,obj.lambda*Hbar+epsilon,1e-4);
        sdot = sqrt(sdotsquare);
        if(any(2*delta_s*ones(1,obj.nT-1)./sum([sdot(1:end-1);sdot(2:end)],1)>obj.dt_max+1e-6))
          error('dt is above dt_max');
        end
      end
    end
  end
end