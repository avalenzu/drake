classdef FixedFootYawCoMPlanningPosition < NonlinearProgramWConstraintObjects
  % 'P-step' of the alternative planning for CoM trajectory and force trajectory. This is
  % a SOCP problem for linearized friction cone.            
  properties(SetAccess = protected)
    robot_mass % The mass of the robot
    t_knot % The time knot for planning
    g % The gravitational acceleration
    nT % The length of obj.t_knot
    com_idx; % A 3 x obj.nT matrix. x(com_idx(:,i)) is the CoM position at time t_knot(i) in the decision variable x
    comdot_idx; % A 3 x obj.nT matrix. x(comdot_idx(:,i)) is the first derivative of CoM w.r.t time scaling function s a i'th knot point
    comddot_idx; % A 3 x obj.nT matrix. x(comddot_idx(:,i)) is the second derivative of CoM w.r.t time scaling function s a i'th knot point
    H_idx; %A 3 x obj.nT matrix. x(H_idx(:,i)) is the centroidal angular momentum at time t_knot(i)
    Hdot_idx % A 3 x obj.nT matrix. x(Hdot_idx(:,i)) is the rate of centroidal angular momentum at time t_knot(i)
    epsilon_idx % A 3 x obj.nT matrix. x(epsilon_idx(:,i)) is the residue of the PD law Hdot[i] = lambda*H[i]+epsilon[i]
    sigma_idx % An integer scalar. x(sigma_idx) is the index of sigma, which is the dummy variable used to bound the PD residue: sum_n epsilon[n]'*epsilon[n] <= sigma  
    
    A_iris,b_iris % A_iris * x <= b_iris is the union of all half space constraint on the foot location from iris
    A_com,A_com_bnd % A_com * x = A_com_bnd is the constraint on the euler integration of CoM
    A_H,A_H_bnd % A_H * x = A_H_bnd is the constraint on the euler integraton of angular momentum
    A_angular_PD,A_angular_PD_bnd % A_angular_PD * x = A_angular_PD_bnd is the constraint Hdot[n] = lambda*H[n]+epsilon[n]
    num_fsrc_cnstr % An integer. The total number of FootStepRegionContactConstraint
    fsrc_cnstr % An array of FootStepRegionContactConstraint object
    fsrc_body_pos_idx % A 2 x length(fsrc_cnstr) matrix. x(obj.fsrc_body_pos_idx(:,i)) is the body position for the i'th FootStepRegionContactConstraint in the decision variables.
    F2fsrc_map % A cell arry. obj..fsrc_cnstr{F2fsrc_map{i}(j)} is the FootStepContactRegionConstraint corresponds to the force x(obj.F_idx{i}{j})
    fsrc_knot_active_idx % A cell array. fsrc_knot_active_idx{i} is the indices of the knots that are active for i'th FootStepRegionContactConstraint
    yaw % A 1 x num_fsrc_cnstr double vector. yaw(i) is the yaw angle for obj.fsrc_cnstr(i)
    A_kin,b_kin  % A_kin*x<=b_kin encodes the kinematic constraint on the contact points and CoM
    
    lambda % A 3 x 3 Hurwitz matrix
    
    robot_dim  % The approximate dimension of the robot in meters. This is used to scale the constraint
    Q_comddot % A 3 x 3 PSD matrix. Penalize the quadratic cost on CoM acceleration
    Q_cost % The Hessian of the cost
    f_cost % The linear component of the cost
    com_traj_order % An integer. The order of the polynomial to interpolate CoM. Acceptable orders are cubic or quartic
  end
  
  properties(Access = protected)
    A_force % A cell array.  A_force{i} = obj.fsrc_cnstr(i).force, which is a 3 x obj.fsrc_cnstr(i).num_edges matrix
    A_xy,b_xy,rotmat  % A_xy is 3 x 2 x obj.num_fsrc_cnstr matrix. b_xy is 3 x 1 x obj.num_fsrc_cnstr matrix. rotmat is 3 x 3 x obj.num_fsrc_cnstr matrix. [rotmat(:,:,i),A_xy(:,:,i),b_xy(:,:,i)] = obj.fsrc_cnstr(i).bodyTransform(obj.yaw(i)); 
  end
  
  methods
    function obj = FixedFootYawCoMPlanningPosition(robot_mass,robot_dim,t,g,lambda,Q_comddot,fsrc_cnstr,yaw,F2fsrc_map,fsrc_knot_active_idx,A_force,A_xy,b_xy,rotmat,com_traj_order)
      % obj =
      % FixedFootYawCoMPlanningPosition(robot_mass,t,lambda,Q_comddot,foot_step_region_contact_cnstr1,yaw1,foot_step_region_contact_cnstr2,yaw2,...)
      % @param robot_mass    The mass of the robot
      % @param t             The time knot for planning. This indicates which
      % FootStepRegionContactConstraint is active at a given time knot. The actual time is
      % determined by the scaling function.
      % @param g             The gravitational acceleration
      % @param lambda        A 3 x 3 Hurwitz matrix. It tries to drive the angular
      % momentum stays at 0 by putting the constraint Hdot[n] = lambda*H[n]+epsilon[n]
      % @param Q_comddot     A 3 x 3 PSD matrix. The cost is sum_n
      % comddot[n]'*Q_comddot*comddot[n]+epsilon[n]'*epsilon[n]
      % @param fsrc_cnstr  An array of FootStepRegionContactConstraint object
      % @param yaw     A 1 x num_fsrc_cnstr double vector. yaw(i) is the yaw angle for
      % obj.fsrc_cnstr(i)
      % @param A_xy,b_xy,rotmat   A_xy is 3 x 2 x obj.num_fsrc_cnstr matrix. b_xy is 3 x 1 x obj.num_fsrc_cnstr matrix. rotmat is 3 x 3 x obj.num_fsrc_cnstr matrix. [rotmat(:,:,i),A_xy(:,:,i),b_xy(:,:,i)] = obj.fsrc_cnstr(i).bodyTransform(obj.yaw(i)); 
      % @param A_force    A_force{i} = obj.fsrc_cnstr(i).force, which is a 3 x obj.fsrc_cnstr(i).num_edges matrix
      obj = obj@NonlinearProgramWConstraintObjects(0);
      obj.robot_mass = robot_mass;
      obj.robot_dim = robot_dim;
      obj.t_knot = t;
      obj.nT = length(obj.t_knot);
      obj.g = g;
      obj.lambda = lambda;
      obj.Q_comddot = Q_comddot;
      obj.num_fsrc_cnstr = length(fsrc_cnstr);
      obj.fsrc_cnstr = fsrc_cnstr;
      obj.yaw = yaw;
      obj.A_xy = A_xy;
      obj.b_xy = b_xy;
      obj.rotmat = rotmat;
      obj.F2fsrc_map = F2fsrc_map;
      obj.fsrc_knot_active_idx = fsrc_knot_active_idx;
      obj.A_force = A_force;
      obj.com_traj_order = com_traj_order;
      
      obj.com_idx = reshape(obj.num_vars+(1:3*obj.nT),3,obj.nT);
      com_names = cell(3*obj.nT,1);
      for i = 1:obj.nT
        com_names((i-1)*3+(1:3)) = {sprintf('com_x[%d]',i);sprintf('com_y[%d]',i);sprintf('com_z[%d]',i)};
      end
      obj = obj.addDecisionVariable(3*obj.nT,com_names);
      
      obj.comdot_idx = reshape(obj.num_vars+(1:3*obj.nT),3,obj.nT);
      comp_names = cell(3*obj.nT,1);
      for i = 1:obj.nT
        comp_names((i-1)*3+(1:3)) = {sprintf('comp_x[%d]',i);sprintf('comp_y[%d]',i);sprintf('comp_z[%d]',i)};
      end
      obj = obj.addDecisionVariable(3*obj.nT,comp_names);
      
      obj.comddot_idx = reshape(obj.num_vars+(1:3*obj.nT),3,obj.nT);
      compp_names = cell(3*obj.nT,1);
      for i = 1:obj.nT
        compp_names((i-1)*3+(1:3)) = {sprintf('compp_x[%d]',i);sprintf('compp_y[%d]',i);sprintf('compp_z[%d]',i)};
      end
      obj = obj.addDecisionVariable(3*obj.nT,compp_names);
      
      obj.H_idx = reshape(obj.num_vars+(1:3*obj.nT),3,obj.nT);
      H_names = cell(3*obj.nT,1);
      for i = 1:obj.nT
        H_names((i-1)*3+(1:3)) = {sprintf('H_x[%d]',i);sprintf('H_y[%d]',i);sprintf('H_z[%d]',i)};
      end
      obj = obj.addDecisionVariable(3*obj.nT,H_names);
      
      obj.Hdot_idx = reshape(obj.num_vars+(1:3*obj.nT),3,obj.nT);
      Hdot_names = cell(3*obj.nT,1);
      for i = 1:obj.nT
        Hdot_names((i-1)*3+(1:3)) = {sprintf('Hdot_x[%d]',i);sprintf('Hdot_y[%d]',i);sprintf('Hdot_z[%d]',i)};
      end
      obj = obj.addDecisionVariable(3*obj.nT,Hdot_names);
      
      obj.epsilon_idx = reshape(obj.num_vars+(1:3*obj.nT),3,obj.nT);
      epsilon_names = cell(3*obj.nT,1);
      for i = 1:obj.nT
        epsilon_names((i-1)*3+(1:3)) = repmat({sprintf('epsilon[%d]',i)},3,1);
      end
      obj = obj.addDecisionVariable(3*obj.nT,epsilon_names);
      
      obj.sigma_idx = obj.num_vars+1;
      sigma_names = {'sigma'};
      obj = obj.addDecisionVariable(1,sigma_names);
      
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
        A_iris_i = obj.fsrc_cnstr(i).foot_step_region_cnstr.A;
        b_iris_i = obj.fsrc_cnstr(i).foot_step_region_cnstr.b;
        iA_iris_i = num_halfspace_iris+reshape([(1:size(A_iris_i,1))' (1:size(A_iris_i,1))'],[],1);
        jA_iris_i = [(obj.num_vars+1)*ones(size(A_iris_i,1),1);(obj.num_vars+2)*ones(size(A_iris_i,1),1)];
        Aval_iris_i = reshape(A_iris_i(:,1:2),[],1);
        iA_iris = [iA_iris;iA_iris_i];
        jA_iris = [jA_iris;jA_iris_i];
        Aval_iris = [Aval_iris;Aval_iris_i];
        obj.b_iris = [obj.b_iris;b_iris_i-obj.yaw(i)*A_iris_i(:,3)];
        num_halfspace_iris = num_halfspace_iris+size(A_iris_i,1);
        foot_pos_names = {sprintf('Foot x position for %d''th FootStepRegionContactConstraint',i);...
          sprintf('Foot y position for %d''th FootStepRegionContactConstraint',i)};
        obj = obj.addDecisionVariable(2,foot_pos_names);
      end
      obj.A_iris = sparse(iA_iris,jA_iris,Aval_iris,num_halfspace_iris,obj.num_vars);
      dt = diff(obj.t_knot);
      % linear constraint on the quatic interpolation of com
      % For quartic polynoial, dt^2(comddot[i+1]-comddot[i]) = -12(com[i+1]-com[i])+6*dt(comdot[i]+comdot[i+1])
      % For cubic polynomial, 6*(com[i+1]-com[i])-2*dt*comdot[i+1]-4*dt*comdot[i]-dt^2*comddot[i] = 0
      if(obj.com_traj_order == 4)
        iAcom = [(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';(1:3*(obj.nT-1))'];
        jAcom = [reshape(obj.com_idx(:,1:end-1),[],1);reshape(obj.com_idx(:,2:end),[],1);...
          reshape(obj.comdot_idx(:,1:end-1),[],1);reshape(obj.comdot_idx(:,2:end),[],1);...
          reshape(obj.comddot_idx(:,1:end-1),[],1);reshape(obj.comddot_idx(:,2:end),[],1)];
        Aval_com = [12*ones(3*(obj.nT-1),1);-12*ones(3*(obj.nT-1),1);reshape(bsxfun(@times,6*dt,ones(3,1)),[],1);...
          reshape(bsxfun(@times,6*dt,ones(3,1)),[],1);reshape(bsxfun(@times,dt.^2,ones(3,1)),[],1);reshape(bsxfun(@times,-dt.^2,ones(3,1)),[],1)];
      elseif(obj.com_traj_order == 3)
        iAcom = [(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';(1:3*(obj.nT-1))'];
        jAcom = [reshape(obj.com_idx(:,1:end-1),[],1);reshape(obj.com_idx(:,2:end),[],1);...
          reshape(obj.comdot_idx(:,1:end-1),[],1);reshape(obj.comdot_idx(:,2:end),[],1);reshape(obj.comddot_idx(:,1:end-1),[],1)];
        Aval_com = [-6*ones(3*(obj.nT-1),1);6*ones(3*(obj.nT-1),1);reshape(bsxfun(@times, -4*dt,ones(3,1)),[],1);...
          reshape(bsxfun(@times,-2*dt,ones(3,1)),[],1);reshape(bsxfun(@times,-dt.^2,ones(3,1)),[],1)];
      end
      obj.A_com = sparse(iAcom,jAcom,Aval_com,3*(obj.nT-1),obj.num_vars);
      obj.A_com_bnd = zeros(3*(obj.nT-1),1);
      A_com_name = repmat({'com difference'},3*(obj.nT-1),1);
      obj = obj.addLinearConstraint(LinearConstraint(obj.A_com_bnd,obj.A_com_bnd,obj.A_com),(1:obj.num_vars),A_com_name);
      % linear constraint that H[n]-H[n-1] = Hdot[n]*dt
      iA_H = [(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';];
      jA_H = [reshape(obj.H_idx(:,2:end),[],1);reshape(obj.H_idx(:,1:end-1),[],1); reshape(obj.Hdot_idx(:,2:end),[],1)];
      Aval_H = [ones(3*(obj.nT-1),1);-ones(3*(obj.nT-1),1);reshape(bsxfun(@times,-dt,ones(3,1)),[],1)];
      obj.A_H = sparse(iA_H,jA_H,Aval_H,3*(obj.nT-1),obj.num_vars);
      obj.A_H_bnd = zeros(3*(obj.nT-1),1);
      A_H_names = repmat({'H difference'},3*(obj.nT-1),1);
      obj = obj.addLinearConstraint(LinearConstraint(obj.A_H_bnd,obj.A_H_bnd,obj.A_H),(1:obj.num_vars),A_H_names);
      % linear constraint that Hdot[n] = lambda*H[n]+epsilon[n]
      iA_angular_PD = [(1:3*obj.nT)';reshape(repmat(reshape((1:3*obj.nT),3,obj.nT),3,1),[],1);(1:3*obj.nT)'];
      jA_angular_PD = [obj.Hdot_idx(:);reshape(bsxfun(@times,obj.H_idx(:)',ones(3,1)),[],1);obj.epsilon_idx(:)];
      Aval_angular_PD = [ones(3*obj.nT,1);reshape(repmat(-obj.lambda,1,obj.nT),[],1);-ones(3*obj.nT,1)];
      obj.A_angular_PD = sparse(iA_angular_PD,jA_angular_PD,Aval_angular_PD,3*obj.nT,obj.num_vars);
      obj.A_angular_PD_bnd = zeros(3*obj.nT,1);
      A_angular_PD_names = repmat({'PD law of Hdot'},3*obj.nT,1);
      obj = obj.addLinearConstraint(LinearConstraint(obj.A_angular_PD_bnd,obj.A_angular_PD_bnd,obj.A_angular_PD),(1:obj.num_vars),A_angular_PD_names);
      
      % The kinematic constraint on the contact bodies will be set through function
      % addKinematicPolygon
      obj.A_kin = [];
      obj.b_kin = [];
      
      obj.solver = 'gurobi';
      Q_comddot_row = reshape(repmat(obj.comddot_idx,3,1),[],1);
      Q_comddot_col = reshape(bsxfun(@times,ones(3,1),obj.comddot_idx(:)'),[],1);
      Q_comddot_val = reshape(bsxfun(@times,ones(1,obj.nT),Q_comddot(:)),[],1);
      obj.Q_cost = sparse(Q_comddot_row,Q_comddot_col,Q_comddot_val,obj.num_vars,obj.num_vars);
      obj.Q_cost = obj.Q_cost+sparse(obj.epsilon_idx(:),obj.epsilon_idx(:),ones(3*obj.nT,1),obj.num_vars,obj.num_vars);
      obj.f_cost = zeros(obj.num_vars,1);
    end
    
    function [com,comdot,comddot,foot_pos,Hdot,H,sigma,epsilon,INFO] = solve(obj,F,sigma)
      % @param F     A cell array. F{i}{j} is the force parameters for
      % obj.fsrc_cnstr(obj.F2fsrc_map{i}(j))
      % @param sigma   A scalar. The upper bound for sum_n
      % epsilon[n]'*epsilon[n]
      % @param sdotsquare  A 1 x obj.nT vector. sdotsquare(i) is the square of the time
      % scaling function at time obj.t_knot(i)
      % @retval sigma   A scalar. The updated value for sum_n epsilon[n]'*epsilon[n]
      % @retval com   A 3 x obj.nT matrix. com(:,i) is the position of the robot at the
      % i'th knot point
      % @retval comdot  A 3 x obj.nT matrix. comdot(:,i) is the first derivative of com w.r.t
      % scaling function s at i'th knot point
      % @retval comddot  A 3 x obj.nT matrix. comddot(:,i) is the second derivative of com
      % w.r.t scaling function s at i'th knot point
      % @retval foot_pos   A 2 x obj.num_fsrc_cnstr matrix. foot_pos(:,i) is the xy
      % position of the contact body in obj.fsrc_cnstr(i)
      % @retval epsilon   A 3 x obj.nT matrix. The residue of the PD law on angular
      % momentum.
      % @retval INFO   1 for success, 0 for failure
      if(~iscell(F) || length(F) ~= obj.nT)
        error('Drake:FixedFootYawCoMPlanningPosition:F should be a cell of length obj.nT');
      end
      % A_angular*x=A_angular_bnd enforce the constraint Hdot*(obj.g*obj.robot_mass)-(contact_pt_pos-com)xF=0
      A_angular = zeros(3*obj.nT,obj.num_vars);
      A_angular_bnd = zeros(3*obj.nT,1);
      comddot_bnd = zeros(3*obj.nT,1);
      for i = 1:obj.nT
        F_i = zeros(3,1);
        A_angular((i-1)*3+(1:3),obj.Hdot_idx(:,i)) = eye(3)*obj.robot_mass*obj.g*obj.robot_dim;
        for j = 1:length(obj.F2fsrc_map{i})
          fsrc_idx = obj.F2fsrc_map{i}(j);
          num_contact_pts_ij = obj.fsrc_cnstr(fsrc_idx).num_contact_pts;
          sizecheck(F{i}{j},[obj.fsrc_cnstr(fsrc_idx).num_edges,num_contact_pts_ij]);
          F_ij = obj.A_force{fsrc_idx}*F{i}{j}; % F_ij(:,k) is the contact force at the k'th corner of the foot.
          F_i = F_i+sum(F_ij,2);
          A_angular((i-1)*3+(1:3),obj.fsrc_body_pos_idx(:,fsrc_idx)) = ...
            [sum(cross(F_ij,bsxfun(@times,obj.A_xy(:,1,fsrc_idx),ones(1,num_contact_pts_ij))),2) sum(cross(F_ij,bsxfun(@times,obj.A_xy(:,2,fsrc_idx),ones(1,num_contact_pts_ij))),2)];
          A_angular_bnd((i-1)*3+(1:3)) = A_angular_bnd((i-1)*3+(1:3))-sum(cross(F_ij,bsxfun(@times,obj.b_xy(:,:,fsrc_idx),ones(1,num_contact_pts_ij))+obj.rotmat(:,:,fsrc_idx)*obj.fsrc_cnstr(fsrc_idx).body_contact_pts),2);
        end
        A_angular((i-1)*3+(1:3),obj.com_idx(:,i)) = -[0 -F_i(3) F_i(2);F_i(3) 0 -F_i(1);-F_i(2) F_i(1) 0];
        comddot_bnd((i-1)*3+(1:3)) = (F_i-[0;0;obj.robot_mass*obj.g])/(obj.robot_mass);
      end
      obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(comddot_bnd,comddot_bnd),obj.comddot_idx(:));
      A_angular = A_angular/(obj.robot_mass*obj.g*obj.robot_dim);
      A_angular_bnd = A_angular_bnd/(obj.robot_mass*obj.g*obj.robot_dim);
      A_angular_names = repmat({'angular momentum'},3*obj.nT,1);
      obj = obj.addLinearConstraint(LinearConstraint(A_angular_bnd,A_angular_bnd,A_angular),(1:obj.num_vars),A_angular_names);

      model.A = sparse([obj.Ain;obj.Aeq]);
      model.rhs = [obj.bin;obj.beq];
      % normalize the A matrix and rhs, so that the maximum entry in each row is either 1
      % or -1
%         max_row_entry = max(abs(model.A),[],2);
%         model.A = sparse(model.A./bsxfun(@times,max_row_entry,ones(1,obj.num_vars)));
%         model.rhs = model.rhs./max_row_entry;
      model.sense = [repmat('<',length(obj.bin),1);repmat('=',length(obj.beq),1)];
      model.Q = obj.Q_cost;
      model.obj = obj.f_cost;
      obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(sqrt(sigma),sqrt(sigma)),obj.sigma_idx);

      model.lb = obj.x_lb;
      model.ub = obj.x_ub;
      model.cones.index = [obj.sigma_idx obj.epsilon_idx(:)'];
      params = struct('OutputFlag',false);

      result = gurobi(model,params);
      if(strcmp(result.status,'OPTIMAL')||strcmp(result.status,'SUBOPTIMAL'))
        % back off the solution a little bit. Add the conic constraint
        % x'Qx+f'x<=backoff_factor*objective

        model_backoff.A = sparse([obj.Ain;obj.Aeq]);
        model_backoff.rhs = [obj.bin;obj.beq];
        model_backoff.sense = [repmat('<',length(obj.bin),1);repmat('=',length(obj.beq),1)];
        model_backoff.obj = zeros(obj.num_vars,1);
        model_backoff.lb = obj.x_lb;
        model_backoff.ub = obj.x_ub;
        model_backoff.cones.index = [obj.sigma_idx obj.epsilon_idx(:)'];
        backoff_factor = 0.02;
        model_backoff.quadcon.Qc = model.Q;
        model_backoff.quadcon.q = model.obj;
        model_backoff.quadcon.rhs = result.objval*(1+sign(result.objval)*backoff_factor);
        params = struct('OutputFlag',false);

        result = gurobi(model_backoff,params);
        if(strcmp(result.status,'OPTIMAL')||strcmp(result.status,'SUBOPTIMAL'))
          com = reshape(result.x(obj.com_idx(:)),3,obj.nT);
          comdot = reshape(result.x(obj.comdot_idx(:)),3,obj.nT);
          comddot = reshape(result.x(obj.comddot_idx(:)),3,obj.nT);
          H_scale = obj.robot_mass*obj.robot_dim*obj.g;
          Hdot = reshape(result.x(obj.Hdot_idx(:)),3,obj.nT)*H_scale;
          H = reshape(result.x(obj.H_idx(:)),3,obj.nT)*H_scale;
          epsilon = reshape(result.x(obj.epsilon_idx(:)),3,obj.nT);
          sigma = sum(epsilon(:).^2);
          foot_pos = reshape(result.x(obj.fsrc_body_pos_idx),2,[]);
          INFO = 1;
        else
          INFO = 0;
%             error('Backoff should always be feasible');
        end
      elseif(~strcmp(result.status,'OPTIMAL'))
        INFO = 0;
%           error('P-step infesible');
      end
      if(INFO == 0)
        com = [];
        comdot = [];
        comddot = [];
        foot_pos = [];
        Hdot = [];
        H = [];
        epsilon = [];
        sigma = [];
      end
    end
    
    function obj = addQuadraticCost(obj,H,f,xind)
      % @param H   The Hessian of the quadratic cost
      % @param f   The linear component of the quadratic cost
      % xind  The indices of the variables used in evaluating the cost
      xind = xind(:);
      num_x = length(xind);
      sizecheck(H,[num_x,num_x]);
      sizecheck(f,[num_x,1]);
      obj.Q_cost = obj.Q_cost+sparse(xind,xind,H(:),obj.num_vars,obj.num_vars);
      obj.f_cost(xind) = obj.f_cost(xindt)+f;
    end
  end
end