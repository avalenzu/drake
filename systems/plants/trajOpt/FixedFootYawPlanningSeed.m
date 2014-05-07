classdef FixedFootYawPlanningSeed < NonlinearProgramWConstraintObjects
  % Find the contact force that will give the desired center of mass. This is used to find
  % the initial seed
  properties(SetAccess = protected)
    robot_mass % The mass of the robot
    t_knot % The time knot for planning
    g % The gravitational acceleration
    nT % The length of obj.t_knot
    lambda % A 3 x 3 Hurwitz matrix. This is used in the PD law on angular momentum.
    com_idx; % A 3 x obj.nT matrix. x(com_idx(:,i)) is the CoM position at time t_knot(i) in the decision variable x
    comp_idx; % A 3 x obj.nT matrix. x(comp_idx(:,i)) is the first derivative of CoM w.r.t time scaling function s a i'th knot point
    compp_idx; % A 3 x obj.nT matrix. x(compp_idx(:,i)) is the second derivative of CoM w.r.t time scaling function s a i'th knot point
    margin_idx % A 1 x obj.nT vector. x(margin_idx(i)) is the force margin at time t_knot(i). We want to maximize this margin for better robustness
    F_idx % A cell array. x(F_idx{i}{j}(:,k)) is the contact force parameter (the weights for the friction cone extreme rays) at time t_knot(i), for the j'th FootStepRegionContactConstraint, at k'th contact point
    
    iQ_cost % The row indices of non-zero entries in the quadratic cost
    jQ_cost % The column indices of non-zero entries in the quadratic cost
    Qval_cost % The value of the non-zero entries in the quadratic cost
    f_cost % A obj.num_vars x 1 vector. The linear componenet of the cost
    A_iris,b_iris % A_iris * x <= b_iris is the union of all half space constraint on the foot location from iris
    A_com,A_com_bnd % A_com * x = A_com_bnd is the constraint on the euler integration of CoM
    num_fsrc_cnstr % An integer. The total number of FootStepRegionContactConstraint
    fsrc_cnstr % A cell array. All the FootStepRegionContactConstraint object
    fsrc_body_pos_idx % A 2 x length(fsrc_cnstr) matrix. x(obj.fsrc_body_pos_idx(:,i)) is the body position for the i'th FootStepRegionContactConstraint in the decision variables.
    F2fsrc_map % A cell arry. obj.fsrc_cnstr{F2fsrc_map{i}(j)} is the FootStepContactRegionConstraint corresponds to the force x(obj.F_idx{i}{j})
    fsrc_knot_active_idx % A cell array. fsrc_knot_active_idx{i} is the indices of the knots that are active for i'th FootStepRegionContactConstraint
    yaw % A 1 x num_fsrc_cnstr double vector. yaw(i) is the yaw angle for obj.fsrc_cnstr{i}
    A_kin,b_kin  % A_kin*x<=b_kin encodes the kinematic constraint on the contact points and CoM
    lb_comdot,ub_comdot % lb_comdot and ub_comdot are the lower and upper bound of the CoM velocities respectively
    A_newton % A 3*obj.nT x obj.num_vars matrix. force(:) = A_newton*x, where force(:,i) is the total force at i'th knot point
    A_margin % A obj.nT x obj.num_vars matrix. 
    A_margin_bnd % A obj.nT x 1 vector. A_margin*x<=A_margin_bnd represents margin(i)<= min(F_i), where F_i are all the force weights at the k'th knot point
    
    dt_max %  A positive scalar. The upperbound for the time interval between any two consecutive knot points
    sdot_max % A positive scalar. The upper bound for the derivitive of time scaling funtion s w.r.t time.
    com_cone_idx % A n x 4 matrix. com_cone_idx(i,:) is the index of the i'th cone for the distance between CoM and contact body
    
  end
  
  properties(Access = protected)
    A_force % A cell array.  A_force{i} = obj.fsrc_cnstr{i}.force, which is a 3 x obj.fsrc_cnstr[i}.num_edges matrix
    A_xy,b_xy,rotmat  % A_xy is 3 x 2 x obj.num_fsrc_cnstr matrix. b_xy is 3 x 1 x obj.num_fsrc_cnstr matrix. rotmat is 3 x 3 x obj.num_fsrc_cnstr matrix. [rotmat(:,:,i),A_xy(:,:,i),b_xy(:,:,i)] = obj.fsrc_cnstr{i}.bodyTransform(obj.yaw(i)); 
    num_force_weight % A scalar. The total number of force weights.
  end
  
  methods
    function obj = FixedFootYawPlanningSeed(robot_mass,t,g,lambda,c_margin,dt_max,sdot_max,Q_comddot,fsrc_cnstr,yaw,F2fsrc_map,fsrc_knot_active_idx,A_force,A_xy,b_xy,rotmat)
      obj = obj@NonlinearProgramWConstraintObjects(0);
      obj.robot_mass = robot_mass;
      obj.t_knot = t;
      obj.nT = length(obj.t_knot);
      obj.g = g;
      obj.lambda = lambda;
      obj.dt_max = dt_max;
      obj.sdot_max = sdot_max;
      obj.num_fsrc_cnstr = length(fsrc_cnstr);
      obj.fsrc_cnstr = fsrc_cnstr;
      obj.fsrc_knot_active_idx = fsrc_knot_active_idx;
      obj.yaw = yaw;
      obj.A_xy = A_xy;
      obj.b_xy = b_xy;
      obj.rotmat = rotmat;
      obj.F2fsrc_map = F2fsrc_map;
      obj.A_force = A_force;
      
     
      obj.com_idx = reshape(obj.num_vars+(1:3*obj.nT),3,obj.nT);
      com_names = cell(3*obj.nT,1);
      for i = 1:obj.nT
        com_names((i-1)*3+(1:3)) = {sprintf('com_x[%d]',i);sprintf('com_y[%d]',i);sprintf('com_z[%d]',i)};
      end
      obj = obj.addDecisionVariable(3*obj.nT,com_names);
      
      obj.comp_idx = reshape(obj.num_vars+(1:3*obj.nT),3,obj.nT);
      comp_names = cell(3*obj.nT,1);
      for i = 1:obj.nT
        comp_names((i-1)*3+(1:3)) = {sprintf('comp_x[%d]',i);sprintf('comp_y[%d]',i);sprintf('comp_z[%d]',i)};
      end
      obj = obj.addDecisionVariable(3*obj.nT,comp_names);
      obj.compp_idx = reshape(obj.num_vars+(1:3*obj.nT),3,obj.nT);
      compp_names = cell(3*obj.nT,1);
      for i = 1:obj.nT
        compp_names((i-1)*3+(1:3)) = {sprintf('compp_x[%d]',i);sprintf('compp_y[%d]',i);sprintf('compp_z[%d]',i)};
      end
      obj = obj.addDecisionVariable(3*obj.nT,compp_names);
      obj.margin_idx = obj.num_vars+(1:obj.nT);
      margin_names = cell(obj.nT,1);
      for i = 1:obj.nT
        margin_names{i} = sprintf('force_margin[%d]',i);
      end
      obj = obj.addDecisionVariable(obj.nT,margin_names);
      
      obj.num_fsrc_cnstr = length(fsrc_cnstr);
      obj.fsrc_body_pos_idx = zeros(2,obj.num_fsrc_cnstr);
      obj.F_idx = cell(1,obj.nT);
      iA_iris = [];
      jA_iris = [];
      Aval_iris = [];
      obj.b_iris = [];
      num_halfspace_iris = 0;
      obj.num_force_weight = 0;
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
        foot_pos_names = {sprintf('Foot x position for %d''th FootStepRegionContactConstraint',i);...
          sprintf('Foot y position for %d''th FootStepRegionContactConstraint',i)};
        obj = obj.addDecisionVariable(2,foot_pos_names);
        for j = 1:obj.nT
          if(obj.fsrc_cnstr{i}.foot_step_region_cnstr.isTimeValid(obj.t_knot(j)))
            obj.F_idx{j} = [obj.F_idx{j} {obj.num_vars+reshape((1:obj.fsrc_cnstr{i}.num_force_weight),obj.fsrc_cnstr{i}.num_edges,obj.fsrc_cnstr{i}.num_contact_pts)}];
            F_names = cell(obj.fsrc_cnstr{i}.num_force_weight,1);
            for k = 1:obj.fsrc_cnstr{i}.num_contact_pts
              for l = 1:obj.fsrc_cnstr{i}.num_edges
                F_names{(k-1)*obj.fsrc_cnstr{i}.num_edges+l} = sprintf('fsrc[%d] pt %d weight %d at %d knot',i,k,l,j);
              end
            end
            obj = obj.addDecisionVariable(obj.fsrc_cnstr{i}.num_force_weight,F_names);
            obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(zeros(obj.fsrc_cnstr{i}.num_force_weight,1),inf(obj.fsrc_cnstr{i}.num_force_weight,1)),reshape(obj.F_idx{j}{end},[],1));
            obj.num_force_weight = obj.num_force_weight+obj.fsrc_cnstr{i}.num_force_weight;
          end
        end
      end
      obj.A_iris = sparse(iA_iris,jA_iris,Aval_iris,num_halfspace_iris,obj.num_vars);
      
      delta_s = 1/(obj.nT-1);
      % linear constraint that com[n]-com[n-1] = comp[n]*delta_s and comp[n]-comp[n-1] =
      % compp[n]*delta_s
      iAcom = [(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';];
      jAcom = [reshape(obj.com_idx(:,2:end),[],1);reshape(obj.com_idx(:,1:end-1),[],1); reshape(obj.comp_idx(:,2:end),[],1)];
      Aval_com = [ones(3*(obj.nT-1),1);-ones(3*(obj.nT-1),1);-reshape(bsxfun(@times,ones(3,1),delta_s*ones(1,obj.nT-1)),[],1)];
      iAcom = [iAcom;3*(obj.nT-1)+iAcom];
      jAcom = [jAcom;reshape(obj.comp_idx(:,2:end),[],1);reshape(obj.comp_idx(:,1:end-1),[],1); reshape(obj.compp_idx(:,2:end),[],1)];
      Aval_com = [Aval_com;Aval_com];
      obj.A_com = sparse(iAcom,jAcom,Aval_com,6*(obj.nT-1),obj.num_vars);
      obj.A_com_bnd = zeros(6*(obj.nT-1),1);
      obj = obj.addLinearConstraint(LinearConstraint(obj.A_com_bnd,obj.A_com_bnd,obj.A_com));
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
      obj = obj.addLinearConstraint(LinearConstraint(-inf(force_weight_count,1),obj.A_margin_bnd,obj.A_margin));
      % The kinematic constraint on the contact bodies will be set through function
      % addKinematicPolygon
      obj.A_kin = [];
      obj.b_kin = [];
      obj.lb_comdot = -inf(3,obj.nT);
      obj.ub_comdot = inf(3,obj.nT);
      
      obj.iQ_cost = reshape(repmat(obj.compp_idx,3,1),[],1);
      obj.jQ_cost = reshape(bsxfun(@times,obj.compp_idx(:)',ones(3,1)),[],1);
      obj.Qval_cost = reshape(repmat(Q_comddot,1,obj.nT),[],1);
      obj.f_cost = zeros(obj.num_vars,1);
      obj.f_cost(obj.margin_idx(:)) = -c_margin;
    end
    
    function [com,comp,compp,foot_pos,F,sdotsquare] = solve(obj,sdot0)
      % @param sdot0   A 1 x obj.nT vector
      sdotsquare = sdot0.^2;
      sdotsquare_diff = diff(sdotsquare);
      sdotsquare_diff = [sdotsquare_diff sdotsquare_diff(end)];
      A_compp = obj.A_newton;
      A_compp_bnd = reshape(bsxfun(@times,[0;0;obj.robot_mass*obj.g],ones(1,obj.nT)),[],1);
      for i = 1:obj.nT
        A_compp((i-1)*3+(1:3),obj.compp_idx(:,i)) = -obj.robot_mass*sdotsquare(i)*eye(3);
        A_compp((i-1)*3+(1:3),obj.comp_idx(:,i)) = -obj.robot_mass*(obj.nT-1)/2*sdotsquare_diff(i)*eye(3);
      end
      obj = obj.addLinearConstraint(LinearConstraint(A_compp_bnd,A_compp_bnd,A_compp));
      comp_lb = reshape(obj.lb_comdot./bsxfun(@times,ones(3,1),sqrt(sdotsquare)),[],1);
      comp_ub = reshape(obj.ub_comdot./bsxfun(@times,ones(3,1),sqrt(sdotsquare)),[],1);
      obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(comp_lb,comp_ub),obj.comp_idx(:));
      model.A = sparse([obj.Ain;obj.Aeq]);
      model.rhs = [obj.bin;obj.beq];
      max_row_entry = max(abs(model.A),[],2);
      model.A = sparse(model.A./bsxfun(@times,max_row_entry,ones(1,obj.num_vars)));
      model.rhs = model.rhs./max_row_entry;
      model.sense = [repmat('<',length(obj.bin),1);repmat('=',length(obj.beq),1)];
      model.Q = sparse(obj.iQ_cost,obj.jQ_cost,obj.Qval_cost,obj.num_vars,obj.num_vars);
      model.obj = obj.f_cost;
      model.lb = obj.x_lb;
      model.ub = obj.x_ub;
      if(~isempty(obj.com_cone_idx))
        for i = 1:size(obj.com_cone_idx,1)
          model.cones(i).index = obj.com_cone_idx(i,:);
        end
      end
      params = struct('OutputFlag',false,'BarHomogeneous',1,'Threads',4);
      
      result = gurobi(model,params);
      if(strcmp(result.status,'OPTIMAL'))
        com = reshape(result.x(obj.com_idx),3,obj.nT);
        comp = reshape(result.x(obj.comp_idx),3,obj.nT);
        compp = reshape(result.x(obj.compp_idx),3,obj.nT);
        foot_pos = reshape(result.x(obj.fsrc_body_pos_idx),2,obj.num_fsrc_cnstr);
        F = cell(1,obj.nT);
        for i = 1:obj.nT
          for j = 1:length(obj.F2fsrc_map{i})
            fsrc_idx = obj.F2fsrc_map{i}(j);
            F{i} = [F{i},{reshape(result.x(obj.F_idx{i}{j}),obj.fsrc_cnstr{fsrc_idx}.num_edges,obj.fsrc_cnstr{fsrc_idx}.num_contact_pts)}];
          end
        end
      else
        error('Initial seed is invalid. Consider to change the FootStepRegionContactConstraint objects, or adjust the sdot');
      end
    end
    
    function [Hbar,Hdot,sigma,epsilon] = angularMomentum(obj,com,foot_pos,F,H0)
      % Compute the angular momentum given the com trajectory, foot_pos, force and initial
      % angular momentum
      % @param com    A 3 x obj.nT matrix. The CoM position
      % @param foot_pos   A 2 x obj.num_fsrc_cnstr matrix.
      % @param F     A 1 x obj.nT cell, F{i}{j} is the force weights for
      % obj.fsrc_cnstr{i}{j}
      % @param H0    A 3 x 1 vector. The initial angular momentum
      foot_contact_pos = cell(1,obj.num_fsrc_cnstr);
      for i = 1:obj.num_fsrc_cnstr
        foot_contact_pos{i} = bsxfun(@times,ones(1,obj.fsrc_cnstr{i}.num_contact_pts),...
          obj.A_xy(:,:,i)*foot_pos(:,i)+obj.b_xy(:,:,i))+obj.rotmat(:,:,i)*obj.fsrc_cnstr{i}.body_contact_pts;
      end
      Hdot = zeros(3,obj.nT);
      for i = 1:obj.nT
        for j = 1:length(obj.F2fsrc_map{i})
          fsrc_idx = obj.F2fsrc_map{i}(j);
          force = obj.A_force{fsrc_idx}*F{i}{j};
          Hdot(:,i) = Hdot(:,i)+sum(cross(foot_contact_pos{fsrc_idx}-bsxfun(@times,com(:,i),ones(1,obj.fsrc_cnstr{fsrc_idx}.num_contact_pts)),force),2);
        end
      end
      delta_s = 1/(obj.nT-1);
      Hbar = bsxfun(@times,H0,ones(1,obj.nT));
      Hbar = Hbar+[zeros(3,1) cumsum(Hdot(:,2:end)*delta_s,2)];
      epsilon = Hdot-obj.lambda*Hbar;
      sigma = sum(sum(epsilon.^2));
    end
    
    function obj = addCoMFootDistanceConstraint(obj,fsrc_idx,dist_ub)
      for i = 1:obj.fsrc_knot_active_idx{fsrc_idx}
        com_foot_idx = obj.num_vars+(1:3);
        dist_idx = obj.num_vars+4;
        obj = obj.addDecisionVariable(4);
        b = obj.b_xy(:,:,fsrc_idx)+obj.rotmat(:,:,fsrc_idx)*obj.fsrc_cnstr{fsrc_idx}.foot_step_region_cnstr.body_pt;
        obj = obj.addLinearConstraint(LinearConstraint(b,b, [eye(3) -obj.A_xy(:,:,fsrc_idx) -eye(3)]),[obj.com(:,obj.fsrc_knot_active_idx{fsrc_idx}(i));obj.fsrc_body_pos_idx(:,fsrc_idx);com_foot_idx]);
        obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(dist_ub,dist_ub),dist_idx);
        obj.com_cone_idx = [obj.com_cone_idx;dist_idx com_foot_idx];
      end
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
  end
end