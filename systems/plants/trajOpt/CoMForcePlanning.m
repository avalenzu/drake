classdef CoMForcePlanning < NonlinearProgramWConstraintObjects
  % This class tends to find the CoM trajectory and contact forces, so as to minimize the
  % angular momentum around the CoM.
  properties(SetAccess = protected)
    robot_mass % The mass of the robot
    g % The gravitational acceleration
    t_knot % The knot points of planning. This is only used to determine which FootStepRegionContactConstraint is active at a specific knot point
    nT % The total number of knot points
    com_idx  % A 3 x obj.nT matrix. x(com_idx(:,i)) is the CoM position at i'th knot point
    comp_idx  % A 3 x obj.nT matrix. x(comp_idx(:,i)) is the first derivative of CoM position w.r.t time scaling function s at i'th knot point
    compp_idx  % A 3 x obj.nT matrix. x(compp_idx(:,i)) is the second derivative of CoM position w.r.t time scaling function s at i'th knot point
    sdot_idx % A 1 x obj.nT vector. x(sdot_idx(i)) is the square of the derivitative of time scaling function s at i'th knot point
    H_idx; %A 3 x obj.nT matrix. x(H_idx(:,i)) is the centroidal angular momentum at time t_knot(i)
    Hdot_idx % A 3 x obj.nT matrix. x(Hdot_idx(:,i)) is the rate of centroidal angular momentum at time t_knot(i)
    margin_idx % A 1 x obj.nT vector. x(margin_idx(i)) is the force margin at i'th knot point
    F_idx % A cell array. x(F_idx{i}{j}(:,k)) is the contact force parameter (the weights for the friction cone extreme rays) at time t_knot(i), for the j'th FootStepRegionContactConstraint, at k'th contact point
    num_fsrc_cnstr % An integer. The total number of FootStepRegionContactConstraint
    fsrc_cnstr % A cell array. All the FootStepRegionContactConstraint object
    fsrc_body_pos_idx % A 2 x length(fsrc_cnstr) matrix. x(obj.fsrc_body_pos_idx(:,i)) is the body position for the i'th FootStepRegionContactConstraint in the decision variables.
    yaw_idx % A 1 x num_fsrc_cnstr vector. x(obj.yaw_idx(i)) is the yaw angle of the contact body in obj.fsrc_cnstr{i}
    F2fsrc_map % A cell array. obj..fsrc_cnstr{F2fsrc_map{i}(j)} is the FootStepContactRegionConstraint corresponds to the force x(obj.F_idx{i}{j})
    fsrc_knot_active_idx % A cell array. fsrc_knot_active_idx{i} is the indices of the knots that are active for i'th FootStepRegionContactConstraint
  end
  
  
  methods
    function obj = CoMForcePlanning(robot_mass,t,lambda,c_margin,dt_max,sdot_max,Q_comddot,fsrc_cnstr)
      % @properties robot_mass    The mass of the robot
      % @param t             The time knot for planning. This indicates which
      % FootStepRegionContactConstraint is active at a given time knot. The actual time is
      % determined by the scaling function.
      % @param lambda     A 3 x 3 Hurwitz matrix. This is used in the PD law on angular
      % momentum
      % @param c_margin   A non-negative scalar. The weight of the force margin
      % penalization.
      % @param dt_max     A positive scalar. The upper bound for the time duration between
      % two consecutive knot points
      % @param sdot_max  A positive scalar. The upper bound for the derivitive of time scaling funtion s
      % w.r.t time.
      % @param fsrc_cnstr     A cell of FootStepRegionContactConstraint object
      obj = obj@NonlinearProgramWConstraintObjects(0);
      if(~isnumeric(robot_mass))
        error('Drake:CoMForcePlanning:robot mass should be numeric');
      end
      sizecheck(robot_mass,[1,1]);
      if(robot_mass<=0)
        error('Drake:CoMForcePlanning:robot mass should be positive');
      end
      obj.robot_mass = robot_mass;
      if(~isnumeric(t))
        error('Drake:CoMForcePlanning:t should be numeric');
      end
      obj.t_knot = reshape(unique(t),1,[]);
      obj.nT = length(obj.t_knot);
      obj.g = 9.81;
      if(~isnumeric(lambda))
        error('Drake:CoMForcePlanning:lambda should be numeric');
      end
      sizecheck(lambda,[3,3]);
      if(any(eig(lambda)>=0))
        error('Drake:CoMForcePlanning:lambda should be a Hurwitz matrix. Namely all its eigen values should be negative');
      end
      if(~isnumeric(c_margin))
        error('Drake:CoMForcePlanning:c_margin should be numeric');
      end
      sizecheck(c_margin,[1,1]);
      if(c_margin<0)
        error('Drake:CoMForcePlanning:c_margin should be non-negative');
      end
      if(~isnumeric(dt_max))
        error('Drake:CoMForcePlanning:dt_max should be numeric');
      end
      sizecheck(dt_max,[1,1]);
      if(dt_max<=0)
        error('Drake:CoMForcePlanning:dt_max should be positive');
      end
      if(~isnumeric(sdot_max))
        error('Drake:CoMForcePlanningForce:sdot_max should be numeric');
      end
      sizecheck(sdot_max,[1,1]);
      if(sdot_max<=0)
        error('Drake:CoMForcePlanningForce:sdot_max should be positive');
      end
      if(~isnumeric(Q_comddot))
        error('Drake:CoMForcePlanning:Q_comddot should be numeric');
      end
      sizecheck(Q_comddot,[3,3]);
      if(any(eig(Q_comddot)<0))
        error('Drake:CoMForcePlanning:Q_comddot should be a positive semi-definite matrix');
      end
      obj.num_fsrc_cnstr = length(fsrc_cnstr);
      obj.fsrc_cnstr = fsrc_cnstr;
      body_pos_names = cell(2*obj.num_fsrc_cnstr,1);
      yaw_names = cell(obj.num_fsrc_cnstr,1);
      obj.F2fsrc_map = cell(1,obj.nT);
      obj.F_idx = cell(1,obj.nT);
      obj.fsrc_knot_active_idx = cell(1,obj.num_fsrc_cnstr);
      for i = 1:obj.num_fsrc_cnstr
        if(~isa(obj.fsrc_cnstr{i},'FootStepRegionContactConstraint'))
          error('Drake:CoMForcePlanning: the input should be a FootStepRegionContactConstraint');
        end
        body_pos_names(2*(i-1)+(1:2)) = {sprintf('fsrc[%d]_pos_x',i);sprintf('fsrc[%d]_pos_y',i)};
        yaw_names{i} = sprintf('fsrc[%d]_yaw',i);
        is_fsrc_active = false;
        for j = 1:obj.nT
          if(obj.fsrc_cnstr{i}.foot_step_region_cnstr.isTimeValid(obj.t_knot(j)))
            obj.fsrc_knot_active_idx{i} = [obj.fsrc_knot_active_idx{i} j];
            obj.F2fsrc_map{j} = [obj.F2fsrc_map{j} i];
            obj.F_idx{j} = [obj.F_idx{j} {obj.num_vars+reshape((1:obj.fsrc_cnstr{i}.num_force_weight),obj.fsrc_cnstr{i}.num_edges,obj.fsrc_cnstr{i}.num_contact_pts)}];
            F_names = cell(obj.fsrc_cnstr{i}.num_force_weight,1);
            for k = 1:obj.fsrc_cnstr{i}.num_contact_pts
              for l = 1:obj.fsrc_cnstr{i}.num_edges
                F_names{(k-1)*obj.fsrc_cnstr{i}.num_edges+l} = sprintf('fsrc[%d] pt %d weight %d at %d knot',i,k,l,j);
              end
            end
            obj = obj.addDecisionVariable(obj.fsrc_cnstr{i}.num_force_weight,F_names);
            obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(zeros(obj.fsrc_cnstr{i}.num_force_weight,1),inf(obj.fsrc_cnstr{i}.num_force_weight,1)),obj.F_idx{j}{end}(:));
            is_fsrc_active = true;
          end
        end
        if(~is_fsrc_active)
          error('Drake:CoMForcePlanning: The %dth FootStepRegionContactConstraint is not active for any t');
        end
      end
      obj.fsrc_body_pos_idx = obj.num_vars+reshape(1:2*obj.num_fsrc_cnstr,2,obj.num_fsrc_cnstr);
      obj = obj.addDecisionVariable(2*obj.num_fsrc_cnstr,body_pos_names);
      obj.yaw_idx = obj.num_vars+(1:obj.num_fsrc_cnstr);
      obj = obj.addDecisionVariable(obj.num_fsrc_cnstr,yaw_names);
      com_names = cell(3*obj.nT,1);
      comp_names = cell(3*obj.nT,1);
      compp_names = cell(3*obj.nT,1);
      sdot_names = cell(obj.nT,1);
      H_names = cell(3*obj.nT,1);
      Hdot_names = cell(3*obj.nT,1);
      margin_names = cell(obj.nT,1);
      for i = 1:obj.nT
        com_names((i-1)*3+(1:3)) = {sprintf('com_x[%d]',i);sprintf('com_y[%d]',i);sprintf('com_z[%d]',i)};
        comp_names((i-1)*3+(1:3)) = {sprintf('comp_x[%d]',i);sprintf('comp_y[%d]',i);sprintf('comp_z[%d]',i)};
        compp_names((i-1)*3+(1:3)) = {sprintf('compp_x[%d]',i);sprintf('compp_y[%d]',i);sprintf('compp_z[%d]',i)};
        sdot_names{i} = sprintf('sdot[%d]',i);
        H_names((i-1)*3+(1:3)) = {sprintf('H_x[%d]',i);sprintf('H_y[%d]',i);sprintf('H_z[%d]',i)};
        Hdot_names((i-1)*3+(1:3)) = {sprintf('Hdot_x[%d]',i);sprintf('Hdot_y[%d]',i);sprintf('Hdot_z[%d]',i)};
        margin_names{i} = sprintf('force_margin[%d]',i);
      end
      obj.com_idx = obj.num_vars+reshape(1:3*obj.nT,3,obj.nT);
      obj = obj.addDecisionVariable(3*obj.nT,com_names);
      obj.comp_idx = obj.num_vars+reshape(1:3*obj.nT,3,obj.nT);
      obj = obj.addDecisionVariable(3*obj.nT,comp_names);
      obj.compp_idx = obj.num_vars+reshape(1:3*obj.nT,3,obj.nT);
      obj = obj.addDecisionVariable(3*obj.nT,compp_names);
      obj.sdot_idx = obj.num_vars+(1:obj.nT);
      obj = obj.addDecisionVariable(obj.nT,sdot_names);
      obj.H_idx = obj.num_vars+reshape(1:3*obj.nT,3,obj.nT);
      obj = obj.addDecisionVariable(3*obj.nT,H_names);
      obj.Hdot_idx = obj.num_vars + reshape(1:3*obj.nT,3,obj.nT);
      obj = obj.addDecisionVariable(3*obj.nT,Hdot_names);
      obj.margin_idx = obj.num_vars+(1:obj.nT);
      obj = obj.addDecisionVariable(obj.nT,margin_names);
      
      delta_s = 1/(obj.nT-1);
      % linear constraint com[n]-com[n-1] = comp[n]*delta_s; comp[n]-comp[n-1] =
      % compp[n]*delta_s
      iAcom = [(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';];
      jAcom = [reshape(obj.com_idx(:,2:end),[],1);reshape(obj.com_idx(:,1:end-1),[],1); reshape(obj.comp_idx(:,2:end),[],1)];
      Aval_com = [ones(3*(obj.nT-1),1);-ones(3*(obj.nT-1),1);-reshape(bsxfun(@times,ones(3,1),delta_s*ones(1,obj.nT-1)),[],1)];
      iAcom = [iAcom;3*(obj.nT-1)+iAcom];
      jAcom = [jAcom;reshape(obj.comp_idx(:,2:end),[],1);reshape(obj.comp_idx(:,1:end-1),[],1); reshape(obj.compp_idx(:,2:end),[],1)];
      Aval_com = [Aval_com;Aval_com];
      lin_com_cnstr = LinearConstraint(zeros(6*(obj.nT-1),1),zeros(6*(obj.nT-1),1),sparse(iAcom,jAcom,Aval_com,6*(obj.nT-1),obj.num_vars));
      obj = obj.addLinearConstraint(lin_com_cnstr);
      % linear constraint that H[n]-H[n-1] = Hdot[n]*delta_s
      iA_H = [(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';];
      jA_H = [reshape(obj.H_idx(:,2:end),[],1);reshape(obj.H_idx(:,1:end-1),[],1); reshape(obj.Hdot_idx(:,2:end),[],1)];
      Aval_H = [ones(3*(obj.nT-1),1);-ones(3*(obj.nT-1),1);-reshape(bsxfun(@times,ones(3,1),delta_s*ones(1,obj.nT-1)),[],1)];
      lin_H_cnstr = LinearConstraint(zeros(3*(obj.nT-1),1),zeros(3*(obj.nT-1),1),sparse(iA_H,jA_H,Aval_H,3*(obj.nT-1),obj.num_vars));
      obj = obj.addLinearConstraint(lin_H_cnstr);
      
      % Add iris half plane constraints on [x;y;yaw] of the contact body.
      for i = 1:obj.num_fsrc_cnstr
        A_iris_i = obj.fsrc_cnstr{i}.foot_step_region_cnstr.A;
        b_iris_i = obj.fsrc_cnstr{i}.foot_step_region_cnstr.b;
        num_halfspace_iris_i = length(b_iris_i);
        obj = obj.addLinearConstraint(LinearConstraint(-inf(num_halfspace_iris_i,1),b_iris_i,A_iris_i),[obj.fsrc_body_pos_idx(:,i);obj.yaw_idx(i)]);
      end
      
      % Add constraint on time scaling function s
      obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(zeros(obj.nT,1),sdot_max*ones(obj.nT,1)),obj.sdot_idx(:));
      iA_dtmax = reshape(bsxfun(@times,(1:obj.nT-1),ones(2,1)),[],1);
      jA_dtmax = reshape([obj.sdot_idx(1:end-1);obj.sdot_idx(2:end)],[],1);
      Aval_dtmax = ones(2*(obj.nT-1),1);
      obj = obj.addLinearConstraint(LinearConstraint(-inf(obj.nT-1,1),dt_max*ones(obj.nT-1,1),sparse(iA_dtmax,jA_dtmax,Aval_dtmax,obj.nT-1,obj.num_vars)));
      
      % Add the constraint for force margin
      for i = 1:obj.nT
        for j = 1:length(obj.F2fsrc_map{i})
          fsrc_idx = obj.F2fsrc_map{i}(j);
          num_F_ij = obj.fsrc_cnstr{fsrc_idx}.num_force_weight;
          iA_margin = [(1:num_F_ij)';(1:num_F_ij)'];
          jA_margin = [obj.margin_idx(i)*ones(num_F_ij,1);obj.F_idx{i}{j}(:)];
          Aval_margin = [ones(num_F_ij,1);-ones(num_F_ij,1)];
          obj = obj.addLinearConstraint(LinearConstraint(-inf(num_F_ij,1),zeros(num_F_ij,1),sparse(iA_margin,jA_margin,Aval_margin,num_F_ij,obj.num_vars)));
        end
      end
      
      % Add the nonlinear constraints on the wrench
      sdot1_idx = [obj.sdot_idx(1:end-1) obj.sdot_idx(end-1)];
      sdot2_idx = [obj.sdot_idx(2:end) obj.sdot_idx(end)];
      for i = 1:obj.nT
        fsrc_idx = obj.F2fsrc_map{i};
        wrench_cnstr = WrenchMatchConstraint(obj.robot_mass,obj.g,obj.fsrc_cnstr(fsrc_idx),delta_s);
        Fi_idx = [];
        for j = 1:length(fsrc_idx)
          Fi_idx = [Fi_idx;obj.F_idx{i}{j}(:)];
        end
        obj = obj.addNonlinearConstraint(wrench_cnstr,...
          [obj.com_idx(:,i);obj.comp_idx(:,i);obj.compp_idx(:,i);sdot1_idx(i);sdot2_idx(i);obj.Hdot_idx(:,i);reshape(obj.fsrc_body_pos_idx(:,fsrc_idx),[],1);(obj.yaw_idx(:,fsrc_idx))';Fi_idx]);
      end
      
      % Add cost function
      compp_cost = QuadraticSumConstraint(-inf,inf,Q_comddot,zeros(3,obj.nT));
      obj = obj.addCost(compp_cost,obj.compp_idx(:));
      margin_cost = LinearConstraint(-inf,inf,-c_margin*ones(1,obj.nT));
      obj = obj.addCost(margin_cost,obj.margin_idx(:));
      angular_PD_cost = QuadraticSumConstraint(-inf,inf,[eye(3) -lambda';-lambda lambda'*lambda],zeros(6,obj.nT));
      obj = obj.addCost(angular_PD_cost,reshape([obj.Hdot_idx;obj.H_idx],[],1));
    end
    
    function obj = addCoMFootDistanceConstraint(obj,fsrc_idx,dist_lb,dist_ub)
      % add a CoMFootDistanceConstraint to the object
      % @param fsrc_idx    An integer. The distance is imposed on obj.fsrc_cnstr{fsrc_idx}.foot_step_region_cnstr
      % @param dist_lb   A scalar. The lower bound of the distance
      % @param dist_ub   A scalar. The upper bound of the distance
      cfd_cnstr = CoMFootDistanceConstraint(obj.fsrc_cnstr{fsrc_idx}.foot_step_region_cnstr,dist_lb,dist_ub);
      for i = 1:length(obj.fsrc_knot_active_idx{fsrc_idx})
        obj = obj.addNonlinearConstraint(cfd_cnstr,[obj.com_idx(:,obj.fsrc_knot_active_idx{fsrc_idx}(i));obj.fsrc_body_pos_idx(:,fsrc_idx);obj.yaw_idx(:,fsrc_idx)]);
      end
    end
    
    
  end
end