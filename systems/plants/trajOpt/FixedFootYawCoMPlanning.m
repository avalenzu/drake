classdef FixedFootYawCoMPlanning
  % this planner takes the input FootStepRegionConstraint and FIXED yaw angle positions,
  % and output a dynamically feasible CoM trajectory of the robot and the corresponding
  % contact forces.
  properties
    seed_step  % A FixedFootYawCoMPlanningSeed object
    f_step  % A FixedFootYawCoMPlanningForce object
    p_step  % A FixedFootYawCoMPlanningPosition object
    nlp_step % A FixedFootYawCoMPlanningNLP object
  end
  
  properties(SetAccess = protected)
    num_fsrc_cnstr % An integer. The total number of FootStepRegionContactConstraint
    fsrc_cnstr % A cell array. All the FootStepRegionContactConstraint object
    F2fsrc_map % A cell arry. obj..fsrc_cnstr{F2fsrc_map{i}(j)} is the FootStepContactRegionConstraint corresponds to the force x(obj.F_idx{i}{j})
    fsrc_knot_active_idx % A cell array. fsrc_knot_active_idx{i} is the indices of the knots that are active for i'th FootStepRegionContactConstraint
    yaw % A 1 x num_fsrc_cnstr double vector. yaw(i) is the yaw angle for obj.fsrc_cnstr{i}
    A_force % A cell array.  A_force{i} = obj.fsrc_cnstr{i}.force, which is a 3 x obj.fsrc_cnstr[i}.num_edges matrix
    A_xy,b_xy,rotmat  % A_xy is 3 x 2 x obj.num_fsrc_cnstr matrix. b_xy is 3 x 1 x obj.num_fsrc_cnstr matrix. rotmat is 3 x 3 x obj.num_fsrc_cnstr matrix. [rotmat(:,:,i),A_xy(:,:,i),b_xy(:,:,i)] = obj.fsrc_cnstr{i}.bodyTransform(obj.yaw(i)); 
    lambda % A 3 x 3 Hurwitz matrix
    nT % The total number of knot points
    robot_mass % The mass of the robot
    g % gravitational acceleration
    robot_dim % A estimation of the dimension of robot in meters
  end
  
  methods
    function obj = FixedFootYawCoMPlanning(robot_mass,robot_dim,t,lambda,c_margin,dt_max,sdot_max,Q_comddot,fsrc_cnstr,yaw)
      % @param robot_mass    The mass of the robot
      % @param robot_dim     The estimated robot dimension in meters
      % @param t             The time knot for planning. This indicates which
      % FootStepRegionContactConstraint is active at a given time knot. The actual time is
      % determined by the scaling function.
      % @properties fsrc_cnstr    A cell of FootStepRegionContactConstraint
      % @properties yaw      A double vector. yaw(i) is the yaw angle of the body in
      % fsrc_cnstr{i}
      if(~isnumeric(robot_mass))
        error('Drake:FixedFootYawCoMPlanning:robot mass should be numeric');
      end
      sizecheck(robot_mass,[1,1]);
      if(robot_mass<=0)
        error('Drake:FixedFootYawCoMPlanning:robot mass should be positive');
      end
      obj.robot_mass = robot_mass;
      if(~isnumeric(robot_dim))
        error('Drake:FixedFootYawCoMPlanning:robot dimension should be numeric');
      end
      sizecheck(robot_dim,[1,1]);
      if(robot_dim<=0)
        error('Drake:FixedFootYawCoMPlanning:robot dimension should be positive');
      end
      obj.robot_dim = robot_dim;
      if(~isnumeric(t))
        error('Drake:FixedFootYawCoMPlanning:t should be numeric');
      end
      t_knot = reshape(unique(t),1,[]);
      obj.nT = length(t_knot);
      obj.g = 9.81;
      if(~isnumeric(lambda))
        error('Drake:FixedFootYawCoMPlanning:lambda should be numeric');
      end
      sizecheck(lambda,[3,3]);
      if(any(eig(lambda)>=0))
        error('Drake:FixedFootYawCoMPlanning:lambda should be a Hurwitz matrix. Namely all its eigen values should be negative');
      end
      obj.lambda = lambda;
      if(~isnumeric(c_margin))
        error('Drake:FixedFootYawCoMPlanning:c_margin should be numeric');
      end
      sizecheck(c_margin,[1,1]);
      if(c_margin<0)
        error('Drake:FixedFootYawCoMPlanning:c_margin should be non-negative');
      end
      if(~isnumeric(dt_max))
        error('Drake:FixedFootYawCoMPlanning:dt_max should be numeric');
      end
      sizecheck(dt_max,[1,1]);
      if(dt_max<=0)
        error('Drake:FixedFootYawCoMPlanning:dt_max should be positive');
      end
      if(~isnumeric(sdot_max))
        error('Drake:FixedFootYawCoMPlanning:sdot_max should be numeric');
      end
      sizecheck(sdot_max,[1,1]);
      if(sdot_max<=0)
        error('Drake:FixedFootYawCoMPlanning:sdot_max should be positive');
      end
%       if(~isnumeric(Q_H))
%         error('Drake:FixedFootYawCoMPlanning:Q_H should be numeric');
%       end
%       sizecheck(Q_H,[3,3]);
%       if(any(eig(Q_H)<0))
%         error('Drake:FixedFootYawCoMPlanning:Q_H should be PSD matrix');
%       end
%       if(~isnumeric(Q_Hdot))
%         error('Drake:FixedFootYawCoMPlanning:Q_Hdot should be numeric');
%       end
%       sizecheck(Q_Hdot,[3,3]);
%       if(any(eig(Q_Hdot)<0))
%         error('Drake:FixedFootYawCoMPlanning:Q_Hdot should be PSD matrix');
%       end
      if(~isnumeric(Q_comddot))
        error('Drake:FixedFootYawCoMPlanning:Q_comddot should be numeric');
      end
      sizecheck(Q_comddot,[3,3]);
      if(any(eig(Q_comddot)<0))
        error('Drake:FixedFootYawCoMPlanning:Q_comddot should be a positive semi-definite matrix');
      end
      
      obj.num_fsrc_cnstr = length(fsrc_cnstr);
      obj.fsrc_cnstr = fsrc_cnstr;
      obj.F2fsrc_map = cell(1,obj.nT);
      obj.fsrc_knot_active_idx = cell(1,obj.num_fsrc_cnstr);
      sizecheck(yaw,[1,obj.num_fsrc_cnstr]);
      obj.yaw = yaw;
      obj.A_force = cell(1,obj.num_fsrc_cnstr);
      obj.A_xy = zeros(3,2,obj.num_fsrc_cnstr);
      obj.b_xy = zeros(3,1,obj.num_fsrc_cnstr);
      obj.rotmat = zeros(3,3,obj.num_fsrc_cnstr);
      num_force_weight = 0;
      for i = 1:obj.num_fsrc_cnstr
        if(~isa(obj.fsrc_cnstr{i},'FootStepRegionContactConstraint'))
          error('Drake:FixedFootYawCoMPlanningPosition:The input should be a FootStepRegionContactConstraint');
        end
        if(~isnumeric(obj.yaw(i)))
          error('Drake:FixedFootYawCoMPlanningPosition:The input yaw angle should be a double');
        end
        obj.A_force{i} = obj.fsrc_cnstr{i}.force(yaw(i));
        [obj.rotmat(:,:,i),obj.A_xy(:,:,i),obj.b_xy(:,:,i)] = obj.fsrc_cnstr{i}.foot_step_region_cnstr.bodyTransform(obj.yaw(i));
        for j = 1:obj.nT
          if(obj.fsrc_cnstr{i}.foot_step_region_cnstr.isTimeValid(t_knot(j)))
            obj.fsrc_knot_active_idx{i} = [obj.fsrc_knot_active_idx{i} j];
            obj.F2fsrc_map{j} = [obj.F2fsrc_map{j} i];
            num_force_weight = num_force_weight+obj.fsrc_cnstr{i}.num_force_weight;
            is_fsrc_active = true;
          end
        end
        if(~is_fsrc_active)
          error('Drake:FixedFootYawCoMPlanningPosition:The %dth FootStepRegionContactConstraint is not active for any t_knot');
        end
      end
      
      obj.p_step = FixedFootYawCoMPlanningPosition(robot_mass,robot_dim,t_knot,obj.g,lambda,Q_comddot,obj.fsrc_cnstr,...
        obj.yaw,obj.F2fsrc_map,obj.fsrc_knot_active_idx,obj.A_force,obj.A_xy,obj.b_xy,obj.rotmat);
      obj.f_step = FixedFootYawCoMPlanningForce(robot_mass,robot_dim,t_knot,obj.g,lambda,c_margin,dt_max,sdot_max,...
        obj.fsrc_cnstr,obj.yaw,obj.F2fsrc_map,obj.fsrc_knot_active_idx,obj.A_force,obj.A_xy,obj.b_xy,obj.rotmat);
      obj.seed_step = FixedFootYawCoMPlanningSeed(robot_mass,t_knot,obj.g,lambda,c_margin,Q_comddot,...
        obj.fsrc_cnstr,obj.yaw,obj.F2fsrc_map,obj.fsrc_knot_active_idx,obj.A_force,obj.A_xy,obj.b_xy,obj.rotmat);
      obj.nlp_step = FixedFootYawCoMPlanningNLP(robot_mass,robot_dim,t_knot,obj.g,lambda,c_margin,Q_comddot,obj.fsrc_cnstr,...
        obj.yaw,obj.F2fsrc_map,obj.fsrc_knot_active_idx,obj.A_force,obj.A_xy,obj.b_xy,obj.rotmat);
      
    end
    
    function [com,comp,compp,foot_pos,Hdot,F] = solve(obj,sdot0,H0)
      % @param sdot0  A 1 x obj.nT vector. sdot(i) is the time derivative of the scaling
      % function s at i'th knot. This is used as a initial guess
      [com,comp,compp,foot_pos,F,sdotsquare] = obj.seed_step.solve(sdot0);
      [Hbar,Hdot,sigma,epsilon] = obj.seed_step.angularMomentum(com,foot_pos,F,H0);
      max_iter = 5;
      sigma_sol = zeros(1,2*max_iter);
      iter = 0;
      while(iter<max_iter)
        iter = iter+1;
        [com,comp,compp,foot_pos,Hdot,Hbar,sigma_sol(2*iter-1),epsilon] = obj.p_step.solve(F,sdotsquare,sigma);
%         checkSolution(obj,com,comp,compp,foot_pos,F,sdotsquare,Hdot,Hbar,epsilon);
        sigma = sigma_sol(2*iter-1);
        [F,sdotsquare,Hdot,Hbar,margin,sigma_sol(2*iter),epsilon] = obj.f_step.solve(com,comp,compp,foot_pos,sigma);
%         checkSolution(obj,com,comp,compp,foot_pos,F,sdotsquare,Hdot,Hbar,epsilon);
        sigma = sigma_sol(2*iter);
      end
      
      obj.nlp_step = obj.nlp_step.setSolverOptions('snopt','iterationslimit',1e5);
      [com,comp,compp,foot_pos,F,Hdot,sigma,INFO] = obj.nlp_step.solve(sqrt(sdotsquare),com,comp,compp,foot_pos,F,margin,Hbar(:,1));
      sdot = sqrt(sdotsquare);
      comdot = comp.*bsxfun(@times,sdot,ones(3,1));
      sdotsquare_diff = diff(sdotsquare);
      sdotsquare_diff = [sdotsquare_diff sdotsquare_diff(end)];
      comddot = compp.*bsxfun(@times,sdotsquare,ones(3,1))+comp.*bsxfun(@times,sdotsquare_diff,(obj.nT-1)/2*ones(3,1));
      delta_t = 2/(obj.nT-1)./(sdot(1:end-1)+sdot(2:end));
      t = cumsum([0 delta_t]);
    end
    
    function obj = addCoMBounds(obj,com_idx,com_lb,com_ub)
      % @param com_idx    A 1 x n vector. The com(:,com_idx) will be bounded
      % @param com_lb     A 3 x n vector. The lower bound for com(:,com_idx);
      % @param com_ub     A 3 x n vector. The upper bound for com(:,com_idx);
      num_idx = length(com_idx);
      sizecheck(com_lb,[3,num_idx]);
      sizecheck(com_ub,[3,num_idx]);
      bnds = BoundingBoxConstraint(com_lb(:),com_ub(:));
      obj.seed_step = obj.seed_step.addBoundingBoxConstraint(bnds,reshape(obj.seed_step.com_idx(:,com_idx),[],1));
      obj.p_step = obj.p_step.addBoundingBoxConstraint(bnds,reshape(obj.p_step.com_idx(:,com_idx),[],1));
      obj.nlp_step = obj.nlp_step.addBoundingBoxConstraint(bnds,reshape(obj.nlp_step.com_idx(:,com_idx),[],1));
    end
    
    function obj = addH0Bounds(obj,H0_lb,H0_ub)
      % set the bounds on the initial angular momentum
      sizecheck(H0_lb,[3,1]);
      sizecheck(H0_ub,[3,1]);
      scaling_factor = obj.robot_dim*obj.robot_mass*obj.g;
      bnds = BoundingBoxConstraint(H0_lb/scaling_factor,H0_ub/scaling_factor);
      obj.p_step = obj.p_step.addBoundingBoxConstraint(bnds,obj.p_step.H_idx(:,1));
      obj.f_step = obj.f_step.addBoundingBoxConstraint(bnds,obj.f_step.H_idx(:,1));
      obj.nlp_step = obj.nlp_step.addBoundingBoxConstraint(bnds,obj.nlp_step.H0_idx);
    end
    
    function obj = addCoMdotBounds(obj,comdot_idx,comdot_lb,comdot_ub)
      % set the bounds on the com velocity
      % @param comdot_idx  A 1 x n vector. comdot(:,comdot_idx) will be bounded
      % @param comdot_lb   A 3 x n vector. The lower bounds of the CoM velocity
      % @param comdot_ub   A 3 x n vector. The upper bounds of the CoM velocity
      num_idx = length(comdot_idx);
      sizecheck(comdot_lb,[3,num_idx]);
      sizecheck(comdot_ub,[3,num_idx]);
      obj.seed_step.lb_comdot(:,comdot_idx) = comdot_lb;
      obj.seed_step.ub_comdot(:,comdot_idx) = comdot_ub;
      obj.p_step.lb_comdot(:,comdot_idx) = comdot_lb;
      obj.p_step.ub_comdot(:,comdot_idx) = comdot_ub;
      obj.f_step.lb_comdot(:,comdot_idx) = comdot_lb;
      obj.f_step.ub_comdot(:,comdot_idx) = comdot_ub;
    end
    
    function checkSolution(obj,com,comp,compp,foot_pos,F,sdotsquare,Hdot,Hbar,epsilon)
      delta_s = 1/(obj.nT-1);
      valuecheck(diff(com,1,2)-comp(:,2:end)*delta_s,0,1e-4);
      valuecheck(diff(comp,1,2)-compp(:,2:end)*delta_s,0,1e-4);
      if(any(sdotsquare<0))
        error('sdotsquare cannot be negative');
      end
      if(any(sdotsquare>obj.f_step.sdot_max^2))
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
        for j = 1:length(obj.f_step.F_idx{i})
          fsrc_idx = obj.F2fsrc_map{i}(j);
          F_ij = obj.A_force{fsrc_idx}*F{i}{j};
          F_i = F_i+sum(F_ij,2);
          foot_contact_pts_CoM = foot_contact_pts_pos{fsrc_idx}-bsxfun(@times,com(:,i),ones(1,obj.fsrc_cnstr{fsrc_idx}.num_contact_pts));
          tau_i = tau_i+sum(cross(foot_contact_pts_CoM,F_ij),2);
        end
        F_i(3) = F_i(3)-obj.robot_mass*obj.g;
        mcomddot = obj.robot_mass*(compp(:,i)*sdotsquare(i)+comp(:,i)/(2*delta_s)*sdot_diff(i));
        valuecheck(F_i,mcomddot,1e-5);
        valuecheck(tau_i,Hdot(:,i),1e-3);
      end
      valuecheck(diff(Hbar,1,2),Hdot(:,2:end)*delta_s/(obj.robot_mass*obj.g),1e-3);
      valuecheck(Hdot/(obj.robot_mass*obj.g),obj.lambda*Hbar+epsilon,1e-3);
      sdot = sqrt(sdotsquare);
      if(any(2*delta_s*ones(1,obj.nT-1)./sum([sdot(1:end-1);sdot(2:end)],1)>obj.f_step.dt_max+1e-6))
        error('dt is above dt_max');
      end      
    end
    
    function plotResult(obj,com,foot_pos,sdotsquare)
      figure;
      hold on;
      axis equal;
      plot(com(1,:),com(2,:),'x-');
      plot(foot_pos(1,:),foot_pos(2,:),'o');
      sdot = sqrt(sdotsquare);
      t = sum((2/(obj.nT-1))./(sdot(1:end-1)+sdot(2:end)));
      title(sprintf('t=%5.2f',t));
    end
  end
end