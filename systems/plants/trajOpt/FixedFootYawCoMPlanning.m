classdef FixedFootYawCoMPlanning
  % this planner takes the input FootStepRegionConstraint and FIXED yaw angle positions,
  % and output a dynamically feasible CoM trajectory of the robot and the corresponding
  % contact forces.
  properties
    seed_step  % A FixedFootYawCoMPlanningSeed object
%     f_step  % A FixedFootYawCoMPlanningForce object
%     p_step  % A FixedFootYawCoMPlanningPosition object
    nlp_step % A FixedFootYawCoMPlanningNLP object
    com_traj_order % An integer. The order of the polynomial to interpolate CoM. Acceptable orders are cubic or quartic
    visualizer % A FixedFootYawCoMPlanningVisualizer object
  end
  
  properties(SetAccess = protected)
    num_fsrc_cnstr % An integer. The total number of FootStepRegionContactConstraint
    fsrc_cnstr % An array of FootStepRegionContactConstraint object
    F2fsrc_map % A cell arry. obj.fsrc_cnstr{F2fsrc_map{i}(j)} is the FootStepContactRegionConstraint corresponds to the force x(obj.F_idx{i}{j})
    fsrc_tspan % A obj.num_fsrc_cnstr x 2 double matrix. fsrc_tspan(i,:) is the time span for the i'th knot points
    fsrc_knot_active_idx % A cell array. fsrc_knot_active_idx{i} is the indices of the knots that are active for i'th FootStepRegionContactConstraint
    yaw % A 1 x num_fsrc_cnstr double vector. yaw(i) is the yaw angle for obj.fsrc_cnstr(i)
    A_force % A cell array.  A_force{i} = obj.fsrc_cnstr(i).force, which is a 3 x obj.fsrc_cnstr(i).num_edges matrix
    A_xy,b_xy,rotmat  % A_xy is 3 x 2 x obj.num_fsrc_cnstr matrix. b_xy is 3 x 1 x obj.num_fsrc_cnstr matrix. rotmat is 3 x 3 x obj.num_fsrc_cnstr matrix. [rotmat(:,:,i),A_xy(:,:,i),b_xy(:,:,i)] = obj.fsrc_cnstr(i).bodyTransform(obj.yaw(i)); 
    lambda % A 3 x 3 Hurwitz matrix
    t_knot % The time knots
    nT % The total number of knot points
    robot_mass % The mass of the robot
    g % gravitational acceleration
    robot_dim % A estimation of the dimension of robot in meters
    robot_inertia % An estimation of the inertia of the robot
    
    com_frame;
    fsrc_frame;
    zmp_frame;
    visualize_flag;
  end
  
  methods
    function obj = FixedFootYawCoMPlanning(robot_mass,robot_dim,robot_inertia,t,lambda,c_margin,Q_comddot,fsrc_cnstr,yaw,com_traj_order)
      % @param robot_mass    The mass of the robot
      % @param robot_dim     The estimated robot dimension in meters
      % @param robot_inertia  The estimated robot inertia
      % @param t             The time knot for planning. 
      % @properties fsrc_cnstr    A cell of FootStepRegionContactConstraint
      % @param yaw      A double vector. yaw(i) is the yaw angle of the body in
      % fsrc_cnstr(i)
      % @param com_traj_order    The order of polynomial to interpolate the CoM
      % trajectory. Currently accept 3 or 4
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
      sizecheck(robot_inertia,[3,3]);
      if(any(eig(robot_inertia)<=0))
        error('Drake:FixedFootYawCoMPlanning:robot inertia should be positive definite');
      end
      obj.robot_inertia = robot_inertia;
      if(~isnumeric(t))
        error('Drake:FixedFootYawCoMPlanning:t should be numeric');
      end
      obj.t_knot = reshape(unique(t),1,[]);
      for i = 1:length(fsrc_cnstr)
        t_diff = bsxfun(@minus,obj.t_knot,fsrc_cnstr(i).foot_step_region_cnstr.tspan');
        obj.t_knot = unique([obj.t_knot(all(abs(t_diff)>1e-5,1)) fsrc_cnstr(i).foot_step_region_cnstr.tspan]);
      end
      obj.nT = length(obj.t_knot);
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
        if(~isa(obj.fsrc_cnstr(i),'FootStepRegionContactConstraint'))
          error('Drake:FixedFootYawCoMPlanningPosition:The input should be a FootStepRegionContactConstraint');
        end
        if(~isnumeric(obj.yaw(i)))
          error('Drake:FixedFootYawCoMPlanningPosition:The input yaw angle should be a double');
        end
        obj.A_force{i} = obj.fsrc_cnstr(i).force(yaw(i));
        [obj.rotmat(:,:,i),obj.A_xy(:,:,i),obj.b_xy(:,:,i)] = obj.fsrc_cnstr(i).foot_step_region_cnstr.bodyTransform(obj.yaw(i));
        for j = 1:obj.nT
          if(obj.fsrc_cnstr(i).foot_step_region_cnstr.isTimeValid(obj.t_knot(j)))
            obj.fsrc_knot_active_idx{i} = [obj.fsrc_knot_active_idx{i} j];
            obj.F2fsrc_map{j} = [obj.F2fsrc_map{j} i];
            num_force_weight = num_force_weight+obj.fsrc_cnstr(i).num_force_weight;
            is_fsrc_active = true;
          end
        end
        if(~is_fsrc_active)
          error('Drake:FixedFootYawCoMPlanningPosition:The %dth FootStepRegionContactConstraint is not active for any t_knot');
        end
      end
      if(com_traj_order == 3 || com_traj_order == 4)
        obj.com_traj_order = com_traj_order;
      else
        error('Unsupported order of polynoimal to interpolate com trajectory');
      end
      % compute a reference z-axis angular momentum profile
      body_yaw = zeros(1,obj.nT);
      for i = 1:obj.nT
        body_yaw(i) = mean(obj.yaw(obj.F2fsrc_map{i}));
      end
%       body_yaw_rate = diff(body_yaw)./diff(obj.t_knot);
%       body_yaw_rate = [body_yaw_rate(1) body_yaw_rate];
      body_yaw_spline = PPTrajectory(spline(obj.t_knot,[0 body_yaw 0]));
      body_yaw_rate = body_yaw_spline.deriv(obj.t_knot);
      H_des = obj.robot_inertia(3,3)*[zeros(2,obj.nT);body_yaw_rate];
      obj.com_frame = CoordinateFrame('com',3,[],{'com_x';'com_y';'com_z'});
      obj.fsrc_frame = cell(obj.num_fsrc_cnstr,1);
      for i = 1:obj.num_fsrc_cnstr
        obj.fsrc_frame{i} = CoordinateFrame('fsrc',3+obj.fsrc_cnstr(i).num_contact_pts*3,[]);
      end
      obj.zmp_frame = CoordinateFrame('zmp',2,[],{'zmp_x','zmp_y'});
%       obj.p_step = FixedFootYawCoMPlanningPosition(robot_mass,robot_dim,obj.t_knot,obj.g,lambda,Q_comddot,obj.fsrc_cnstr,...
%         obj.yaw,obj.F2fsrc_map,obj.fsrc_knot_active_idx,obj.A_force,obj.A_xy,obj.b_xy,obj.rotmat,obj.com_traj_order);
%       obj.f_step = FixedFootYawCoMPlanningForce(robot_mass,robot_dim,obj.t_knot,obj.g,lambda,c_margin,...
%         obj.fsrc_cnstr,obj.yaw,obj.F2fsrc_map,obj.fsrc_knot_active_idx,obj.A_force,obj.A_xy,obj.b_xy,obj.rotmat);
      obj.seed_step = FixedFootYawCoMPlanningSeed(robot_mass,robot_dim,obj.t_knot,obj.g,lambda,c_margin,Q_comddot,...
        obj.fsrc_cnstr,obj.yaw,obj.F2fsrc_map,obj.fsrc_knot_active_idx,obj.A_force,obj.A_xy,obj.b_xy,obj.rotmat,obj.com_traj_order);
      obj.nlp_step = FixedFootYawCoMPlanningNLP(robot_mass,robot_dim,obj.t_knot,obj.g,lambda,c_margin,Q_comddot,obj.fsrc_cnstr,...
        obj.yaw,obj.F2fsrc_map,obj.fsrc_knot_active_idx,obj.A_force,obj.A_xy,obj.b_xy,obj.rotmat,obj.com_traj_order,H_des);
      obj.fsrc_tspan = zeros(obj.num_fsrc_cnstr,2);
      for i = 1:obj.num_fsrc_cnstr
        obj.fsrc_tspan(i,:) = [obj.t_knot(obj.fsrc_knot_active_idx{i}(1)) obj.t_knot(obj.fsrc_knot_active_idx{i}(end))];
      end
      obj.visualizer = FixedFootYawCoMPlanningVisualizer(robot_mass,obj.g,obj.fsrc_cnstr,obj.fsrc_tspan,obj.com_frame,obj.fsrc_frame,obj.zmp_frame);
      obj.visualize_flag = false;
    end
    
    function [com,comdot,comddot,foot_pos,Hdot,H,F,com_traj,foot_xyzrpy,foot_position_cnstr,foot_quat_cnstr] = solve(obj,robot,H0)
      % @param robot  A RigidBodyManipulator object
      % @param H0   A 3 x 1 vector. The guess of the initial angular momentum
      % @retval com  A 3 x obj.nT matrix. com(:,i) is the com position at i'th knot point
      % @retval comdot  A 3 x obj.nT matrix. comdot(:,i) is the com velocity at i'th knot
      % point
      % @retval comddot  A 3 x obj.nT matrix. comdot(:,i) is the com acceleration at i'th knot
      % point
      % @retval foot_pos A 2 x obj.num_fsrc_cnstr matrix. foot_pos(:,i) is the foot xy
      % position of the i'th FootStepRegionContactConstraint
      % @retval Hdot  A 3 x obj.nT matrix. Hdot(:,i) is the rate of the angular momentum
      % at the i'th knot point
      % @retval H    A 3 x obj.nT matrix. H(:,i) is the angular momentum
      % at the i'th knot point
      % @retval foot_position_cnstr  An array of WorldPositionConstraint.
      % foot_position_cnstr(i) encodes the position constraint for i'th
      % FootStepRegionContactConstraint
      % @retval foot_quat_cnstr  An array of WorldQuatConstraint.
      % foot_quat_cnstr(i) encodes the orientation constraint for i'th
      % FootStepRegionContactConstraint
      tic
      display('initial QP');
      [com,comdot,comddot,foot_pos,F,margin] = obj.seed_step.solve();
      toc
      [H,Hdot,sigma,epsilon] = obj.seed_step.angularMomentum(com,foot_pos,F,H0);
      checkSolution(obj,com,comdot,comddot,foot_pos,F,Hdot,H,epsilon);
%       max_iter = 10;
%       sigma_sol = zeros(1,2*max_iter);
%       iter = 0;
%       INFO = 1;
      if(obj.visualize_flag)
        obj.visualize(com,comdot,comddot,foot_pos,F,Hdot);
        figure;
        qp_com_handle = plot3(com(1,:),com(2,:),com(3,:),'x-');
        hold on;

        plot3(foot_pos(1,:),foot_pos(2,:),zeros(1,size(foot_pos,2)),'o');
      end
%       while(iter<max_iter && INFO == 1)
%         iter = iter+1;
%         display(sprintf('\nP step, iter %d',iter));
%         tic
%         [com_iter,comdot_iter,comddot_iter,foot_pos_iter,Hdot_iter,H_iter,sigma_sol_iter,epsilon_iter,INFO] = obj.p_step.solve(F,sigma);
%         toc
%         if(INFO == 1)
%           com = com_iter;
%           comdot = comdot_iter;
%           comddot = comddot_iter;
%           foot_pos = foot_pos_iter;
%           Hdot = Hdot_iter;
%           H = H_iter;
%           sigma_sol(2*iter-1) = sigma_sol_iter;
%           epsilon = epsilon_iter;
%         end
%         checkSolution(obj,com,comdot,comddot,foot_pos,F,Hdot,H,epsilon);  
%         sigma = sigma_sol(2*iter-1);
%         display(sprintf('\nF step,iter %d',iter));
%         tic
%         [F_iter,Hdot_iter,H_iter,margin_iter,sigma_sol_iter,epsilon_iter,INFO] = obj.f_step.solve(com,comdot,comddot,foot_pos,sigma);
%         toc
%         if(INFO == 1)
%           F = F_iter;
%           Hdot = Hdot_iter;
%           H = H_iter;
%           margin = margin_iter;
%           sigma_sol(2*iter) = sigma_sol_iter;
%           epsilon = epsilon_iter;
%         end
%         
%         checkSolution(obj,com,comdot,comddot,foot_pos,F,Hdot,H,epsilon);
%         sigma = sigma_sol(2*iter);
%       end
%       bilinear_com_handle = plot3(com(1,:),com(2,:),com(3,:),'x-r');
%       plot3(foot_pos(1,:),foot_pos(2,:),zeros(1,size(foot_pos,2)),'or');
      
      display('nlp step');
      tic;
      obj.nlp_step = obj.nlp_step.setSolverOptions('snopt','superbasicslimit',2000);
      obj.nlp_step = obj.nlp_step.setSolverOptions('snopt','iterationslimit',1e3);
      obj.nlp_step = obj.nlp_step.setSolverOptions('snopt','majoriterationslimit',50);
      obj.nlp_step = obj.nlp_step.setSolverOptions('snopt','majoroptimalitytolerance',1e-4);
%       obj.nlp_step = obj.nlp_step.setSolverOptions('snopt','print','nlp.out');
      [com,comdot,comddot,foot_pos,F,Hdot,H,epsilon,sigma,INFO] = obj.nlp_step.solve(com,comdot,comddot,foot_pos,F,margin,H(:,1));
      if(INFO == 31)
        obj.nlp_step = obj.nlp_step.setSolverOptions('snopt','iterationslimit',1e5);
        obj.nlp_step = obj.nlp_step.setSolverOptions('snopt','majoriterationslimit',200);
        obj.nlp_step = obj.nlp_step.setSolverOptions('snopt','majoroptimalitytolerance',1e-4);
        [com,comdot,comddot,foot_pos,F,Hdot,H,epsilon,sigma,INFO] = obj.nlp_step.solve(com,comdot,comddot,foot_pos,F,margin,H(:,1));
      end
      toc;
      checkSolution(obj,com,comdot,comddot,foot_pos,F,Hdot,H,epsilon);
      com_traj = obj.CoMPPTraj(com,comdot,comddot);
      foot_position_cnstr = WorldPositionConstraint.empty(obj.num_fsrc_cnstr,0);
      foot_quat_cnstr = WorldQuatConstraint.empty(obj.num_fsrc_cnstr,0);
      foot_xyzrpy = zeros(6,obj.num_fsrc_cnstr);
      for i = 1:obj.num_fsrc_cnstr
        [foot_xyzrpy(:,i),foot_position_cnstr(i),foot_quat_cnstr(i)] = obj.fsrc_cnstr(i).foot_step_region_cnstr.generateFixedPosConstraint(robot,foot_pos(:,i),obj.yaw(i));
      end
      if(obj.visualize_flag)
        nlp_com_handle = plot3(com(1,:),com(2,:),com(3,:),'x-g');
        plot3(foot_pos(1,:),foot_pos(2,:),zeros(1,size(foot_pos,2)),'og');
        legend([qp_com_handle nlp_com_handle],'initial guess without optimizing angular momentum','NLP')
        title('COM trajectory')
        axis equal
        figure;
        plot(obj.t_knot,H'/(obj.robot_mass*obj.robot_dim));
        title('angular momentum nomalized by mass times robot length');
        legend('x','y','z');
        obj.visualize(com,comdot,comddot,foot_pos,F,Hdot);
      end
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
%       obj.p_step = obj.p_step.addBoundingBoxConstraint(bnds,reshape(obj.p_step.com_idx(:,com_idx),[],1));
      obj.nlp_step = obj.nlp_step.addBoundingBoxConstraint(bnds,reshape(obj.nlp_step.com_idx(:,com_idx),[],1));
    end
    
    function obj = addH0Bounds(obj,H0_lb,H0_ub)
      % set the bounds on the initial angular momentum
      sizecheck(H0_lb,[3,1]);
      sizecheck(H0_ub,[3,1]);
      scaling_factor = obj.robot_dim*obj.robot_mass*obj.g;
      bnds = BoundingBoxConstraint(H0_lb/scaling_factor,H0_ub/scaling_factor);
%       obj.p_step = obj.p_step.addBoundingBoxConstraint(bnds,obj.p_step.H_idx(:,1));
%       obj.f_step = obj.f_step.addBoundingBoxConstraint(bnds,obj.f_step.H_idx(:,1));
      obj.nlp_step = obj.nlp_step.addBoundingBoxConstraint(bnds,obj.nlp_step.H0_idx);
    end
    
%     function obj = addHBounds(obj,t_idx,H_lb,H_ub)
%       add the bounds on the angular momentum for the bilinear alternation
%       bnds = BoundingBoxConstraint(H_lb,H_ub);
%       obj.p_step = obj.p_step.addBoundingBoxConstraint(bnds,reshape(obj.p_step.H_idx(:,t_idx),[],1));
%       obj.f_step = obj.f_step.addBoundingBoxConstraint(bnds,reshape(obj.f_step.H_idx(:,t_idx),[],1));
%     end
%     
    function obj = addCoMdotBounds(obj,comdot_idx,comdot_lb,comdot_ub)
      % set the bounds on the com velocity
      % @param comdot_idx  A 1 x n vector. comdot(:,comdot_idx) will be bounded
      % @param comdot_lb   A 3 x n vector. The lower bounds of the CoM velocity
      % @param comdot_ub   A 3 x n vector. The upper bounds of the CoM velocity
      num_idx = length(comdot_idx);
      sizecheck(comdot_lb,[3,num_idx]);
      sizecheck(comdot_ub,[3,num_idx]);
      obj.seed_step = obj.seed_step.addBoundingBoxConstraint(BoundingBoxConstraint(comdot_lb,comdot_ub),reshape(obj.seed_step.comdot_idx(:,comdot_idx),[],1));
%       obj.p_step = obj.p_step.addBoundingBoxConstraint(BoundingBoxConstraint(comdot_lb,comdot_ub),reshape(obj.p_step.comdot_idx(:,comdot_idx),[],1));
      obj.nlp_step = obj.nlp_step.addBoundingBoxConstraint(BoundingBoxConstraint(comdot_lb,comdot_ub),reshape(obj.nlp_step.comdot_idx(:,comdot_idx),[],1));
    end
    
    function obj = addCoMddotBounds(obj,comddot_idx,comddot_lb,comddot_ub)
      % set the bounds on the com acceleration
      % @param comddot_idx  A 1 x n vector. comdot(:,comddot_idx) will be bounded
      % @param comddot_lb   A 3 x n vector. The lower bounds of the CoM acceleration
      % @param comddot_ub   A 3 x n vector. The upper bounds of the CoM acceleration
      num_idx = length(comddot_idx);
      sizecheck(comddot_lb,[3,num_idx]);
      sizecheck(comddot_ub,[3,num_idx]);
      obj.seed_step = obj.seed_step.addBoundingBoxConstraint(BoundingBoxConstraint(comddot_lb,comddot_ub),reshape(obj.seed_step.comddot_idx(:,comddot_idx),[],1));
%       obj.p_step = obj.p_step.addBoundingBoxConstraint(BoundingBoxConstraint(comddot_lb,comddot_ub),reshape(obj.p_step.comddot_idx(:,comddot_idx),[],1));
      obj.nlp_step = obj.nlp_step.addBoundingBoxConstraint(BoundingBoxConstraint(comddot_lb,comddot_ub),reshape(obj.nlp_step.comddot_idx(:,comddot_idx),[],1));
    end
    
    function obj = addCoMFootPolygon(obj,fsrc_idx,vertices)
      % add a CoMFootStepPolygon on the CoM and the body in obj.fsrc_cnstr(fsrc_idx)
      % @param fsrc_idx   An integer. The index of the FootStepRegionContactConstraint
      % @param vertices   A 3 x n matrix. vertices(:,i) is the coordinate of the i'th
      % vertex in the body frame
      com_foot_polygon = CoMFootStepPolygon(obj.fsrc_cnstr(fsrc_idx).foot_step_region_cnstr,vertices);
      [A,b] = com_foot_polygon.halfspace(obj.yaw(fsrc_idx));
      for i = 1:length(obj.fsrc_knot_active_idx{fsrc_idx})
        t_idx = obj.fsrc_knot_active_idx{fsrc_idx}(i);
        con_name = repmat({sprintf('polygon constraint between fsrc%d and com at t[%d]',fsrc_idx,t_idx)},length(b),1);
        obj.seed_step = obj.seed_step.addLinearConstraint(LinearConstraint(-inf(length(b),1),b,A),...
          [obj.seed_step.com_idx(:,t_idx);obj.seed_step.fsrc_body_pos_idx(:,fsrc_idx)],con_name);
%         obj.p_step = obj.p_step.addLinearConstraint(LinearConstraint(-inf(length(b),1),b,A),...
%           [obj.p_step.com_idx(:,t_idx);obj.p_step.fsrc_body_pos_idx(:,fsrc_idx)],con_name);
        obj.nlp_step = obj.nlp_step.addLinearConstraint(LinearConstraint(-inf(length(b),1),b,A),...
          [obj.nlp_step.com_idx(:,t_idx);obj.nlp_step.fsrc_body_pos_idx(:,fsrc_idx)],con_name);
      end
    end
    
    function obj = addFootPolygon(obj,fsrc_idx,A_polygon,b_polygon)
      % add a polygonal constraint on the relative position between several
      % FootStepRegionContactConstraint. The polygon is defined as A_polygon*[x1;y1;x2;y2;...;xN;yN] <=
      % b_polygon, where [xi;yi] is the xy position of the i'th
      % FootStepRegionContactConstraint
      % @param fsrc_idx An integer vector. The kinematic constraint is on the
      % bodies in obj.fsrc_cnstr(fsrc_idx(1)) obj.fsrc_cnstr(fsrc_idx(2)) and
      % obj.fsrc_cnstr(fsrc_idx(N))
      % @param A_polygon    A n x (2*length(fsrc_idx)) matrix.
      % @param b_polygon    A n x 1 vector
      if(~isnumeric(fsrc_idx))
        error('Drake:FixedFootYawCoMPlanning:fsrc_idx1 and fsrc_idx2 should be numeric scalar');
      end
      num_fsrc = numel(fsrc_idx);
      sizecheck(fsrc_idx,[1,num_fsrc]);
      if(~isnumeric(A_polygon) || ~isnumeric(b_polygon))
        error('Drake:FixedFootYawCoMPlanning:A and b should be numeric');
      end
      num_cnstr = numel(b_polygon);
      sizecheck(A_polygon,[num_cnstr,2*num_fsrc]);
      sizecheck(b_polygon,[num_cnstr,1]);
      iA = reshape(bsxfun(@times,(1:num_cnstr)',ones(1,2*num_fsrc)),[],1);
      jA = reshape(bsxfun(@times,1:2*length(fsrc_idx),ones(num_cnstr,1)),[],1);
      A_kin_new = sparse(iA,jA,A_polygon(:),num_cnstr,2*length(fsrc_idx));
      A_kin_name = repmat({sprintf('polygon region on fsrc %d',fsrc_idx)},num_cnstr,1);
      obj.seed_step = obj.seed_step.addLinearConstraint(LinearConstraint(-inf(num_cnstr,1),b_polygon,A_kin_new),obj.seed_step.fsrc_body_pos_idx(:,fsrc_idx),A_kin_name);
%       obj.p_step = obj.p_step.addLinearConstraint(LinearConstraint(-inf(num_cnstr,1),b_polygon,A_kin_new),obj.p_step.fsrc_body_pos_idx(:,fsrc_idx),A_kin_name);
      obj.nlp_step = obj.nlp_step.addLinearConstraint(LinearConstraint(-inf(num_cnstr,1),b_polygon,A_kin_new),obj.nlp_step.fsrc_body_pos_idx(:,fsrc_idx),A_kin_name);
    end
    
    function obj = addFootPolygon3D(obj,fsrc_idx,A_polygon,b_polygon)
      % add a 3D polygon constraint on the relative position between several
      % FootStepRegionContactConstraints. The polygon is defined as
      % A_polygon*[x1;y1;z1;x2;y2;z2;...;xN;yN;zN]<=b_polygon
      % @param fsrc_idx An integer vector. The kinematic constraint is on the
      % bodies in obj.fsrc_cnstr(fsrc_idx(1)) obj.fsrc_cnstr(fsrc_idx(2)) and
      % obj.fsrc_cnstr(fsrc_idx(N))
      % @param A_polygon    A n x (3*length(fsrc_idx)) matrix.
      % @param b_polygon    A n x 1 vector 
      num_fsrc = numel(fsrc_idx);
      A_xy_diag_row = reshape(repmat(reshape(1:3*num_fsrc,3,num_fsrc),2,1),[],1);
      A_xy_diag_col = reshape(bsxfun(@times,ones(3,1),1:2*num_fsrc),[],1);
      A_xy_diag_val = reshape(obj.A_xy(:,:,fsrc_idx),[],1);
      A_polygon2D = A_polygon*sparse(A_xy_diag_row,A_xy_diag_col,A_xy_diag_val,3*num_fsrc,2*num_fsrc);
      b_polygon2D = b_polygon-A_polygon*reshape(obj.b_xy(:,:,fsrc_idx),[],1);
      obj = obj.addFootPolygon(fsrc_idx,A_polygon2D,b_polygon2D);
    end
    
    
    function obj = addFootPositionConstraint(obj,fsrc_idx,constraint)
      % Add a BoundingBoxConstraint or a LinearConstraint on the foot position
      % @param fsrc_idx   A scalar. The index of the FootStepRegionContactConstraint
      % @param constraint   A BoundingBoxConstraint or a LinearConstraint object
      if(~isnumeric(fsrc_idx) || numel(fsrc_idx) ~= 1)
        error('Drake:FixedFootYawCoMPlanning: fsrc_idx should be a scaler');
      end
      if(isa(constraint,'LinearConstraint'))
        obj.seed_step = obj.seed_step.addLinearConstraint(constraint,obj.seed_step.fsrc_body_pos_idx(:,fsrc_idx));
%         obj.p_step = obj.p_step.addLinearConstraint(constraint,obj.p_step.fsrc_body_pos_idx(:,fsrc_idx));
        obj.nlp_step = obj.nlp_step.addLinearConstraint(constraint,obj.nlp_step.fsrc_body_pos_idx(:,fsrc_idx));
      elseif(isa(constraint,'BoundingBoxConstraint'))
        obj.seed_step = obj.seed_step.addBoundingBoxConstraint(constraint,obj.seed_step.fsrc_body_pos_idx(:,fsrc_idx));
%         obj.p_step = obj.p_step.addBoundingBoxConstraint(constraint,obj.p_step.fsrc_body_pos_idx(:,fsrc_idx));
        obj.nlp_step = obj.nlp_step.addBoundingBoxConstraint(constraint,obj.nlp_step.fsrc_body_pos_idx(:,fsrc_idx));
      else
        error('Drake:FixedFootYawCoMPlanning: unsupported constraint');
      end
    end
    
    function checkSolution(obj,com,comdot,comddot,foot_pos,F,Hdot,H,epsilon)
      dt = diff(obj.t_knot);
      foot_contact_pts_pos = cell(1,obj.num_fsrc_cnstr);
      for i = 1:obj.num_fsrc_cnstr
        foot_contact_pts_pos{i} = bsxfun(@times,obj.A_xy(:,:,i)*foot_pos(:,i)+obj.b_xy(:,:,i),...
          ones(1,obj.fsrc_cnstr(i).num_contact_pts))+obj.rotmat(:,:,i)*obj.fsrc_cnstr(i).body_contact_pts;
      end
      cop = zeros(2,obj.nT);
      tau_oi = zeros(3,obj.nT);
      for i = 1:obj.nT
        F_i = zeros(3,1);
        tau_i = zeros(3,1);
        for j = 1:length(obj.F2fsrc_map{i})
          fsrc_idx = obj.F2fsrc_map{i}(j);
          F_ij = obj.A_force{fsrc_idx}*F{i}{j};
          F_i = F_i+sum(F_ij,2);
          foot_contact_pts_CoM = foot_contact_pts_pos{fsrc_idx}-bsxfun(@times,com(:,i),ones(1,obj.fsrc_cnstr(fsrc_idx).num_contact_pts));
          tau_i = tau_i+sum(cross(foot_contact_pts_CoM,F_ij),2);
          tau_oi(:,i) = tau_oi(:,i)+sum(cross(foot_contact_pts_pos{fsrc_idx},F_ij),2);
          cop(1,i) = cop(1,i)+sum(F_ij(3,:).*foot_contact_pts_pos{fsrc_idx}(1,:)-F_ij(1,:).*foot_contact_pts_pos{fsrc_idx}(3,:));
          cop(2,i) = cop(2,i)+sum(F_ij(3,:).*foot_contact_pts_pos{fsrc_idx}(2,:)-F_ij(2,:).*foot_contact_pts_pos{fsrc_idx}(3,:));
        end
        cop(:,i) = cop(:,i)/F_i(3);
        F_i(3) = F_i(3)-obj.robot_mass*obj.g;
        mcomddot = obj.robot_mass*comddot(:,i);
        valuecheck(F_i,mcomddot,3e-3);
        valuecheck(tau_i,Hdot(:,i),1e-3);
      end
      zmp = [(-Hdot(2,:)+obj.robot_mass*obj.g*com(1,:)-obj.robot_mass*com(3,:).*comddot(1,:)+obj.robot_mass*com(1,:).*comddot(3,:))./(obj.robot_mass*comddot(3,:)+obj.robot_mass*obj.g);...
        (Hdot(1,:)+obj.robot_mass*com(2,:).*comddot(3,:)-obj.robot_mass*com(3,:).*comddot(2,:)+obj.robot_mass*obj.g*com(2,:))./(obj.robot_mass*comddot(3,:)+obj.robot_mass*obj.g)];
      valuecheck(cop,zmp,1e-3);
      valuecheck(diff(H,1,2),Hdot(:,2:end).*bsxfun(@times,dt,ones(3,1)),1e-3);
      H_scale = obj.robot_mass*obj.g*obj.robot_dim;
      valuecheck(obj.lambda*H/H_scale+epsilon-Hdot/H_scale,zeros(3,obj.nT),1e-3);
      if(obj.com_traj_order == 3)
        a = com(:,1:end-1);
        b = comdot(:,1:end-1);
        c = (3*com(:,2:end)-3*com(:,1:end-1)-(2*comdot(:,1:end-1)+comdot(:,2:end)).*bsxfun(@times,ones(3,1),dt))./bsxfun(@times,ones(3,1),dt.^2);
        d = ((comdot(:,2:end)+comdot(:,1:end-1)).*bsxfun(@times,ones(3,1),dt)-2*com(:,2:end)+2*com(:,1:end-1))./bsxfun(@times,ones(3,1),dt.^3);
        valuecheck(a+b.*bsxfun(@times,ones(3,1),dt)+c.*bsxfun(@times,ones(3,1),dt.^2)+d.*bsxfun(@times,ones(3,1),dt.^3),com(:,2:end),1e-3);
        valuecheck(b+2*c.*bsxfun(@times,ones(3,1),dt)+3*d.*bsxfun(@times,ones(3,1),dt.^2),comdot(:,2:end),1e-3);
        valuecheck(comddot(:,1:end-1),2*c,1e-3);
        valuecheck(12*(com(:,2:end)-com(:,1:end-1))-(6*comdot(:,1:end-1)+6*comdot(:,2:end)).*bsxfun(@times,ones(3,1),dt)+(comddot(:,2:end)-comddot(:,1:end-1)).*bsxfun(@times,ones(3,1),dt.^2),zeros(3,obj.nT-1),1e-3);
      elseif(obj.com_traj_order == 4)
        a = com(:,1:end-1);
        b = comdot(:,1:end-1);
        c = 0.5*comddot(:,1:end-1);
        d = (4*com(:,2:end)-4*com(:,1:end-1)-(3*comdot(:,1:end-1)+comdot(:,2:end)).*bsxfun(@times,ones(3,1),dt)-comddot(:,1:end-1).*bsxfun(@times,ones(3,1),dt.^2))./bsxfun(@times,ones(3,1),dt.^3);
        e = ((comdot(:,2:end)+2*comdot(:,1:end-1)).*bsxfun(@times,ones(3,1),dt)+0.5*comddot(:,1:end-1).*bsxfun(@times,ones(3,1),dt.^2)-3*com(:,2:end)+3*com(:,1:end-1))./bsxfun(@times,ones(3,1),dt.^4);
        valuecheck(a+b.*bsxfun(@times,ones(3,1),dt)+c.*bsxfun(@times,ones(3,1),dt.^2)+d.*bsxfun(@times,ones(3,1),dt.^3)+e.*bsxfun(@times,ones(3,1),dt.^4),com(:,2:end),1e-3);
        valuecheck(b+2*c.*bsxfun(@times,ones(3,1),dt)+3*d.*bsxfun(@times,ones(3,1),dt.^2)+4*e.*bsxfun(@times,ones(3,1),dt.^3),comdot(:,2:end),1e-3);
        valuecheck(2*c+6*d.*bsxfun(@times,ones(3,1),dt)+12*e.*bsxfun(@times,ones(3,1),dt.^2),comddot(:,2:end),1e-3);
        valuecheck(-12*(com(:,2:end)-com(:,1:end-1))+6*(comdot(:,1:end-1)+comdot(:,2:end)).*bsxfun(@times,ones(3,1),dt)-(comddot(:,2:end)-comddot(:,1:end-1)).*bsxfun(@times,ones(3,1),dt.^2),zeros(3,obj.nT-1),1e-3);
      end
    end
    
    function com_traj = CoMPPTraj(obj,com,comdot,comddot)
      dt = diff(obj.t_knot);
      if(obj.com_traj_order == 3)
        a = com(:,1:end-1);
        b = comdot(:,1:end-1);
        c = (3*com(:,2:end)-3*com(:,1:end-1)-(2*comdot(:,1:end-1)+comdot(:,2:end)).*bsxfun(@times,ones(3,1),dt))./bsxfun(@times,ones(3,1),dt.^2);
        d = ((comdot(:,2:end)+comdot(:,1:end-1)).*bsxfun(@times,ones(3,1),dt)-2*com(:,2:end)+2*com(:,1:end-1))./bsxfun(@times,ones(3,1),dt.^3);
        coefs = zeros(3,obj.nT-1,4);
        coefs(:,:,4) = a;
        coefs(:,:,3) = b;
        coefs(:,:,2) = c;
        coefs(:,:,1) = d;
        pp = mkpp(obj.t_knot,coefs,3);
      elseif(obj.com_traj_order == 4)
        error('Not supported any more');
        a = com(:,1:end-1);
        b = comdot(:,1:end-1);
        c = 0.5*comddot(:,1:end-1);
        d = (4*com(:,2:end)-4*com(:,1:end-1)-(3*comdot(:,1:end-1)+comdot(:,2:end)).*bsxfun(@times,ones(3,1),dt)-comddot(:,1:end-1).*bsxfun(@times,ones(3,1),dt.^2))./bsxfun(@times,ones(3,1),dt.^3);
        e = ((comdot(:,2:end)+2*comdot(:,1:end-1)).*bsxfun(@times,ones(3,1),dt)+0.5*comddot(:,1:end-1).*bsxfun(@times,ones(3,1),dt.^2)-3*com(:,2:end)+3*com(:,1:end-1))./bsxfun(@times,ones(3,1),dt.^4);
        coefs = zeros(3,obj.nT-1,5);
        coefs(:,:,5) = a;
        coefs(:,:,4) = b;
        coefs(:,:,3) = c;
        coefs(:,:,2) = d;
        coefs(:,:,1) = e;
        pp = mkpp(obj.t_knot,coefs,3);
      end
      com_traj = PPTrajectory(pp);
      com_traj = com_traj.setOutputFrame(obj.com_frame);
    end
    
    function [force_traj,force_traj_stack] = forceTrajectory(obj,F)
      % @param F   The force weight 
      % @retval force_traj   A obj.num_fsrc_cnstr x 1 cell of trajectories.
      % @retval force_traj_stack   Same as force_traj, except force_traj_stack.eval(t) =
      % reshape(force_traj.eval(t),[],1)
      force_traj = cell(obj.num_fsrc_cnstr,1);
      force_traj_stack = cell(obj.num_fsrc_cnstr,1);
      force = cell(obj.num_fsrc_cnstr,1);
      force_stack = cell(obj.num_fsrc_cnstr,1);
      for i = 1:obj.num_fsrc_cnstr
        force{i} = zeros(3,obj.fsrc_cnstr(i).num_contact_pts,length(obj.fsrc_knot_active_idx{i}));
      end
      for i = 1:obj.nT
        for j = 1:length(obj.F2fsrc_map{i})
          fsrc_idx = obj.F2fsrc_map{i}(j);
          force{fsrc_idx}(:,:,i==obj.fsrc_knot_active_idx{fsrc_idx}) = obj.A_force{fsrc_idx}*F{i}{j};
        end
      end
      for i = 1:obj.num_fsrc_cnstr
        force_stack{i} = zeros(3*obj.fsrc_cnstr(i).num_contact_pts,obj.nT);
        force_stack{i}(:,obj.fsrc_knot_active_idx{i}) = reshape(force{i},[],length(obj.fsrc_knot_active_idx{i}));
        force_traj{i} = PPTrajectory(foh(obj.t_knot(obj.fsrc_knot_active_idx{i}),force{i}));
        force_traj_stack{i} = PPTrajectory(foh(obj.t_knot,force_stack{i}));
      end
    end
    
    function visualize(obj,com,comdot,comddot,foot_pos,F,Hdot)
      com_traj = obj.CoMPPTraj(com,comdot,comddot);
      [~,force_traj_stack] = obj.forceTrajectory(F);
      visualizer_traj = com_traj;
      for i = 1:obj.num_fsrc_cnstr
        fsrc_pos_traj = PPTrajectory(zoh(obj.t_knot,bsxfun(@times,[foot_pos(:,i);obj.yaw(i)],ones(1,obj.nT))));
        fsrc_traj = [fsrc_pos_traj;force_traj_stack{i}];
        fsrc_traj = fsrc_traj.setOutputFrame(obj.fsrc_frame{i});
        visualizer_traj = [visualizer_traj;fsrc_traj];
      end
      zmp_traj = obj.ZMPTrajectory(com,comddot,Hdot);
      visualizer_traj = [visualizer_traj;zmp_traj];
      obj.visualizer.viz{1} = obj.visualizer.viz{1}.setCoMSamples(com);
      zmp = zmp_traj.eval(obj.t_knot);
      obj.visualizer.viz{end} = obj.visualizer.viz{end}.setZMPSamples(zmp);
      obj.visualizer.playback(visualizer_traj,struct('slider',true));
    end
    
    function zmp_traj = ZMPTrajectory(obj,com,comddot,Hdot)
      zmp = [(-Hdot(2,:)+obj.robot_mass*obj.g*com(1,:)-obj.robot_mass*com(3,:).*comddot(1,:)+obj.robot_mass*com(1,:).*comddot(3,:))./(obj.robot_mass*comddot(3,:)+obj.robot_mass*obj.g);...
        (Hdot(1,:)+obj.robot_mass*com(2,:).*comddot(3,:)-obj.robot_mass*com(3,:).*comddot(2,:)+obj.robot_mass*obj.g*com(2,:))./(obj.robot_mass*comddot(3,:)+obj.robot_mass*obj.g)];
      zmp_traj = PPTrajectory(foh(obj.t_knot,zmp));
      zmp_traj = zmp_traj.setOutputFrame(obj.zmp_frame);
    end
  end
  
  
end
