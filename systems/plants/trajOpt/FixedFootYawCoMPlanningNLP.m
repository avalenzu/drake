classdef FixedFootYawCoMPlanningNLP < FixedFootYawCoMPlanningSeed
  % This planner search for the CoM and contact force simultaneously. We want to make sure
  % that all the constraints are linear, only the cost function is nonlinear. The time is
  % fixed for the trajectory.
  properties(SetAccess = protected)
    H0_idx % A 3 x 1 vector. x(H0_idx) is the initial angular momentum
    F_idx_all % The indices of all the forces;
    angular_cost % A AngularMomentumCost object
  end
  
  methods
    function obj = FixedFootYawCoMPlanningNLP(robot_mass,robot_dim,t,g,lambda,c_margin,Q_comddot,fsrc_cnstr,yaw,F2fsrc_map,fsrc_knot_active_idx,A_force,A_xy,b_xy,rotmat,com_traj_order)
      % @param robot_mass  The mass of the robot
      % @param robot_dim    An estimation of the dimension of the robot in meters.
      % @param t   The time knot for planning. 
      % @param g   The gravitational acceleration
      % @param c_margin  A positive scalar. The weight of the force margin in the
      % objective. We will minimize -c_margin*margin
      % @param lambda   A 3 x 3 Hurwitz matrix.
      % @param Q_comddot  A 3 x 3 PSD matrix. This puts a quadratic cost on the CoM
      % acceleration
      % @param fsrc_cnstr  A cell array. All the FootStepRegionContactConstraint object
      % @param fsrc_knot_active_idx   A cell array. fsrc_knot_active_idx{i} is the indices of the knots that are active for i'th FootStepRegionContactConstraint
      % @param yaw     A 1 x num_fsrc_cnstr double vector. yaw(i) is the yaw angle for obj.fsrc_cnstr{i}
      % @param A_xy,b_xy,rotmat   A_xy is 3 x 2 x obj.num_fsrc_cnstr matrix. b_xy is 3 x 1 x obj.num_fsrc_cnstr matrix. rotmat is 3 x 3 x obj.num_fsrc_cnstr matrix. [rotmat(:,:,i),A_xy(:,:,i),b_xy(:,:,i)] = obj.fsrc_cnstr{i}.bodyTransform(obj.yaw(i)); 
      % @param A_force    A_force{i} = obj.fsrc_cnstr{i}.force, which is a 3 x obj.fsrc_cnstr[i}.num_edges matrix
      obj = obj@FixedFootYawCoMPlanningSeed(robot_mass,robot_dim,t,g,lambda,c_margin,Q_comddot,fsrc_cnstr,yaw,F2fsrc_map,fsrc_knot_active_idx,A_force,A_xy,b_xy,rotmat,com_traj_order);
      obj.H0_idx = obj.num_vars+(1:3)';
      H0_name = {'H0_x';'H0_y';'H0_z'};
      obj = obj.addDecisionVariable(3,H0_name);
      obj.angular_cost = AngularMomentumCost(obj.robot_mass,obj.robot_dim,obj.nT,obj.g,obj.lambda,obj.num_force_weight,obj.fsrc_cnstr,obj.yaw,obj.F2fsrc_map,obj.fsrc_knot_active_idx,obj.A_force,obj.A_xy,obj.b_xy,obj.rotmat);
      obj.F_idx_all = zeros(obj.num_force_weight,1);
      F_count = 0;
      for i = 1:obj.nT
        for j = 1:length(obj.F_idx{i})
          obj.F_idx_all(F_count+(1:numel(obj.F_idx{i}{j}))) = obj.F_idx{i}{j}(:);
          F_count = F_count+numel(obj.F_idx{i}{j});
        end
      end
      obj = obj.addCost(obj.angular_cost,[obj.com_idx(:);obj.H0_idx;obj.fsrc_body_pos_idx(:);obj.F_idx_all]);
      margin_cost = LinearConstraint(-inf,inf,-c_margin*ones(1,obj.nT));
      obj = obj.addCost(margin_cost,obj.margin_idx);
      comddot_cost = QuadraticSumConstraint(-inf,inf,Q_comddot,zeros(3,obj.nT));
       obj = obj.addCost(comddot_cost,obj.comddot_idx(:));
    end
    
    function [com,comdot,comddot,foot_pos,F,Hdot,H,epsilon,sigma,INFO] = solve(obj,com,comdot,comddot,foot_pos,F,margin,H0)
      x = inf(obj.num_vars,1);
      x(obj.com_idx) = com(:);
      x(obj.comdot_idx) = comdot(:);
      x(obj.comddot_idx) = comddot(:);
      x(obj.margin_idx) = margin(:);
      x(obj.fsrc_body_pos_idx) = foot_pos(:);
      for i = 1:obj.nT
        for j = 1:length(obj.F_idx{i})
          x(obj.F_idx{i}{j}) = F{i}{j}(:);
        end
      end
      H_scale = obj.robot_mass*obj.robot_dim*obj.g;
      x(obj.H0_idx) = H0/H_scale;
      
      [x_sol,objective,INFO] = solve@NonlinearProgramWConstraintObjects(obj,x);
      com = reshape(x_sol(obj.com_idx),3,obj.nT);
      comdot = reshape(x_sol(obj.comdot_idx),3,obj.nT);
      comddot = reshape(x_sol(obj.comddot_idx),3,obj.nT);
      foot_pos = reshape(x_sol(obj.fsrc_body_pos_idx),2,obj.num_fsrc_cnstr);
      F = cell(1,obj.nT);
      Hdot = zeros(3,obj.nT);
      H0 = x_sol(obj.H0_idx)*H_scale;
      for i = 1:obj.nT
        F{i} = cell(1,length(obj.F_idx{i}));
        for j = 1:length(obj.F_idx{i})
          fsrc_idx = obj.F2fsrc_map{i}(j);
          num_contact_pts_ij = obj.fsrc_cnstr{fsrc_idx}.num_contact_pts;
          F{i}{j} = reshape(x_sol(obj.F_idx{i}{j}),obj.fsrc_cnstr{fsrc_idx}.num_edges,obj.fsrc_cnstr{fsrc_idx}.num_contact_pts);
          force_ij = obj.A_force{fsrc_idx}*F{i}{j};
          contact_pos = obj.rotmat(:,:,fsrc_idx)*obj.fsrc_cnstr{fsrc_idx}.body_contact_pts+bsxfun(@times,ones(1,num_contact_pts_ij),obj.A_xy(:,:,fsrc_idx)*foot_pos(:,fsrc_idx)+obj.b_xy(:,:,fsrc_idx));
          contact_pos_CoM = contact_pos-bsxfun(@times,com(:,i),ones(1,num_contact_pts_ij));
          Hdot(:,i) = Hdot(:,i)+sum(cross(contact_pos_CoM,force_ij),2);
        end
      end
      dt = diff(obj.t_knot);
      
      H = cumsum([H0 Hdot(:,2:end).*bsxfun(@times,ones(3,1),dt)],2);
      epsilon = (Hdot/H_scale-obj.lambda*H/H_scale);
      sigma = sum(sum(epsilon.^2));
    end
  end
end