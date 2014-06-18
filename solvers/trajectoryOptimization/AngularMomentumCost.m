classdef AngularMomentumCost < NonlinearConstraint
  % Cost of sum_i Hdot(:,i)'*Q_Hdot*Hdot(:,i)+H(:,i)'*Q_H*H(:,i)
  properties(SetAccess = protected)
    robot_mass % The mass of the robot
    robot_dim % An estimate of the robot dimension in meters
    g % The gravitational acceleration
    t_knot % The time knots
    nT % The length of obj.t_knot
    com_idx % A 3 x obj.nT matrix. x(com_idx(:,i)) is the CoM position at the i'th knot
    F_idx % A cell array. x(F_idx{i}(j)(:,k)) is the contact force parameters (the weights for the friction cone extreme rays) at time t_knot(i), for the j'th FootStepRegionContactConstraint, at k'th contact point
    H0_idx % A 3 x 1 vector. x(H0_idx) is the angular momentum at the beginning.
    
    lambda % A 3 x 3 Hurwitz matrix
    
    num_fsrc_cnstr  % A scalar. The total number of FootStepRegionContactConstraint
    fsrc_cnstr % An array of FootStepRegionContactConstraint object
    fsrc_body_pos_idx % A 2 x length(fsrc_cnstr) matrix. x(obj.fsrc_body_pos_idx(:,i)) is the body position for the i'th FootStepRegionContactConstraint in the decision variables.
    F2fsrc_map % A cell array. obj.fsrc_cnstr(F2fsrc_map{i}(j)) is the FootStepContactRegionConstraint corresponds to the force x(obj.F_idx{i}{j})
    fsrc_knot_active_idx % A cell array. fsrc_knot_active_idx{i} is the indices of the knots that are active for i'th FootStepRegionContactConstraint
    yaw % A 1 x num_fsrc_cnstr double vector. yaw(i) is the yaw angle for obj.fsrc_cnstr(i)
    H_des  % A 3 x obj.nT matrix. The desired angular momentum
  end
  
  properties(Access = protected)
    A_force % A cell array.  A_force{i} = obj.fsrc_cnstr(i).force, which is a 3 x obj.fsrc_cnstr(i).num_edges matrix
    A_xy,b_xy,rotmat  % A_xy is 3 x 2 x obj.num_fsrc_cnstr matrix. b_xy is 3 x 1 x obj.num_fsrc_cnstr matrix. rotmat is 3 x 3 x obj.num_fsrc_cnstr matrix. [rotmat(:,:,i),A_xy(:,:,i),b_xy(:,:,i)] = obj.fsrc_cnstr(i).bodyTransform(obj.yaw(i)); 
    num_force_weight % A scalar. The total number of force weights.
  end
  
  methods
    function obj = AngularMomentumCost(robot_mass,robot_dim,t_knot,g,lambda,num_force_weight,fsrc_cnstr,yaw,F2fsrc_map,fsrc_knot_active_idx,A_force,A_xy,b_xy,rotmat,H_des)
      % @param robot_mass  The mass of the robot
      % @param robot_dim    An estimation of the dimension of the robot in meters.
      % @param t_knot   The time knots
      % @param g   The gravitational acceleration
      % @param lambda   A 3 x 3 Hurwitz matrix
      % @param num_force_weight   The total number of force weights
      % @param fsrc_cnstr  An array of the FootStepRegionContactConstraint object
      % @param fsrc_knot_active_idx   A cell array. fsrc_knot_active_idx{i} is the indices of the knots that are active for i'th FootStepRegionContactConstraint
      % @param yaw     A 1 x num_fsrc_cnstr double vector. yaw(i) is the yaw angle for
      % obj.fsrc_cnstr(i)
      % @param A_xy,b_xy,rotmat   A_xy is 3 x 2 x obj.num_fsrc_cnstr matrix. b_xy is 3 x 1 x obj.num_fsrc_cnstr matrix. rotmat is 3 x 3 x obj.num_fsrc_cnstr matrix. [rotmat(:,:,i),A_xy(:,:,i),b_xy(:,:,i)] = obj.fsrc_cnstr(i).bodyTransform(obj.yaw(i)); 
      % @param A_force    A_force{i} = obj.fsrc_cnstr(i).force, which is a 3 x obj.fsrc_cnstr(i).num_edges matrix
      % @param H_des   A 3 x obj.nT matrix. The desired angular momentum
      obj = obj@NonlinearConstraint(-inf,inf,num_force_weight+3+3*length(t_knot)+2*length(fsrc_cnstr));
      obj.robot_mass = robot_mass;
      obj.robot_dim = robot_dim;
      obj.t_knot = t_knot;
      obj.nT = length(obj.t_knot);
      obj.g = g;
      obj.lambda = lambda;
      obj.num_fsrc_cnstr = length(fsrc_cnstr);
      obj.fsrc_cnstr = fsrc_cnstr;
      obj.yaw = yaw;
      obj.F2fsrc_map = F2fsrc_map;
      obj.fsrc_knot_active_idx = fsrc_knot_active_idx;
      obj.A_force = A_force;
      obj.A_xy = A_xy;
      obj.b_xy = b_xy;
      obj.rotmat = rotmat;
      obj.H_des = H_des;
      
      var_count = 0;
      obj.com_idx = var_count+reshape(1:3*obj.nT,3,obj.nT);
      var_count = var_count+3*obj.nT;
      obj.H0_idx = var_count+(1:3)';
      var_count = var_count +3;
      obj.fsrc_body_pos_idx = var_count+reshape(1:2*obj.num_fsrc_cnstr,2,obj.num_fsrc_cnstr);
      var_count = var_count+2*obj.num_fsrc_cnstr;
      obj.F_idx = cell(1,obj.nT);
      for i = 1:obj.nT
        obj.F_idx{i} = cell(1,length(obj.F2fsrc_map{i}));
        for j = 1:length(obj.F2fsrc_map{i})          
          fsrc_idx = obj.F2fsrc_map{i}(j);
          obj.F_idx{i}{j} =  var_count+reshape((1:obj.fsrc_cnstr(fsrc_idx).num_force_weight),obj.fsrc_cnstr(fsrc_idx).num_edges,obj.fsrc_cnstr(fsrc_idx).num_contact_pts);
          var_count = var_count+obj.fsrc_cnstr(fsrc_idx).num_force_weight;
        end
      end
    end
    
    function [c,dc] = eval(obj,x)
      % @param x    x = [com(:);H0;foot_pos(:),F];
      com = reshape(x(obj.com_idx(:)),3,obj.nT);
      H0bar = x(obj.H0_idx);
      dH0bar = zeros(3,obj.xdim);
      dH0bar(:,obj.H0_idx) = eye(3);
      foot_pos = reshape(x(obj.fsrc_body_pos_idx),2,obj.num_fsrc_cnstr);
      F = cell(1,obj.nT);
      Hdot = zeros(3,obj.nT);
      dHdot = zeros(3*obj.nT,obj.xdim);
      for i = 1:obj.nT
        F{i} = cell(1,length(obj.F_idx{i}));
        for j = 1:length(obj.F_idx{i})
          fsrc_idx = obj.F2fsrc_map{i}(j);
          num_contact_pts_ij = obj.fsrc_cnstr(fsrc_idx).num_contact_pts;
          num_edges_ij = obj.fsrc_cnstr(fsrc_idx).num_edges;
          F{i}{j} = reshape(x(obj.F_idx{i}{j}),num_edges_ij,num_contact_pts_ij);
          force_ij = obj.A_force{fsrc_idx}*F{i}{j};
          dforce_ij_row = reshape(bsxfun(@plus,reshape(bsxfun(@times,(1:3)',ones(1,num_edges_ij)),[],1),3*(0:num_contact_pts_ij-1)),[],1);
          dforce_ij_col = reshape(bsxfun(@times,1:obj.fsrc_cnstr(fsrc_idx).num_force_weight,ones(3,1)),[],1);
          dforce_ij_val = reshape(bsxfun(@times,reshape(obj.A_force{fsrc_idx},[],1),ones(1,num_contact_pts_ij)),[],1);
          dforce_ij = sparse(dforce_ij_row,dforce_ij_col,dforce_ij_val,3*num_contact_pts_ij,obj.fsrc_cnstr(fsrc_idx).num_force_weight);
          contact_pos = obj.rotmat(:,:,fsrc_idx)*obj.fsrc_cnstr(fsrc_idx).body_contact_pts+bsxfun(@times,ones(1,num_contact_pts_ij),obj.A_xy(:,:,fsrc_idx)*foot_pos(:,fsrc_idx)+obj.b_xy(:,:,fsrc_idx));
          dcontact_pos = obj.A_xy(:,:,fsrc_idx);
          contact_pos_CoM = contact_pos-bsxfun(@times,com(:,i),ones(1,num_contact_pts_ij));
          Hdot(:,i) = Hdot(:,i)+sum(cross(contact_pos_CoM,force_ij),2);
          skew_contact_pos_CoM_row = reshape(bsxfun(@times,[1;1;2;2;3;3],ones(1,num_contact_pts_ij)),[],1);
          skew_contact_pos_CoM_col = reshape(bsxfun(@plus,[2;3;1;3;1;2],3*(0:num_contact_pts_ij-1)),[],1);
          skew_contact_pos_CoM_val = reshape([-contact_pos_CoM(3,:);contact_pos_CoM(2,:);contact_pos_CoM(3,:);-contact_pos_CoM(1,:);-contact_pos_CoM(2,:);contact_pos_CoM(1,:)],[],1);
          skew_contact_pos_CoM = sparse(skew_contact_pos_CoM_row,skew_contact_pos_CoM_col,skew_contact_pos_CoM_val,3,3*num_contact_pts_ij);
          dHdot((i-1)*3+(1:3),obj.F_idx{i}{j}) = skew_contact_pos_CoM*dforce_ij;
          force_ij_total = sum(force_ij,2);
          dHdot((i-1)*3+(1:3),obj.fsrc_body_pos_idx(:,fsrc_idx)) = dHdot((i-1)*3+(1:3),obj.fsrc_body_pos_idx(:,fsrc_idx))...
            -[0 -force_ij_total(3) force_ij_total(2);force_ij_total(3) 0 -force_ij_total(1);-force_ij_total(2) force_ij_total(1) 0]*dcontact_pos;
          dHdot((i-1)*3+(1:3),obj.com_idx(:,i)) = dHdot((i-1)*3+(1:3),obj.com_idx(:,i))...
            +[0 -force_ij_total(3) force_ij_total(2);force_ij_total(3) 0 -force_ij_total(1);-force_ij_total(2) force_ij_total(1) 0];
        end
      end
      scaling_factor = obj.robot_mass*obj.g*obj.robot_dim;
      Hdot_bar = Hdot/scaling_factor;
      dHdot_bar = dHdot/scaling_factor;
      dt = diff(obj.t_knot);
      Hbar = cumsum([H0bar Hdot_bar(:,2:end).*bsxfun(@times,ones(3,1),dt)],2);
      dHbar = reshape(cumsum(reshape(([dH0bar;dHdot_bar(4:end,:).*bsxfun(@times,ones(1,obj.xdim),reshape(bsxfun(@times,ones(3,1),dt),[],1))])',3,obj.xdim,obj.nT),3),obj.xdim,3*obj.nT)';      
      Hbar_des = obj.H_des/scaling_factor;
      epsilon = obj.lambda*(Hbar-Hbar_des)-Hdot_bar;
      depsilon = sparse(reshape(bsxfun(@plus,[1;2;3;1;2;3;1;2;3],3*(0:obj.nT-1)),[],1),...
        reshape(bsxfun(@times,(1:3*obj.nT),ones(3,1)),[],1),reshape(bsxfun(@times,obj.lambda(:),ones(1,obj.nT)),[],1),...
        3*obj.nT,3*obj.nT)*dHbar-dHdot_bar;
      c = sum(sum(epsilon.^2));
      dc = 2*epsilon(:)'*depsilon;
    end
  end
end