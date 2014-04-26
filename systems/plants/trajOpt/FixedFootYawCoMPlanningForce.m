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
    num_vars % The total number of decision variables in the SOCP 
    x_names % A obj.num_vars x 1 double vector. The lower bound for the decision variable x
    x_lb % A obj.num_vars x 1 double vector. The lower bound for the decision variable x
    x_ub % A obj.num_vars x 1 double vector. The upper bound for the decision variable x
    Q_cost % A obj.num_vars x obj.num_vars sparse PSD matrix. The Hessian of the quadratic cost
    A_H,A_H_bnd % A_H * x = A_H_bnd is the constraint on the euler integraton of angular momentum
    A_angular_PD,A_angular_PD_bnd % A_angular_PD * x = A_angular_PD_bnd is the constraint Hdot[n] = lambda*H[n]+epsilon[n]
    num_fsrc_cnstr  % A scalar. The total number of FootStepRegionContactConstraint
    fsrc_cnstr % A cell array. All the FootStepRegionContactConstraint object
    
    F2fsrc_map % A cell array. obj.fsrc_cnstr{F2fsrc_map{i}(j)} is the FootStepContactRegionConstraint corresponds to the force x(obj.F_idx{i}{j})
    yaw % A 1 x num_fsrc_cnstr double vector. yaw(i) is the yaw angle for obj.fsrc_cnstr{i}
    
    A_com
  end
  
  properties(Access = protected)
    A_force % A cell array . A_force{i} = obj.fsrc_cnstr{i}.force, which is a 3 x obj.fsrc_cnstr[i}.num_edges matrix
    A_xy,b_xy,rotmat % A_xy is 3 x 2 x obj.num_fsrc_cnstr matrix. b_xy is 3 x 1 x obj.num_fsrc_cnstr matrix. rotmat is 3 x 3 x obj.num_fsrc_cnstr matrix. [rotmat(:,:,i),A_xy(:,:,i),b_xy(:,:,i)] = obj.fsrc_cnstr{i}.bodyTransform(obj.yaw(i)); 
  end
  
  methods
    function obj = FixedFootYawCoMPlanningForce(robot_mass,t,lambda,c_margin,varargin)
      % obj =
      % FixedFootYawCoMPlanningForce(robot_mass,t,lambda,c_margin,foot_step_region_contact_cnstr1,yaw1,foot_step_region_contact_cnstr2,yaw2,...)
      % @param robot_mass  The mass of the robot
      % @param t   The time knot for planning
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
      if(~isnumeric(c_weight))
        error('Drake:FixedFootYawCoMPlanningForce:c_weight should be numeric');
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
      
      obj.num_fsrc_cnstr = length(varargin)/2;
      obj.fsrc_cnstr = cell(1,obj.num_fsrc_cnstr);
      obj.F2fsrc_map = cell(1,obj.nT);
      obj.yaw = zeros(1,obj.num_fsrc_cnstr);
      obj.F_idx = cell(1,obj.nT);
      obj.A_xy = zeros(3,2,obj.num_fsrc_cnstr);
      obj.b_xy = zeros(3,1,obj.num_fsrc_cnstr);
      obj.rotmat = zeros(3,3,obj.num_fsrc_cnstr);
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
        is_fsrc_active = false;
        [obj.rotmat(:,:,i),obj.A_xy(:,:,i),obj.b_xy(:,:,i)] = varargin{2*i-1}.foot_step_region_cnstr.bodyTransform(varargin{2*i});
        for j = 1:obj.nT
          if(varargin{2*i-1}.foot_step_region_cnstr.isTimeValid(obj.t_knot(j)))
            obj.F_idx{j} = [obj.F_idx{j} {obj.num_vars+reshape((1:varargin{2*i-1}.num_force_weight),varargin{2*i-1}.num_edges,varargin{2*i-1}.num_contact_pts)}];
            is_fsrc_active = true;
          end
        end
        if(~is_fsrc_active)
          error('Drake:FixedFootYawCoMPlanningForce:The %dth FootStepRegionContactConstraint is not active for any t_knot');
        end
      end
    end
  end
end