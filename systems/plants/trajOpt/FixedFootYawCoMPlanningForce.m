classdef FixedFootYawCoMPlanningForce < NonlinearProgramWConstraintObjects
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
    sigma_idx % An integer scalar. x(sigma_idx) is the index of sigma, which is the dummy variable used to bound the PD residue: sum_n epsilon[n]'*epsilon[n] <= sigma  
    sdotsquare_idx  % A 1 x obj.nT vector. x(sdotsquare_idx(i)) is the square of the time scaling function s at the i'th knot point.
    tau_idx % A 1 x obj.nT vector. x(tau_idx(i)) is the dummy variable satisfies 4*sdotsquare[i]>=tau[i]^2;
    sdotsquareplus_idx % A 1 x obj.nT vector. x(sdotsquareplus_idx(i)) = sdotsquare[i]+1. This dummy variable is used in the upper bound of dt
    sdotsquareminus_idx % A 1 x obj.nT vector. x(sdotsquareminus_idx(i)) = sdotsquare[i]-1. This dummy variable is used in the upper bound of dt
    Q_cost % A obj.num_vars x obj.num_vars sparse PSD matrix. The Hessian of the quadratic cost
    f_cost % A obj.num_vars x 1 vector. The linear componenet of the cost
    A_H,A_H_bnd % A_H * x = A_H_bnd is the constraint on the euler integraton of angular momentum
    A_angular_PD,A_angular_PD_bnd % A_angular_PD * x = A_angular_PD_bnd is the constraint Hdot[n] = lambda*H[n]+epsilon[n]
    num_fsrc_cnstr  % A scalar. The total number of FootStepRegionContactConstraint
    fsrc_cnstr % A cell array. All the FootStepRegionContactConstraint object
    
    F2fsrc_map % A cell array. obj.fsrc_cnstr{F2fsrc_map{i}(j)} is the FootStepContactRegionConstraint corresponds to the force x(obj.F_idx{i}{j})
    fsrc_knot_active_idx % A cell array. fsrc_knot_active_idx{i} is the indices of the knots that are active for i'th FootStepRegionContactConstraint
    yaw % A 1 x num_fsrc_cnstr double vector. yaw(i) is the yaw angle for obj.fsrc_cnstr{i}
    
    A_newton % A 3*obj.nT x obj.num_vars matrix. force(:) = A_newton*x, where force(:,i) is the total force at i'th knot point
    A_margin % A obj.nT x obj.num_vars matrix. 
    A_margin_bnd % A obj.nT x 1 vector. A_margin*x<=A_margin_bnd represents margin(i)<= min(F_i), where F_i are all the force weights at the k'th knot point
    A_dt % A obj.nT-1 x obj.num_vars sparse matrix. 
    A_dt_bnd % A obj.nT-1 x 1 vector. A_dt*x<=A_dt_bnd encodes the upper bound dt, the time interval between two consecutive knot points
    A_sdotsquare % A 2*obj.nT x obj.num_vars sparse matrix.
    A_sdotsquare_bnd % A 2*obj.nT x 1 vector. A_sdotsquare*x=A_sdotsquare_bnd encodes the constraint that sdotsquareplus[i] = sdotsquare[i]+1 and sdotsquareminus[i] = sdotsquare[i]-1
    cone_dt_idx  % A obj.nT x 3 integer matrix. cone_dt_idx(i,:) is the index of the second order cone, such that the SOCP constraint x(obj.cone_dt_idx(i,1))>=sqrt(sum x(obj.cone_dt_idx(i,2:end)).^2), which would enforce the upper bound on the time interval dt.
    lambda % A 3 x 3 Hurwitz matrix
    dt_max %  A positive scalar. The upperbound for the time interval between any two consecutive knot points
    sdot_max % A positive scalar. The upper bound for the derivitive of time scaling funtion s w.r.t time.
    
    robot_dim % % The approximate dimension of the robot in meters. This is used to scale the constraint
  end
  
  properties(Access = protected)
    A_force % A cell array . A_force{i} = obj.fsrc_cnstr{i}.force, which is a 3 x obj.fsrc_cnstr[i}.num_edges matrix
    A_xy,b_xy,rotmat % A_xy is 3 x 2 x obj.num_fsrc_cnstr matrix. b_xy is 3 x 1 x obj.num_fsrc_cnstr matrix. rotmat is 3 x 3 x obj.num_fsrc_cnstr matrix. [rotmat(:,:,i),A_xy(:,:,i),b_xy(:,:,i)] = obj.fsrc_cnstr{i}.bodyTransform(obj.yaw(i)); 
    num_force_weight % A scalar. The total number of force weights.
  end
  
  methods
    function obj = FixedFootYawCoMPlanningForce(robot_mass,robot_dim,t,g,lambda,c_margin,dt_max,sdot_max,fsrc_cnstr,yaw,F2fsrc_map,fsrc_knot_active_idx,A_force,A_xy,b_xy,rotmat)
      % @param robot_mass  The mass of the robot
      % @param t   The time knot for planning. This indicates which
      % FootStepRegionContactConstraint is active at a given time knot. The actual time is
      % determined by the scaling function.
      % @param g             The gravitational acceleration
      % @param lambda        A 3 x 3 Hurwitz matrix. It tries to drive the angular
      % momentum stays at 0 by putting the constraint Hdot[n] = lambda*H[n]+epsilon[n]
      % @param c_margin  A positive scalar. The weight of the force margin in the
      % objective. The objective function is sum_n lambda[n]'*lambda[n]-c_weight*margin[n]
      % @param dt_max   A positive scalar. The upperbound for the time interval between
      % any two consecutive knot points
      % @param sdot_max  A positive scalar. The upper bound for the derivitive of time scaling funtion s
      % w.r.t time.
      % @param fsrc_cnstr  A cell array. All the FootStepRegionContactConstraint object
      % @param fsrc_knot_active_idx   A cell array. fsrc_knot_active_idx{i} is the indices of the knots that are active for i'th FootStepRegionContactConstraint
      % @param yaw     A 1 x num_fsrc_cnstr double vector. yaw(i) is the yaw angle for obj.fsrc_cnstr{i}
      % @param A_xy,b_xy,rotmat   A_xy is 3 x 2 x obj.num_fsrc_cnstr matrix. b_xy is 3 x 1 x obj.num_fsrc_cnstr matrix. rotmat is 3 x 3 x obj.num_fsrc_cnstr matrix. [rotmat(:,:,i),A_xy(:,:,i),b_xy(:,:,i)] = obj.fsrc_cnstr{i}.bodyTransform(obj.yaw(i)); 
      % @param A_force    A_force{i} = obj.fsrc_cnstr{i}.force, which is a 3 x obj.fsrc_cnstr[i}.num_edges matrix
      obj = obj@NonlinearProgramWConstraintObjects(0);
      obj.robot_mass = robot_mass;
      obj.robot_dim = robot_dim;
      obj.t_knot = t;
      obj.nT = length(obj.t_knot);
      obj.g = g;
      obj.lambda = lambda;
      obj.dt_max = dt_max;
      delta_s = 1/(obj.nT-1);
      if(sdot_max<delta_s/obj.dt_max)
        error('Drake:FixedFootYawCoMPlanningForce:sdot_max is too small, not compatible with dt_max');
      end
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
      
      obj.sdotsquare_idx = obj.num_vars+(1:obj.nT);
      sdotsquare_names = cell(obj.nT,1);
      for i = 1:obj.nT
        sdotsquare_names{i} = sprintf('sdotsquare[%d]',i);
      end
      obj = obj.addDecisionVariable(obj.nT,sdotsquare_names);
      
      obj.margin_idx = obj.num_vars+(1:obj.nT);
      margin_names = cell(obj.nT,1);
      for i = 1:obj.nT
        margin_names{i} = sprintf('force_margin[%d]',i);
      end
      obj = obj.addDecisionVariable(obj.nT,margin_names);
      
      obj.tau_idx = obj.num_vars+(1:obj.nT);
      tau_names = cell(obj.nT,1);
      for i = 1:obj.nT
        tau_names{i} = sprintf('tau[%d]',i);
      end
      obj = obj.addDecisionVariable(obj.nT,tau_names);
      
      obj.sdotsquareplus_idx = obj.num_vars+(1:obj.nT);
      sdotsquareplus_names = cell(obj.nT,1);
      for i = 1:obj.nT
        sdotsquareplus_names{i} = sprintf('sdotsquareplus[%d]',i);
      end
      obj = obj.addDecisionVariable(obj.nT,sdotsquareplus_names);
      
      obj.sdotsquareminus_idx = obj.num_vars+(1:obj.nT);
      sdotsquareminus_names = cell(obj.nT,1);
      for i = 1:obj.nT
        sdotsquareminus_names{i} = sprintf('sdotsquareminus[%d]',i);
      end
      obj = obj.addDecisionVariable(obj.nT,sdotsquareminus_names);
      
      obj.F_idx = cell(1,obj.nT);
      obj.num_force_weight = 0;
      for i = 1:obj.num_fsrc_cnstr
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
            obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(zeros(obj.fsrc_cnstr{i}.num_force_weight,1),inf(obj.fsrc_cnstr{i}.num_force_weight,1)),obj.F_idx{j}{end}(:));
            obj.num_force_weight = obj.num_force_weight+obj.fsrc_cnstr{i}.num_force_weight;
          end
        end
      end
      
      % linear constraint that H[n]-H[n-1] = Hdot[n]*delta_s
      iA_H = [(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';(1:3*(obj.nT-1))';];
      jA_H = [reshape(obj.H_idx(:,2:end),[],1);reshape(obj.H_idx(:,1:end-1),[],1); reshape(obj.Hdot_idx(:,2:end),[],1)];
      Aval_H = [ones(3*(obj.nT-1),1);-ones(3*(obj.nT-1),1);-reshape(bsxfun(@times,ones(3,1),delta_s*ones(1,obj.nT-1)),[],1)];
      obj.A_H = sparse(iA_H,jA_H,Aval_H,3*(obj.nT-1),obj.num_vars);
      obj.A_H_bnd = zeros(3*(obj.nT-1),1);
      A_H_names =repmat({'dHds=Hdot*ds'},3*(obj.nT-1),1);
      obj = obj.addLinearConstraint(LinearConstraint(obj.A_H_bnd,obj.A_H_bnd,obj.A_H),(1:obj.num_vars),A_H_names);
      % linear constraint that Hdot[n] = lambda*H[n]+epsilon[n]
      iA_angular_PD = [(1:3*obj.nT)';reshape(repmat(reshape((1:3*obj.nT),3,obj.nT),3,1),[],1);(1:3*obj.nT)'];
      jA_angular_PD = [obj.Hdot_idx(:);reshape(bsxfun(@times,obj.H_idx(:)',ones(3,1)),[],1);obj.epsilon_idx(:)];
      Aval_angular_PD = [ones(3*obj.nT,1);reshape(repmat(-obj.lambda,1,obj.nT),[],1);-ones(3*obj.nT,1)];
      obj.A_angular_PD = sparse(iA_angular_PD,jA_angular_PD,Aval_angular_PD,3*obj.nT,obj.num_vars);
      obj.A_angular_PD_bnd = zeros(3*obj.nT,1);
      A_angular_PD_names = repmat({'PD law of Hdot'},3*obj.nT,1);
      obj = obj.addLinearConstraint(LinearConstraint(obj.A_angular_PD_bnd,obj.A_angular_PD_bnd,obj.A_angular_PD),(1:obj.num_vars),A_angular_PD_names);
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
      A_margin_names = repmat({'force margin'},obj.num_force_weight,1);
      obj = obj.addLinearConstraint(LinearConstraint(-inf(obj.num_force_weight,1),zeros(obj.num_force_weight,1),obj.A_margin),(1:obj.num_vars),A_margin_names);
      % add the constraint that would enforce the time interval between two consecutive
      % knot points to be upper bounded.
      iAdt = [(1:obj.nT-1)';(1:obj.nT-1)'];
      jAdt = [obj.tau_idx(1:end-1)';obj.tau_idx(2:end)'];
      Aval_dt = -ones(2*(obj.nT-1),1);
      obj.A_dt = sparse(iAdt,jAdt,Aval_dt,obj.nT-1,obj.num_vars);
      obj.A_dt_bnd = -delta_s*4/obj.dt_max*ones(obj.nT-1,1);
      A_dt_names = repmat({'tau_i+tau_i+1'},obj.nT-1,1);
      obj = obj.addLinearConstraint(LinearConstraint(-inf(obj.nT-1,1),obj.A_dt_bnd,obj.A_dt),(1:obj.num_vars),A_dt_names);
      iAsdotsquare = [(1:obj.nT)';(1:obj.nT)';obj.nT+(1:obj.nT)';obj.nT+(1:obj.nT)'];
      jAsdotsquare = [obj.sdotsquare_idx';obj.sdotsquareplus_idx';obj.sdotsquare_idx';obj.sdotsquareminus_idx'];
      Aval_sdotsquare = [ones(obj.nT,1);-ones(obj.nT,1);ones(obj.nT,1);-ones(obj.nT,1)];
      obj.A_sdotsquare = sparse(iAsdotsquare,jAsdotsquare,Aval_sdotsquare,2*obj.nT,obj.num_vars);
      obj.A_sdotsquare_bnd = [-ones(obj.nT,1);ones(obj.nT,1)];
      A_sdotsquare_names = [repmat({'sdotsquare_plus'},obj.nT,1);repmat({'sdotsquare_minus'},obj.nT,1)];
      obj = obj.addLinearConstraint(LinearConstraint(obj.A_sdotsquare_bnd,obj.A_sdotsquare_bnd,obj.A_sdotsquare),(1:obj.num_vars),A_sdotsquare_names);
      obj.cone_dt_idx = [obj.sdotsquareplus_idx' obj.tau_idx' obj.sdotsquareminus_idx'];
      
      obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(zeros(obj.nT,1),obj.sdot_max^2*ones(obj.nT,1)),obj.sdotsquare_idx(:));
      
      obj.Q_cost = sparse(obj.epsilon_idx(:),obj.epsilon_idx(:),ones(3*obj.nT,1),obj.num_vars,obj.num_vars);
      obj.f_cost = zeros(obj.num_vars,1);
      obj.f_cost(obj.margin_idx(:)) = -c_margin;
    end
    
    function [F,sdotsquare,Hdot,Hbar,margin,sigma,epsilon] = solve(obj,com,comp,compp,foot_pos,sigma)
      % @param com    - A 3 x obj.nT matrix. com(:,i) is the CoM position at i'th knot
      % @param comp   - A 3 x obj.nT matrix. comp(:,i) is the first derivative of CoM
      % w.r.t time scaling function s at i'th knot point
      % @param compp  - A 3 x obj.nT matrix. compp(:,i) is the second derivative of CoM
      % w.r.t time scaling function s at i'th knot point
      % @param foot_pos  - A 2 x obj.num_fsrc_cnstr matrix. foot_pos(:,i) is the xy
      % position of the contact body in obj.fsrc_cnstr{i}
      % @param sigma    A scalar. The upper bound for the angular PD residue. sum_n
      % epsilon[n]'*epsilon[n] <= sigma
      % @retval F     A cell array. F{i}{j} is the force parameters for
      % obj.fsrc_cnstr(obj.F2fsrc_map{i}(j))
      % @retval sdotsquare  A 1 x obj.nT vector. sdotsquare(i) is the square of the time
      % scaling function at time obj.t_knot(i)
      % @retval Hdot   A 3 x obj.nT matrix. Hdot[i] is the rate of the angular momentum at
      % i'th knot point
      % @retval Hbar   A 3 x obj.nT matrix. The integration of Hdot w.r.t the time scaling
      % function s.
      % @retval epsilon   A 3 x obj.nT matrix. The residue of the PD law on angular
      % momentum
      sizecheck(com,[3,obj.nT]);
      sizecheck(comp,[3,obj.nT]);
      sizecheck(compp,[3,obj.nT]);
      sizecheck(foot_pos,[2,obj.num_fsrc_cnstr]);
      sizecheck(sigma,[1,1]);
      % compute the CoMddot from the comp,compp and time scaling function
      iAcomddot = reshape(repmat(reshape(1:3*obj.nT,3,obj.nT),2,1),[],1);
      jAcomddot = reshape([bsxfun(@times,ones(3,1),[obj.sdotsquare_idx]);...
        bsxfun(@times,ones(3,1),[obj.sdotsquare_idx(2:end) obj.sdotsquare_idx(end)])],[],1);
      delta_s = 1/(obj.nT-1);
      Aval_comddot = reshape(obj.robot_mass*[compp-comp/(2*delta_s);comp/(2*delta_s)],[],1);
      A_comddot = obj.A_newton-sparse(iAcomddot,jAcomddot,Aval_comddot,3*obj.nT,obj.num_vars);
      A_comddot_bnd = repmat([0;0;obj.robot_mass*obj.g],obj.nT,1);
      A_angular = zeros(3*obj.nT,obj.num_vars);
      A_angular_bnd = zeros(3*obj.nT,1);
      foot_contact_pts_pos = cell(1,obj.num_fsrc_cnstr);
      for i = 1:obj.num_fsrc_cnstr
        foot_contact_pts_pos{i} = bsxfun(@times,obj.A_xy(:,:,i)*foot_pos(:,i)+obj.b_xy(:,:,i),...
          ones(1,obj.fsrc_cnstr{i}.num_contact_pts))+obj.rotmat(:,:,i)*obj.fsrc_cnstr{i}.body_contact_pts;
      end
      for i = 1:obj.nT
        for j = 1:length(obj.F_idx{i})
          fsrc_idx = obj.F2fsrc_map{i}(j);
          num_pts_ij = obj.fsrc_cnstr{fsrc_idx}.num_contact_pts;
          foot_contact_pos_CoM = foot_contact_pts_pos{fsrc_idx}-bsxfun(@times,com(:,i),ones(1,num_pts_ij));
%           iA_cross = reshape(bsxfun(@times,[1;1;2;2;3;3],ones(1,num_pts_ij)),[],1);
%           jA_cross = reshape(bsxfun(@plus,[2;3;1;3;1;2],3*(0:num_pts_ij-1)),[],1);
%           Aval_cross = reshape([-foot_contact_pos_CoM(3,:);foot_contact_pos_CoM(2,:);foot_contact_pos_CoM(3,:);-foot_contact_pos_CoM(1,:);-foot_contact_pos_CoM(2,:);foot_contact_pos_CoM(1,:)],[],1);
%           Across = sparse(iA_cross,jA_cross,Aval_cross,3,3*num_pts_ij);
          A_angular((i-1)*3+(1:3),obj.F_idx{i}{j}(:)) = cross(reshape(repmat(foot_contact_pos_CoM,obj.fsrc_cnstr{fsrc_idx}.num_edges,1),3,[]),...
            repmat(obj.A_force{fsrc_idx},1,obj.fsrc_cnstr{fsrc_idx}.num_contact_pts));
        end
        A_angular((i-1)*3+(1:3),obj.Hdot_idx(:,i)) = -eye(3)*obj.robot_mass*obj.g;
      end
      A_angular = A_angular/(obj.robot_mass*obj.g*obj.robot_dim);
      A_angular_bnd = A_angular_bnd/(obj.robot_mass*obj.g*obj.robot_dim);
      A_angular_names = repmat({'angular momentum'},3*obj.nT,1);
      obj = obj.addLinearConstraint(LinearConstraint(A_angular_bnd,A_angular_bnd,A_angular),(1:obj.num_vars),A_angular_names);
      A_comddot_bnd = A_comddot_bnd/(obj.robot_mass*obj.g);
      A_comddot = A_comddot/(obj.robot_mass*obj.g);
      A_comddot_names = repmat({'newton law'},3*obj.nT,1);
      obj = obj.addLinearConstraint(LinearConstraint(A_comddot_bnd,A_comddot_bnd,A_comddot),(1:obj.num_vars),A_comddot_names);
      
      model.A = sparse([obj.Ain;obj.Aeq]);
      model.rhs = [obj.bin;obj.beq];
%       max_row_entry = max(abs(model.A),[],2);
%       model.A = sparse(model.A./bsxfun(@times,max_row_entry,ones(1,obj.num_vars)));
%       model.rhs = model.rhs./max_row_entry;
      model.sense = [repmat('<',length(obj.bin),1);repmat('=',length(obj.beq),1)];
      model.Q = obj.Q_cost;
      model.obj = obj.f_cost;
      obj = obj.addBoundingBoxConstraint(BoundingBoxConstraint(sqrt(sigma),sqrt(sigma)),obj.sigma_idx);
      model.lb = obj.x_lb;
      model.ub = obj.x_ub;
      model.cones(1) = struct('index',[obj.sigma_idx obj.epsilon_idx(:)']);
      for i = 1:obj.nT
        model.cones(1+i) = struct('index',obj.cone_dt_idx(i,:));
      end
      
      params = struct('OutputFlag',false);
      
      result = gurobi(model,params);
      if(strcmp(result.status,'OPTIMAL')||strcmp(result.status,'SUBOPTIMAL'))
        F = cell(1,obj.nT);
        for i = 1:obj.nT
          for j = 1:length(obj.F_idx{i})
            fsrc_idx = obj.F2fsrc_map{i}(j);
            F{i} = [F{i} {reshape(result.x(obj.F_idx{i}{j}),obj.fsrc_cnstr{fsrc_idx}.num_edges,obj.fsrc_cnstr{fsrc_idx}.num_contact_pts)}];
          end
        end
        sdotsquare = reshape(result.x(obj.sdotsquare_idx),1,[]);
        Hdot = reshape(result.x(obj.Hdot_idx(:)),3,obj.nT)*obj.robot_mass*obj.g;
        Hbar = reshape(result.x(obj.H_idx(:)),3,obj.nT);
        margin = result.x(obj.margin_idx);
        epsilon = reshape(result.x(obj.epsilon_idx(:)),3,obj.nT);
        sigma = sum(epsilon(:).^2);
      else
        error('F-step is infeasible');
      end
    end
    
  end
end