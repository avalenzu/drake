classdef FullDynamicsFullKinematicsPlanner < SimpleDynamicsFullKinematicsPlanner
  % This planner impose the following dynamics constraint
  % kc_com(q) = com at evey t_knot
  % H(:,i) -H(:,i-1) = Hdot(:,i)*dt(i)
  % Hdot(:,i) = sum_j cross(p_contact_j-com(:,i),F_j)
  % com(:,i) - com(:,i-1) = comdot(:,i)*dt(i)
  % comdot(:,i)-comdot(:,i-1) = comddot(:,i)*dt(i)
  % m*comddot(:,i) = sum_j F_j-m*g
  % q(:,i)-q(:,i-1) = v(:,i)*dt(i)
  % A*v(:,i) = H(:,i) where A = robot.getCMM
  properties(SetAccess = protected)
    com_inds % A 3 x obj.N matrix. x(com_inds(:,i)) is the com position at i'th knot point
    comdot_inds % A 3 x obj.N matrix. x(comdot_inds(:,i)) is the com velocity at i'th knot point
    comddot_inds % A 3 x obj.N matrix. x(comddot_inds(:,i)) is the com acceleation at i'th knot point
    H_inds % A 3 x obj.N matrix. x(H_inds(:,i)) is the centroidal angular momentum at i'th knot point
    Hdot_inds % A 3 x obj.N matrix. x(Hdot_inds(:,i)) is the rate of centroidal angular momentum at i'th knot point
    torque_multiplier % A positive scaler. 
    ncp_tol; % The tolerance for nonlinear complementarity constraint
  end
  
  methods
    function obj = FullDynamicsFullKinematicsPlanner(robot,N,tf_range,Qv,Q,q_nom,varargin)
      % @param Q_comddot  A 3 x 3 matrix. penalize sum_j comddot(:,j)*Q_comddot*comddot(:,j)
      % @param Q  an nq x nq matrix. Add the cost sum_j
      % (q(:,j)-q_nom(:,j))'*Q*(q(:,j)-q_nom(:,j));
      plant = robot; 
      obj = obj@SimpleDynamicsFullKinematicsPlanner(plant,robot,N,tf_range,varargin{:});
      
      obj.add_dynamic_constraint_flag = true;
      obj = obj.addDynamicConstraints();
      sizecheck(Q,[obj.nq,obj.nq]);
      if(any(eig(Q)<0))
        error('Drake:ComDynamicsFullKinematicsPlanner:Q should be PSD');
      end
      sizecheck(q_nom,[obj.nq,obj.N]);
      %posture_err_cost = QuadraticSumConstraint(-inf,inf,Q,q_nom);
      %obj = obj.addCost(posture_err_cost,reshape(obj.q_inds,[],1));
      q_cost = FunctionHandleConstraint(-inf,inf,2+obj.nq,@(h,q) qCost(0.5*Q,q_nom(:,1),h,q));
      obj = obj.addCost(q_cost,{obj.h_inds([1,1]);obj.q_inds(:,1)});
      for i = 2:obj.N-1
        q_cost = FunctionHandleConstraint(-inf,inf,2+obj.nq,@(h,q) qCost(Q,q_nom(:,i),h,q));
        obj = obj.addCost(q_cost,{obj.h_inds([i-1,i]);obj.q_inds(:,i)});
      end
      q_cost = FunctionHandleConstraint(-inf,inf,2+obj.nq,@(h,q) qCost(0.5*Q,q_nom(:,end),h,q));
      obj = obj.addCost(q_cost,{obj.h_inds([end,end]);obj.q_inds(:,end)});

      sizecheck(Qv,[obj.nv,obj.nv]);
      if(any(eig(Qv)<0))
        error('Drake:ComDynamicsFullKinematicsPlanner:Q_v should be PSD');
      end
      %v_cost = QuadraticSumConstraint(-inf,inf,Qv,zeros(obj.nv,obj.N));
      %obj = obj.addCost(v_cost,reshape(obj.v_inds,[],1));
      v_cost = FunctionHandleConstraint(-inf,inf,1+obj.nv,@(h,v) vCost(Qv,h,v));
      for i = 1:obj.N-1
        obj = obj.addCost(v_cost,{obj.h_inds(i);obj.v_inds(:,i+1)});
      end
      
      %e = ones(obj.nq*(obj.N-2),1);
      %second_diff_mat = spdiags([e,-2*e,e],[-obj.nq,0,obj.nq],obj.nq*(obj.N-2),obj.nq*obj.N);
      %Qa = second_diff_mat'*kron(eye(obj.N-2),Qv)*second_diff_mat;
      %obj = obj.addCost(QuadraticConstraint(0,0,Qa,zeros(obj.nq*obj.N,1)),obj.q_inds);;

      function [f,df] = vCost(Q,h,v)
        %f = h^2*(v'*Q*v);
        %df = [2*h*(v'*Q*v), 2*h^2*v'*Q];
        f = h*(v'*Q*v);
        df = [v'*Q*v, 2*h*v'*Q];
      end

      function [f,df] = qCost(Q,q_nom,h,q)
        f = 0.5*(h(1)+h(2))*((q-q_nom)'*Q*(q-q_nom));
        df = [0.5*(q-q_nom)'*Q*(q-q_nom)*[1,1], (h(1)+h(2))*(q-q_nom)'*Q];
      end
    end

    function obj = addConstraint(obj, constraint, varargin)
      if isa(constraint, 'RigidBodyConstraint')
        obj = addRigidBodyConstraint(obj,constraint, varargin{:});
      else
        obj = addConstraint@SimpleDynamicsFullKinematicsPlanner(obj,constraint,varargin{:});
      end
    end
    
    function obj = addContactDynamicConstraints(obj,knot_idx,contact_wrench_idx,knot_lambda_idx)
      num_knots = numel(knot_idx);
      sizecheck(num_knots,[1,1]);
      num_lambda = cellfun(@numel,knot_lambda_idx);
      nX = obj.plant.getNumStates();
      nQ = obj.plant.getNumPositions();
      nU = obj.plant.getNumInputs();

      function [f,df] = midpoint_constraint_fun(h,x0,x1,u0,u1,lambda0,lambda1)
        nL0 = numel(lambda0);
        nL1 = numel(lambda1);
        x = 0.5*(x0+x1);
        dx = [zeros(nX,1),0.5*eye(nX), 0.5*eye(nX), zeros(nX,2*nU+nL0+nL1)];
        u = 0.5*(u0+u1);
        du = [zeros(nU,1),zeros(nU,2*nX),0.5*eye(nU), 0.5*eye(nU), zeros(nU,nL0+nL1)];
        q = x(1:nQ);
        dq = dx(1:nQ,:);
        qd = x(nQ+1:end);
        qdd = (x1(nQ+1:end)-x0(nQ+1:end))/h;
        dqdd = [-qdd/h,zeros(nQ),eye(nQ),zeros(nQ),eye(nQ), zeros(nQ,2*nU+nL0+nL1)];
        [H,C,B,dH,dC_dx,dB_dx] = obj.plant.manipulatorDynamics(q,qd);
        dC = dC_dx*dx;

        kinsol = obj.robot.doKinematics(q,true);
        JtransposeForce = cell(2,1);
        dJtransposeForce = cell(2,1);
        dJtransposeForce_dq = cell(2,1);
        dJtransposeForce_dL = cell(2,1);
        dJtransposeForce(:) = {zeros(nQ,1 + 2*nX + 2*nU + nL0 + nL1)};
        lambda_count{1} = 0;
        lambda_count{2} = 0;
        lambda = {lambda0;lambda1};
        dlambda{1} = [zeros(nL0,1+2*nX+2*nU), eye(nL0), zeros(nL0,nL1)];
        dlambda{2} = [zeros(nL1,1+2*nX+2*nU+nL0), eye(nL1)];
        for j = 1:2
          JtransposeForce{j} = zeros(nQ,1);
          dJtransposeForce_dq{j} = zeros(nQ,nQ);
          for i = contact_wrench_idx{j}
            num_pts_i = obj.contact_wrench{i}.num_pts;
            num_lambda_i = obj.contact_wrench{i}.num_pt_F*num_pts_i;
            lambda_idx_i = lambda_count{j}+(1:num_lambda_i);
            A_force{i} = obj.contact_wrench{i}.force();
            force_i = reshape(A_force{i}*lambda{j}(lambda_idx_i),3,num_pts_i);
            lambda_count{j} = lambda_count{j}+num_lambda_i;
            [~,J_i,dJ_i] = obj.robot.forwardKin(kinsol,obj.contact_wrench{i}.body,obj.contact_wrench{i}.body_pts,0);
            JtransposeForce{j} = JtransposeForce{j} + J_i'*reshape(force_i,[],1);
            dJtransposeForce_dq{j} = dJtransposeForce_dq{j} + reshape(matGradMult(dJ_i,reshape(force_i,[],1),true),obj.nq,obj.nq); 
            dJtransposeForce_dL{j} = J_i';
            dJtransposeForce{j} = dJtransposeForce_dq{j}*dq + dJtransposeForce_dL{j}*dlambda{j}(lambda_idx_i,:);
          end
        end
        JtransposeForce_avg = 0.5*(JtransposeForce{1}+JtransposeForce{2});
        dJtransposeForce_avg = 0.5*(dJtransposeForce{1}+dJtransposeForce{2});

        dH_reshaped = reshape(dH,[nQ,nQ,nX]);
        dH_flat = reshape(permute(dH_reshaped,[2,1,3]),[nQ,nQ*nX])';
        dHqdd_dx = reshape(dH_flat*qdd,[nQ,nX]);
        dHqdd = dHqdd_dx*dx + H*dqdd;

        dB_reshaped = reshape(dB_dx,[nQ,nU,nX]);
        dB_flat = reshape(permute(dB_reshaped,[2,1,3]),[nU,nQ*nX])';
        dBu_dx = reshape(dB_flat*u,[nQ,nX]);
        dBu = dBu_dx*dx + B*du;

        f = H*qdd + C - B*u - JtransposeForce_avg;
        df = dHqdd + dC - dBu - dJtransposeForce_avg; 
      end

      if(num_knots == 1)
        % do nothing
      elseif(num_knots == 2)
        n_vars =1 +  2*nX + 2*nU + sum(num_lambda);
        cnstr = FunctionHandleConstraint(zeros(nQ,1),zeros(nQ,1),n_vars,@midpoint_constraint_fun);
        dyn_inds = {obj.h_inds(knot_idx(1));obj.x_inds(:,knot_idx(1));obj.x_inds(:,knot_idx(2));obj.u_inds(:,knot_idx(1));obj.u_inds(:,knot_idx(2));knot_lambda_idx{1};knot_lambda_idx{2}};
        obj = obj.addConstraint(cnstr,dyn_inds);
      end
    end

    function obj = addRunningCost(obj,running_cost_function)
      % Adds an integrated cost to all time steps, which is
      % numerical implementation specific (thus abstract)
      % this cost is assumed to be time-invariant
      % @param running_cost_function a function handle
      %  of the form running_cost_function(dt,x,u)
      
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      
      for i=1:obj.N-1,
        running_cost = FunctionHandleObjective(1+2*nX+2*nU,...
          @(h,x0,x1,u0,u1) obj.midpoint_running_fun(running_cost_function,h,x0,x1,u0,u1));
        inds_i = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.u_inds(:,i+1)};

        obj = obj.addCost(running_cost,inds_i);
      end
    end

    function [f,df] = midpoint_running_fun(obj,running_handle,h,x0,x1,u0,u1)
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      [f,dg] = running_handle(h,.5*(x0+x1),.5*(u0+u1));
      
      df = [dg(:,1) .5*dg(:,2:1+nX) .5*dg(:,2:1+nX) .5*dg(:,2+nX:1+nX+nU) .5*dg(:,2+nX:1+nX+nU)];
    end

    function [q,v,h,t,lambda,wrench] = parseSolution(obj,x_sol)
      nq = obj.robot.getNumPositions;
      nT = obj.N;
      q = reshape(x_sol(obj.q_inds(:)),nq,nT);
      v = reshape(x_sol(obj.v_inds(:)),nq,nT);
      h = reshape(x_sol(obj.h_inds),1,[]);
      t = cumsum([0 h]);
      lambda = cell(length(obj.unique_contact_bodies),1);
      for i = 1:length(obj.unique_contact_bodies)
        lambda{i} = reshape(x_sol(obj.lambda_inds{i}),size(obj.lambda_inds{i},1),[],nT);
      end
      wrench = obj.contactWrench(x_sol);
    end
  end
end
