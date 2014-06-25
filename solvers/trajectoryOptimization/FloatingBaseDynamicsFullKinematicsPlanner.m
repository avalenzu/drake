classdef FloatingBaseDynamicsFullKinematicsPlanner < SimpleDynamicsFullKinematicsPlanner
  % This planner impose the following dynamics constraint
  properties(SetAccess = protected)
  end
  
  methods
    function obj = FloatingBaseDynamicsFullKinematicsPlanner(robot,N,tf_range,Qv,Q,q_nom,varargin)
      % @param Q_comddot  A 3 x 3 matrix. penalize sum_j comddot(:,j)*Q_comddot*comddot(:,j)
      % @param Q  an nq x nq matrix. Add the cost sum_j
      % (q(:,j)-q_nom(:,j))'*Q*(q(:,j)-q_nom(:,j));
      plant = SimpleDynamicsDummyPlant(robot.getNumPositions());
      obj = obj@SimpleDynamicsFullKinematicsPlanner(plant,robot,N,tf_range,varargin{:});
      assert(obj.nv == obj.nq);
      obj.add_dynamic_constraint_flag = true;
      obj = obj.addDynamicConstraints();
      sizecheck(Q,[obj.nq,obj.nq]);
      if(any(eig(Q)<0))
        error('Drake:ComDynamicsFullKinematicsPlanner:Q should be PSD');
      end
      sizecheck(q_nom,[obj.nq,obj.N]);
      posture_err_cost = QuadraticSumConstraint(-inf,inf,Q,q_nom);
      obj = obj.addCost(posture_err_cost,reshape(obj.q_inds,[],1));
      v_cost = QuadraticSumConstraint(-inf,inf,Qv,zeros(obj.nv,obj.N));
      obj = obj.addCost(v_cost,reshape(obj.v_inds,[],1));
    end
    
    function obj = addContactDynamicConstraints(obj,knot_idx,contact_wrench_cnstr_idx,knot_lambda_idx)
      num_knots = numel(knot_idx);
      sizecheck(num_knots,[1,1]);

      function [c,dc] = dtDynFun(h,x_l,x_r,lambda_r,kinsol_r)
        h_idx        = 1;
        q_l_idx      = 1+(1:obj.nq);
        qd_l_idx     = 1+obj.nq+(1:obj.nv);
        q_r_idx      = 1+obj.nq+obj.nv+(1:obj.nq);
        qd_r_idx     = 1+obj.nq+obj.nv+obj.nq+(1:obj.nv);
        lambda_r_idx = 1+obj.nq+obj.nv+obj.nq+obj.nv+(1:numel(lambda_r));
        qd_con_idx = 1:obj.nq;
        qdd_fb_con_idx = obj.nq+(1:6);
        A_force = cell(1,length(contact_wrench_cnstr_idx));
        q_r = x_r(1:obj.nq);
        qd_r = x_r(obj.nq+(1:obj.nq));
        [H,C,~,dH,dC] = manipulatorDynamics(obj.robot,q_r,qd_r,true);
        H_fb = H(1:6,:);
        C_fb = C(1:6,:);
        dC_fb = dC(1:6,:);
        x_diff = x_r - x_l;
        q_diff = x_diff(1:obj.nq);
        qd_diff = x_diff(obj.nq+(1:obj.nq));
        c = zeros(obj.nq+6,1);
        dc = zeros(obj.nq+6,1+2*(obj.nq+obj.nv)+numel(lambda_r));

        % Backwards Euler for q
        c(qd_con_idx) = q_diff - h*qd_r;
        %dc_dh
        dc(qd_con_idx,h_idx) = -qd_r;
        %dc_dq_l
        dc(qd_con_idx,q_l_idx) = eye(obj.nq);
        %dc_dq_r
        dc(qd_con_idx,q_r_idx) = -eye(obj.nq);
        %dc_dqd_r
        dc(qd_con_idx,qd_r_idx) = -h*eye(obj.nq);

        %% Backwards Euler for qd_fb
        %   
        %   H_fb*qd + C_fb - sum(J^T*F) = 0
        %

        % Mass and Coriolis/Gravitational terms
        c(qdd_fb_con_idx) = H_fb*qd_diff + h*C_fb;
        %dc_dh
        dc(qdd_fb_con_idx,h_idx) = C_fb;
        %dc_dqd_l
        dc(qdd_fb_con_idx,qd_l_idx) = -H_fb;
        %dc_dq_r
        dc(qdd_fb_con_idx,q_r_idx) = H_fb + eye(6,obj.nq)*matGradMult(dH(:,1:obj.nq),qd_diff) + h*dC_fb(:,1:obj.nq); 
        %dc_dqd_r
        dc(qdd_fb_con_idx,qd_r_idx) = H_fb + h*dC_fb(:,obj.nq+(1:obj.nv));

        lambda_r_count = 0;

        % External force terms
        for i = contact_wrench_cnstr_idx{2}
          num_pts_i = obj.contact_wrench_cnstr{i}.num_pts;
          num_lambda_r_i = obj.contact_wrench_cnstr{i}.pt_num_F*num_pts_i;
          lambda_r_i_idx = lambda_r_idx(lambda_r_count+(1:num_lambda_r_i));
          A_force{i} = obj.contact_wrench_cnstr{i}.force();
          force_i = reshape(A_force{i}*lambda_r(lambda_r_count+(1:num_lambda_r_i)),3,num_pts_i);
          [~,J_i,dJ_i] = obj.robot.forwardKin(kinsol_r,obj.contact_wrench_cnstr{i}.body,obj.contact_wrench_cnstr{i}.body_pts,0);
          lambda_r_count = lambda_r_count+num_lambda_r_i;
          JtransposeForce = J_i(:,1:6)'*reshape(force_i,[],1);
          c(qdd_fb_con_idx) = c(qdd_fb_con_idx) + h*JtransposeForce;
          % dc_dh
          dc(qdd_fb_con_idx,h_idx) = dc(qdd_fb_con_idx,h_idx) + JtransposeForce;
          dc(qdd_fb_con_idx,q_r_idx) = dc(qdd_fb_con_idx,q_r_idx) + h*eye(6,obj.nq)*reshape(matGradMult(dJ_i,reshape(force_i,[],1),true),obj.nq,obj.nq);
          dc(qdd_fb_con_idx,lambda_r_i_idx) = dc(qdd_fb_con_idx,lambda_r_i_idx) + h*J_i(:,1:6)';
        end
      end

      if(num_knots == 1)
        %do nothing
      elseif(num_knots == 2)
        cnstr = NonlinearConstraint(zeros(obj.nq+6,1),zeros(obj.nq+6,1),1+2*(obj.nq+obj.nv)+numel(knot_lambda_idx{2}),@dtDynFun);
        %[h_row,h_col] = find(h_sparse_pattern);
        %h_cnstr = h_cnstr.setSparseStructure(h_row,h_col);
        dyn_inds = {obj.h_inds(knot_idx(1)); ...
                    obj.x_inds(:,knot_idx(1)); ...
                    obj.x_inds(:,knot_idx(2)); ...
                    knot_lambda_idx{2}};
        obj = obj.addNonlinearConstraint(cnstr,dyn_inds,obj.kinsol_dataind(knot_idx(2)));
      end
    end

    function data = kinematicsData(obj,q)
      data = doKinematics(obj.robot,q,true,false);
    end
    
    function obj = addRunningCost(obj,running_cost)
    end
  end
end
