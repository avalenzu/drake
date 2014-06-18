classdef AngularMomentumConstraint2 < NonlinearConstraint
  properties
    robot
    H_nom
    nq
    nv
    nT
  end
  methods
    function obj = AngularMomentumConstraint2(r,t,H_nom_traj)
      % obj = AngularMomentumConstraint(rbm,lb,ub)
      % @param r      -- RigidBodyManipulator object
      % @param t      -- Vector of knot-point times
      % @param H_nom  -- Nominal angular momentum. May be a 3 x 1 double or a 
      %                  3 x 1 Trajectory (optional)
      %                  @default [0;0;0]
      % @param ub     -- The upper bound of the constraint (optional)
      %                  @default 0
      nT = numel(t);
      %if nargin < 4
        %ub = 1e-2*ones(3*(nT-1),1);
      %end
      ub = zeros(3*(nT-1),1);
      lb = -Inf(3*(nT-1),1);
      nq = r.getNumDOF();
      nv = r.getNumDOF();

      obj = obj@NonlinearConstraint(lb,ub,nq*nT+1);

      idx = logical(kron(eye(nT-1,nT)+circshift(eye(nT-1,nT),[0,1]),ones(3,nq)));
      %idx = idx | logical(kron(diag(ones(nT-2,1),1),ones(3,nq)));
      [I,J] = ndgrid(1:3*(nT-1),[1:nq*nT+1]);
      sizecheck(I,double(idx));
      iCfun = [I(idx);(1:3*(nT-1))'];
      jCvar = [J(idx);repmat(nq*nT+1,3*(nT-1),1)];
      obj = obj.setSparseStructure(iCfun,jCvar);

      obj.robot = r;
      if nargin < 3
        obj.H_nom = [0;0;0];
      else
        obj.H_nom = H_nom_traj.eval(t);
      end
      obj.nq = nq;
      obj.nv = nv;
      obj.nT = nT;
    end

    function [c,dc] = eval(obj,x)
      % [c,dc] = eval(obj,x) evaluates the constraint
      %
      %  c = (H(q,v) - H_nom)'*(H(q,v) - H_nom)
      %
      %  Since H(q,v) = A(q)*v, where A is the first three rows of the
      %  CMM, we can rewrite this as
      %
      %   c = (A(q)*v - H_nom)'*(A(q)*v - H_nom)
      %   dcdq = 2*(A(q)*v - H_nom)'*(dAdq(q)*v) (weird multiplication)
      %   dcdv = 2*(A(q)*v - H_nom)'*A(q);
      % @params x -- Vector of decision variable values. 
      %             
      %                 x = [q;v]
      %
      %               where q is an nq*nT-element vector of positions and v is
      %               and v is an nv*nT-element vector of velocities.
      %q = reshape(x(1:obj.nq*obj.nT),obj.nq,obj.nT);
      %v = reshape(x(obj.nq*obj.nT + (1:obj.nv*obj.nT)));
      compute_first_derivative = nargout > 1;
      c = 0*x(1);
      if compute_first_derivative
        dc = zeros(obj.num_cnstr,obj.xdim)*x(1);
      end
      for i = 1:obj.nT-1
        rows_i = (i-1)*3 + (1:3);
        q_minus_idx = (i-1)*obj.nq + (1:obj.nq);
        q_plus_idx = (i)*obj.nq + (1:obj.nq);
        v_idx = obj.nT*obj.nq + (i-1)*obj.nv + (1:obj.nv);
        q_minus = x(q_minus_idx);
        q_plus = x(q_plus_idx);
        q = (q_minus+q_plus)/2;
        v = (q_plus-q_minus)/2;
        if compute_first_derivative
          [A,dAdq] = obj.robot.getCMMdA(q);
          dq_dq_minus = 0.5*eye(obj.nq);
          dq_dq_plus = 0.5*eye(obj.nq);
          dv_dq_minus = -0.5*eye(obj.nq);
          dv_dq_plus = 0.5*eye(obj.nq);
        else
          A = obj.robot.getCMMdA(q);
        end
        A = A(1:3,:);
        Herr = (A*v-obj.H_nom(:,i));
        c(rows_i) = Herr;
        if compute_first_derivative
          dci_dq = eye(3,6)*matGradMult(dAdq,v);
          dc(rows_i,q_minus_idx) = dc(rows_i,q_minus_idx) + dci_dq*dq_dq_minus;
          dc(rows_i,q_plus_idx) = dc(rows_i,q_plus_idx) + dci_dq*dq_dq_plus;
          dc(rows_i,q_minus_idx) = dc(rows_i,q_minus_idx) + A*dv_dq_minus;
          dc(rows_i,q_plus_idx) = dc(rows_i,q_plus_idx) + A*dv_dq_plus;
        end
      end
      c = c/obj.nT;% - x(end);
      if compute_first_derivative
        dc = dc/obj.nT;
        %dc(:,end) = -1;
      end
    end
  end
end
