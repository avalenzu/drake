classdef RigidBodySymplecticTrajectoryOptimization < DirectTrajectoryOptimization
  properties
    contact_inds = struct('forces', {}, 'torques', {}, 'points', {})
    r_inds % 3 x N array of indices for the COM position of the rigid body
    z_inds % 4 x N array of indices for the quaternion representing the orientation of the rigid body
    w_inds % 3 x N array of indices for the angular velocity of the rigid body (in body frame)
    v_inds % 3 x N array of indices for the translational velocity of the rigid body (in world frame)
    F_inds % 3 x N array of indices for the resultant force on the rigid body
           % (in world frame)
    T_inds % 3 x N array of indices for the resultant torque about the center of
           % mass of the rigid body (in body frame)
    f_inds_by_contact
    p_inds_by_contact
    I = []
    m = []
    g = [0; 0; -9.81]
  end

  methods
    function obj = RigidBodySymplecticTrajectoryOptimization(m, I, contact_wrench_struct, varargin)
      plant = DrakeSystem(0, 7+6, 6, 0, 0, 1); 
      obj = obj@DirectTrajectoryOptimization(plant, varargin{:});
      obj.m = m;
      obj.I = I;
      obj.F_inds = obj.u_inds(1:3, :);
      obj.T_inds = obj.u_inds(4:6, :);
      obj.r_inds = obj.x_inds(1:3, :);
      obj.z_inds = obj.x_inds(4:7, :);
      obj.w_inds = obj.x_inds(8:10, :);
      obj.v_inds = obj.x_inds(11:13, :);
      obj = obj.parseContacts(contact_wrench_struct);
      obj = obj.addDynamicConstraints();
      obj = obj.addForceConstraints();
    end

    % Implement required methods for DirectTrajectoryOptimization
    function obj = addDynamicConstraints(obj)
      import drakeFunction.*
      import drakeFunction.geometry.*
      import drakeFunction.integration.*
      if ~isempty(obj.m)
        dyn_fcn_cell = cell(obj.N-1, 1);
        xinds_cell = cell(obj.N-1, 1);
        for n = 1:obj.N-1
          exp_fcn = ExpMap2Quat();
          quatProduct_fcn = QuaternionProduct();
          rotateVector_fcn = RotateVectorByQuaternion();
          r0= Identity(3);
          r1 = Identity(3);
          z0= Identity(4);
          z1 = Identity(4);
          v0= Identity(3);
          v1 = Identity(3);
          w0= Identity(3);
          w1 = Identity(3);
          h = Identity(1);
          T0 = Identity(3);
          T1 = Identity(3);
          F0 = Identity(3);
          F1 = Identity(3);
          
          offset = 0;
          h_inds = offset + 1; offset = offset + 1;
          r0_inds = offset + (1:3); offset = offset + 3;
          z0_inds = offset + (1:4); offset = offset + 4;
          w0_inds = offset + (1:3); offset = offset + 3;
          v0_inds = offset + (1:3); offset = offset + 3;
          r1_inds = offset + (1:3); offset = offset + 3;
          z1_inds = offset + (1:4); offset = offset + 4;
          w1_inds = offset + (1:3); offset = offset + 3;
          v1_inds = offset + (1:3); offset = offset + 3;
          F0_inds = offset + (1:3); offset = offset + 3;
          T0_inds = offset + (1:3); offset = offset + 3;
          F1_inds = offset + (1:3); offset = offset + 3;
          T1_inds = offset + (1:3); offset = offset + 3;
          n_inputs = offset;
          
          w_mid = 0.5*(w0 + w1);                      % [w0; w1]
          exp_hw_mid = exp_fcn(h.*w_mid);             % [h; w0; w1]
          z1_des = quatProduct_fcn([z0; exp_hw_mid]); % [z0; h; w0; w1]
          z_fcn = z1 - z1_des;                        % [z1; z0; h; w0; w1]
          
          % Distribute inputs
          A = zeros(z_fcn.dim_input, n_inputs);
          A(9, h_inds) = 1;
          A(5:8, z0_inds) = eye(4);
          A(1:4, z1_inds) = eye(4);
          A(10:12, w0_inds) = eye(3);
          A(13:15, w1_inds) = eye(3);
          
          z_fcn = z_fcn(drakeFunction.Linear(A));
          
          A = exp_fcn(-0.5*h.*w_mid);         % [h; w0; w1]
          B = exp_fcn(0.5*h.*w_mid);          % [h; w0; w1]
          
          %  1:3 4  5:7 8:10  11 12:14  15:17 18:20   21 22:24  25 26 27:29 30:32   33:35
          w_fcn = obj.I*w1 - ...              % [w1; h; w0; w1;   h; w0;    w1;   w0;     h; T0;    h; h; w0;   w1;     T1]
            rotateVector_fcn([A; rotateVector_fcn([A; obj.I*w0 + 0.5*h.*T0]) + ...
            0.5*h.*rotateVector_fcn([B; T1])]);
          
          % Distribute inputs
          A = zeros(w_fcn.dim_input, n_inputs);
          A([4,11,21,25,26], h_inds) = 1;
          A(22:24, T0_inds) = eye(3);
          A(33:35, T1_inds) = eye(3);
          A([5:7, 12:14, 18:20, 27:29], w0_inds) = repmat(eye(3), [4, 1]);
          A([1:3, 8:10, 15:17, 30:32], w1_inds) = repmat(eye(3), [4, 1]);
          
          w_fcn = w_fcn(drakeFunction.Linear(A));
          
          xinds = [obj.h_inds(n); reshape(obj.x_inds(:, n:n+1), [], 1); ...
            reshape(obj.u_inds(:, n:n+1), [], 1)];
          lb_all = []; ub_all = [];
          lb = zeros(z_fcn.dim_output, 1); ub = lb;
          lb_all = [lb_all; lb]; ub_all = [ub_all; ub];
          %obj = obj.addConstraint(DrakeFunctionConstraint(lb, ub, z_fcn), xinds);
          lb = zeros(w_fcn.dim_output, 1); ub = lb;
          lb_all = [lb_all; lb]; ub_all = [ub_all; ub];
          %obj = obj.addConstraint(DrakeFunctionConstraint(lb, ub, w_fcn), xinds);
          dyn_fcn = Concatenated({z_fcn; w_fcn}, true);
          obj = obj.addConstraint(DrakeFunctionConstraint(lb_all, ub_all, dyn_fcn), xinds);
        end
        xinds = [obj.r_inds(:); obj.v_inds(:); obj.F_inds(:); obj.h_inds(:)];
        translational_dynamics = StormerVerletResiduals(3, obj.N);
        lb = zeros(translational_dynamics.dim_output, 1);
        ub = lb;
        translational_dynamics_constraint = DrakeFunctionConstraint(lb, ub, translational_dynamics);
        obj = obj.addConstraint(translational_dynamics_constraint, xinds);
      end
    end
    
    function obj = parseContacts(obj, contact_wrench_struct)
      obj.contact_inds(obj.N).forces = [];
      obj.f_inds_by_contact = cell(numel(contact_wrench_struct), 1);
      obj.p_inds_by_contact = cell(numel(contact_wrench_struct), 1);
      for i = 1:numel(contact_wrench_struct)
        assert(max(contact_wrench_struct(i).knots) <= obj.N);
        num_forces = contact_wrench_struct(i).num_forces;
        num_knots = numel(contact_wrench_struct(i).knots);
        num_constraints = numel(contact_wrench_struct(i).constraint);
        num_force_vars = 3*num_forces*num_knots;
        
        var_names = cellStrCat(sprintf('f%d', i), {'x', 'y', 'z'}, ...
                               '_', num2cellStr(contact_wrench_struct(i).knots));
        obj = obj.addDecisionVariable(num_force_vars, var_names);
        f_inds = obj.num_vars - num_force_vars + (1:num_force_vars);
        f_inds = reshape(f_inds, [3, num_forces*num_knots]);
        obj.f_inds_by_contact{i} = f_inds;
        
        var_names = cellStrCat(sprintf('p%d', i), {'x', 'y', 'z'}, ...
                               '_', num2cellStr(contact_wrench_struct(i).knots));
        obj = obj.addDecisionVariable(num_force_vars, var_names);
        p_inds = obj.num_vars - num_force_vars + (1:num_force_vars);
        p_inds = reshape(p_inds, [3, num_forces*num_knots]);
        obj.p_inds_by_contact{i} = p_inds;
        
        offset = 0;
        for n = contact_wrench_struct(i).knots
          indices = offset + (1:num_forces);
          obj.contact_inds(n).forces = [obj.contact_inds(n).forces, f_inds(:, indices)];
          obj.contact_inds(n).points = [obj.contact_inds(n).points, p_inds(:, indices)];
          for j = 1:num_constraints
            xinds = [];
            if ismember('f', contact_wrench_struct(i).vars{j})
              xinds = [xinds; reshape(f_inds(:, indices), [], 1)];
            end
            if ismember('p', contact_wrench_struct(i).vars{j})
              xinds = [xinds; reshape(p_inds(:, indices), [], 1)];
            end
            if ismember('r', contact_wrench_struct(i).vars{j})
              xinds = [xinds; reshape(obj.r_inds(:,n), [], 1)];
            end
            if ismember('z', contact_wrench_struct(i).vars{j})
              xinds = [xinds; reshape(obj.z_inds(:,n), [], 1)];
            end
            if ismember('w', contact_wrench_struct(i).vars{j})
              xinds = [xinds; reshape(obj.w_inds(:,n), [], 1)];
            end
            if ismember('v', contact_wrench_struct(i).vars{j})
              xinds = [xinds; reshape(obj.v_inds(:,n), [], 1)];
            end
            obj = obj.addConstraint(contact_wrench_struct(i).constraint{j}, xinds);
          end
          offset = offset + num_forces;
        end
      end
    end
    
    function obj = addForceConstraints(obj)
      import drakeFunction.*
      import drakeFunction.geometry.*
      r = Identity(3);
      z_conj = QuaternionConjugate();
      p = Identity(3);
      f = Identity(3);
      T_i = compose(RotateVectorByQuaternion(), [z_conj; cross(p-r, f)]);
      T_i = T_i.addInputs(3, false);
      xinds_cell = cell(obj.N,1);
      T_err_cell = cell(obj.N,1);
      for n = 1:obj.N
        A = [eye(3), -repmat(eye(3), [1, size(obj.contact_inds(n).forces, 2)])];
        b = obj.m*obj.g;
        xinds = [obj.F_inds(:,n); obj.contact_inds(n).forces(:)];
        obj = obj.addConstraint(LinearConstraint(b, b, A), xinds);
        
        xinds_cell{n} = [obj.T_inds(:,n); obj.z_inds(:,n); obj.contact_inds(n).points(:); ...
                 obj.r_inds(:,n); obj.contact_inds(n).forces(:)];
        T_err = Linear([eye(3), zeros(3, numel(xinds_cell{n}) - 3)]);
        num_contacts = size(obj.contact_inds(n).forces, 2);
        for i = 1:num_contacts
          selection_matrix = [zeros(3, 3*(i-1)), eye(3), zeros(3, 3*(num_contacts-i))];
          A = blkdiag(eye(3), eye(4), selection_matrix, eye(3), selection_matrix);
          T_err = minus(T_err, compose(T_i, Linear(A)), true);
        end
        T_err_cell{n} = T_err;
      end
      T_err = Concatenated(T_err_cell);
      obj = obj.addConstraint(DrakeFunctionConstraint(zeros(3*obj.N,1), ...
        zeros(3*obj.N,1), ...
        T_err), cell2mat(xinds_cell));
    end
    
    function obj = addRunningCost(obj, running_cost_function)
    end
  end
end
