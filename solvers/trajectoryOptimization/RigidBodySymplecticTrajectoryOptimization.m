classdef RigidBodySymplecticTrajectoryOptimization < DirectTrajectoryOptimization
  properties
    contact_inds = struct('forces', {}, 'torques', {}, 'points', {})
    F_inds % 3 x N array of indices for the resultant force on the rigid body
           % (in world frame)
    T_inds % 3 x N array of indices for the resultant torque about the center of
           % mass of the rigid body (in body frame)
    I = eye(3)
    m = 1
  end

  methods
    function obj = RigidBodySymplecticTrajectoryOptimization(varargin)
      plant = DrakeSystem(0, 7+6, 6, 0, 0, 1); 
      obj = obj@DirectTrajectoryOptimization(plant, varargin{:});
    end

    % Implement required methods for DirectTrajectoryOptimization
    function obj = addDynamicConstraints(obj)
      import drakeFunction.*
      import drakeFunction.geometry.*
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
        v0_inds = offset + (1:3); offset = offset + 3;
        w0_inds = offset + (1:3); offset = offset + 3;
        r1_inds = offset + (1:3); offset = offset + 3;
        z1_inds = offset + (1:4); offset = offset + 4;
        v1_inds = offset + (1:3); offset = offset + 3;
        w1_inds = offset + (1:3); offset = offset + 3;
        F0_inds = offset + (1:3); offset = offset + 3;
        T0_inds = offset + (1:3); offset = offset + 3;
        F1_inds = offset + (1:3); offset = offset + 3;
        T1_inds = offset + (1:3); offset = offset + 3;
        n_inputs = offset;

        v_mid = v0 + (0.5/obj.m)*h.*F0;     % [v0; h; F0]
        r1_des = r0 + h.*v_mid;             % [r0; h; v0; h; F0]
        r_fcn = r1 - r1_des;                % [r1; r0; h; v0; h; F0]

        % Distribute inputs
        A = zeros(r_fcn.dim_input, n_inputs);
        A([7; 11], h_inds) = 1;
        A(4:6, r0_inds) = eye(3);
        A(8:10, v0_inds) = eye(3);
        A(1:3, r1_inds) = eye(3);
        A(12:14, F0_inds) = eye(3);

        r_fcn = r_fcn(drakeFunction.Linear(A));

        v1_des = v_mid + (0.5/obj.m)*h.*F1; % [v0; h; F0; h; F1]
        v_fcn = v1 - v1_des;                % [v1; v0; h; F0; h; F1]
        % Distribute inputs
        A = zeros(v_fcn.dim_input, n_inputs);
        A([7; 11], h_inds) = 1;
        A(4:6, v0_inds) = eye(3);
        A(1:3, v1_inds) = eye(3);
        A(8:10, F0_inds) = eye(3);
        A(12:14, F1_inds) = eye(3);

        v_fcn = v_fcn(drakeFunction.Linear(A));

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
        w_fcn = obj.I*w1 - ...              % [w1; h; w0; w1; h; w0; w1; w0; h; T0; h; h; w0; w1; T1]
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
        lb = zeros(r_fcn.dim_output, 1); ub = lb;
        obj = obj.addConstraint(DrakeFunctionConstraint(lb, ub, r_fcn), xinds);
        lb = zeros(z_fcn.dim_output, 1); ub = lb;
        obj = obj.addConstraint(DrakeFunctionConstraint(lb, ub, z_fcn), xinds);
        lb = zeros(v_fcn.dim_output, 1); ub = lb;
        obj = obj.addConstraint(DrakeFunctionConstraint(lb, ub, v_fcn), xinds);
        lb = zeros(w_fcn.dim_output, 1); ub = lb;
        obj = obj.addConstraint(DrakeFunctionConstraint(lb, ub, w_fcn), xinds);
      end
    end
    
    function obj = addRunningCost(obj, running_cost_function)
    end
  end
end
