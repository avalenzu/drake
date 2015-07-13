classdef StateAlternatingDynamicsProgram < MixedIntegerConvexProgram
  properties
    I
    m
    N = 2
    position_max = 1e2
    velocity_max = 1e2
    force_max = 1e2
    dt = 1e-3
    r_fixed_array
    z_fixed_array
    v_fixed_array
    w_fixed_array
  end

  methods
    function obj = StateAlternatingDynamicsProgram(I, m, N, dt)
      obj = obj@MixedIntegerConvexProgram(true);
      obj.N = N;
      obj.dt = dt;
      obj.I = I;
      obj.m = m;
      obj = obj.addPositionVariables();
      obj = obj.addVelocityVariables();
      obj = obj.addDynamicConstraints();
    end


    function obj = addDynamicConstraints(obj)
      h = obj.dt;
      I = obj.I; %#ok
      m = obj.m; %#ok
      g = [0; 0; -9.81];
      for n = 1:(obj.N-1)
        r = obj.vars.(obj.positionName(n)).symb;
        z = obj.vars.(obj.orientationName(n)).symb;
        w = obj.vars.(obj.angularVelocityName(n)).symb;

        r_next = obj.vars.(obj.positionName(n+1)).symb;
        z_next = obj.vars.(obj.orientationName(n+1)).symb;
        w_next = obj.vars.(obj.angularVelocityName(n+1)).symb;
        Y_next = I*w_next; %#ok

        w_fixed = obj.w_fixed_array(:, n);
        w_next_fixed = obj.w_fixed_array(:, n+1);
        w_mid_fixed = 0.5*(w_fixed + w_next_fixed);
        A = expmap2quat(-h/2*w_mid_fixed);
        B = expmap2quat(h/2*w_mid_fixed);
        T = 0;
        T_next = 0;
        F = m*g; %#ok
        F_next = m*g; %#ok
        Y_next_desired = quatRotateVector(A, ...
                            quatRotateVector(A, I*w + h/2*T) ...
                            + h/2*quatRotateVector(B, T_next)); %#ok
        z_next_desired = quatProduct(z, expmap2quat(h*w_mid_fixed));
        v_mid = v + h/(2*m)*F; %#ok
        r_next_desired = r + h*v_mid;
        v_next_desired = v_mid + h/(2*m)*F_next; %#ok
        obj = obj.addSymbolicConstraints(r_next == r_next_desired);
        obj = obj.addSymbolicConstraints(v_next == v_next_desired);
        obj = obj.addSymbolicConstraints(z_next == z_next_desired);
        obj = obj.addSymbolicConstraints(Y_next == Y_next_desired);
      end
    end

    function obj = addPositionVariables(obj)
      for n = 1:obj.N
        obj = obj.addVariable(obj.positionName(n), 'C', ...
          [3,1], -obj.position_max, ...
          obj.position_max);
        obj = obj.addVariable(obj.orientationName(n), 'C', [4,1], 0, 1);
      end
    end

    function obj = addVelocityVariables(obj)
        for n = 1:obj.N
          obj = obj.addVariable(obj.velocityName(n), 'C', ...
                                [3,1], -obj.velocity_max, obj.velocity_max);
          obj = obj.addVariable(obj.angularVelocityName(n), 'C', ...
                                [3,1], -obj.velocity_max, obj.velocity_max);
        end
    end
  end

  methods (Static)
    function name = positionName(time_index)
      name = sprintf('r%d', time_index);
    end

    function name = orientationName(time_index)
      name = sprintf('z%d', time_index);
    end

    function name = velocityName(time_index)
      name = sprintf('v%d', time_index);
    end

    function name = angularVelocityName(time_index)
      name = sprintf('w%d', time_index);
    end

  end
end
