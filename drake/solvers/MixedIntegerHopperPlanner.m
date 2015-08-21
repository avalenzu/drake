classdef MixedIntegerHopperPlanner < MixedIntegerConvexProgram
  properties
    regions = struct('A', {}, 'b', {}, 'Aeq', {}, 'beq', {}, 'normal', {}, 'mu', {}, 'ncons', {}, 'basis_vectors', {});
    position_max = 1e1
    rotation_max = 2*pi
    velocity_max = 1e1
    angular_velocity_max = 1e1
    force_max = 1e1
    moment_max = 1e2
    n_basis_vectors = 2
    n_orientation_sectors = 8;
    n_regions
    b_constant
    th_constant
    m_sth
    m_cth
    b_sth
    b_cth
    N
    M = 10% number of segments for approximating hyperbolic paraboloids
    dim = 2 % planar case
    dt
    I % dimensionless moment of inertia
    hip_in_body = [-0.5; -0.25];
    c_approx_splits = {} % obj.dim x obj.N x obj.basis_vectors cell array of obj.M-1
                    % element vectors indicating how to divide the approximating
                    % tetrahedra
    c_approx_A = {}
    c_approx_b = {}
    num_legs = 2

    minimize_force = true
    minimize_integral_of_squared_power = false
    use_slack = false;
    use_foot_acceleration = false;
  end
  
  methods
    function obj = MixedIntegerHopperPlanner(I, N, dt)
      obj = obj@MixedIntegerConvexProgram(false);
      obj.I = I;
      obj.N = N;
      obj.dt = dt;
    end
    
    function obj = setupProblem(obj)
      obj.num_legs = size(obj.hip_in_body, 2);
      obj = obj.addVariable('r', 'C', ...
        [obj.dim, obj.N], -obj.position_max, obj.position_max);
      obj = obj.addVariable('th', 'C', ...
        [1, obj.N], -obj.rotation_max, obj.rotation_max);   
      obj = obj.addVariable('v', 'C', ...
        [obj.dim, obj.N], -obj.velocity_max, obj.velocity_max);
      obj = obj.addVariable('w', 'C', ...
        [1, obj.N], -obj.angular_velocity_max, obj.angular_velocity_max);
      obj = obj.addVariable('cth', 'C', ...
        [1 obj.N], -1, 1);
      obj = obj.addVariable('sth', 'C', ...
        [1, obj.N], -1, 1);
      obj = obj.addVariable('r_hip', 'C', ...
        [obj.dim, obj.N, obj.num_legs], -1, 1);
      obj = obj.addVariable('r_foot', 'C', ...
        [obj.dim, obj.N, obj.num_legs], -obj.position_max, obj.position_max);
      obj = obj.addVariable('S', 'B', [obj.n_orientation_sectors, obj.N], 0, 1);
      obj = obj.addVariable('p', 'C', [obj.dim, obj.N, obj.num_legs], repmat(sqrt(2)/2*[-1; -1], [1, obj.N, obj.num_legs]), repmat(sqrt(2)/2*[1; -0.5], [1, obj.N, obj.num_legs]));
      obj = obj.addVariable('pd', 'C', [obj.dim, obj.N, obj.num_legs], -obj.velocity_max, obj.velocity_max);
      if obj.use_foot_acceleration
        obj = obj.addVariable('pdd', 'C', [obj.dim, obj.N, obj.num_legs], -obj.velocity_max, obj.velocity_max);
      end
      obj = obj.addVariable('F', 'C', ...
        [obj.dim, obj.N, obj.num_legs], -obj.force_max, obj.force_max);
      obj = obj.addVariable('T', 'C', ...
        [1, obj.N, obj.num_legs], -obj.moment_max, obj.moment_max);
      if obj.use_slack
        obj = obj.addVariable('slack', 'C', ...
          [1, obj.N, obj.num_legs], 0, 10*obj.moment_max);
      end
      obj = obj.addVariable('b', 'C', ...
        [obj.n_basis_vectors, obj.N, obj.num_legs], 0, obj.force_max);
      obj = obj.addVariable('c', 'C', ...
        [obj.dim, obj.N, obj.n_basis_vectors, obj.num_legs], -obj.moment_max, obj.moment_max);

      obj = obj.addVariable('B', 'B', [obj.dim, obj.N, obj.n_basis_vectors, obj.num_legs, obj.M], 0, 1);
      obj = obj.addVariable('R', 'B', [obj.n_regions, obj.N, obj.num_legs], 0, 1);
      if obj.minimize_integral_of_squared_power
        obj = obj.addVariable('W', 'C', [1, obj.N-1], -obj.dt*(obj.force_max*obj.velocity_max + obj.moment_max*obj.angular_velocity_max), obj.dt*(obj.force_max*obj.velocity_max + obj.moment_max*obj.angular_velocity_max));
        obj = obj.addVariable('PF', 'C', [obj.dim, obj.N-1], -obj.force_max*obj.velocity_max, obj.force_max*obj.velocity_max);
        obj = obj.addVariable('PT', 'C', [1, obj.N-1], -obj.moment_max*obj.angular_velocity_max, obj.moment_max*obj.angular_velocity_max);
        obj = obj.addVariable('BPF', 'B', [obj.dim, obj.N, obj.M], 0, 1);
        obj = obj.addVariable('BPT', 'B', [1, obj.N, obj.M], 0, 1);
      end
      
      for i = 1:obj.n_basis_vectors
        for n = 1:obj.N
          for j = 1:obj.num_legs
            for k = 1:obj.dim
              %obj = obj.addHyparApproximation({'p'}, {{k,n,j}}, ...
              obj = obj.addHyparApproximation({'r_hip', 'p'}, {{k,n,j}, {k,n,j}}, ...
                                               {'b'}, {{i,n,j}}, ...
                                               'c', {k,n,i,j}, ...
                                               obj.M, [], squeeze(obj.vars.B.i(k,n,i,j,:)));
                                               %obj.M, [], squeeze(obj.vars.B.i(k,ceil(n/2),i,j,:)));
            end
          end
        end
      end

      %makePoint = @(w, r) [w; r; w.*r];
      %ind_to_split = [];
      %dim_to_split = [];
      %for i = 0:ceil(log2(obj.M))
        %ind_to_split = [ind_to_split, 2^i:-1:1]; %#ok
        %dim_to_split = [dim_to_split, repmat(2, 1, 2^i)]; %#ok
      %end
      %for n = 1:obj.N
        %for j = 1:obj.num_legs
          %for k = 1:obj.dim
            % sum_m(V_hip(k,n,j,:) = 1
            %if k == 1, not_k = 2; else not_k = 1; end
            %ncons = 1;
            %indices = offset_eq + (1:ncons);
            %Aeq(indices, obj.vars.V_hip.i(k,n,j,:)) = ones(1, obj.M);
            %beq(indices) = 1;
            %offset_eq = offset_eq + ncons;

            %vertices = [makePoint(obj.vars.r_hip.lb(k,n,j), obj.vars.w.lb(n)), ...
                        %makePoint(obj.vars.r_hip.ub(k,n,j), obj.vars.w.lb(n)), ...
                        %makePoint(obj.vars.r_hip.ub(k,n,j), obj.vars.w.ub(n)), ...
                        %makePoint(obj.vars.r_hip.lb(k,n,j), obj.vars.w.ub(n))];
            %if isempty(ind_to_split)
              %vertices_cell = {vertices};
            %else
              %vertices_cell = obj.splitTetrahedron(vertices, default_dim_to_split, ind_to_split);
            %end
            %for m = 1:numel(vertices_cell)
              % B(k,n,i,j,m) --> [p(k,n); b(i,n); c(k,n,i)] lies in chull(vertices_cell{m})
              %[A_local, b_local] = vert2lcon(vertices_cell{m}');
              %A_local = [A_local(:,1), A_local]; %#ok
              %point_indices = [obj.vars.r_hip.i(k,n,j), obj.vars.p.i(k,n,j), obj.vars.b.i(i,n,j), obj.vars.c.i(k,n,i,j)];
              %obj.c_approx_A{k,n,i,j,m} = A_local;
              %obj.c_approx_b{k,n,i,j,m} = b_local;
              %big_M = obj.vars.p.ub(k,n,j) + obj.vars.b.ub(i,n,j) + obj.vars.c.ub(k,n,i,j);
              %ncons = 4;
              %indices = offset + (1:ncons);
              %A(indices, point_indices) = A_local;
              %A(indices, obj.vars.B.i(k,n,i,j,m)) = big_M*ones(ncons,1);
              %if obj.use_slack
                %A(indices, obj.vars.slack.i(1,n,1)) = -1;
              %end
              %b(indices) = b_local + big_M*ones(ncons,1);
              %offset = offset + ncons;
            %end
          %end
        %end
      %end

      obj = obj.setupOrientationSectors();
      
      obj = obj.addDynamicsConstraints();
      obj = obj.addContactPointConstraints(); 
      obj = obj.addOrientationConstraints();
      %obj = obj.addBinaryVariableConstraints(); 
      
      Q = zeros(obj.nv);
      c = zeros(obj.nv, 1);
      alpha = 0;
      if obj.minimize_force
        Q(obj.vars.b.i(:), obj.vars.b.i(:)) = eye(numel(obj.vars.b.i));
      end
      Q(obj.vars.c.i(:), obj.vars.c.i(:)) = eye(numel(obj.vars.c.i));
      %Q(obj.vars.T.i(:), obj.vars.T.i(:)) = 10*eye(numel(obj.vars.T.i));
      if obj.use_foot_acceleration
        Q(obj.vars.pdd.i(:), obj.vars.pdd.i(:)) = eye(numel(obj.vars.pdd.i));
      else
        Q(obj.vars.pd.i(:), obj.vars.pd.i(:)) = eye(numel(obj.vars.pd.i));
      end
      %Q(obj.vars.W.i(:), obj.vars.W.i(:)) = eye(numel(obj.vars.W.i));
      if obj.minimize_integral_of_squared_power
        Q(obj.vars.PF.i(:), obj.vars.PF.i(:)) = eye(numel(obj.vars.PF.i));
        Q(obj.vars.PT.i(:), obj.vars.PT.i(:)) = eye(numel(obj.vars.PT.i));
      end
      if obj.use_slack
        Q(obj.vars.slack.i(:), obj.vars.slack.i(:)) = 1e0*eye(numel(obj.vars.slack.i));
      end
      %A = spdiags([ones(obj.N-1, 1), ones(obj.N-1, 1)], [0,1], sparse(obj.N-1, obj.N));
      %ATA = A'*A;
      %for i = 1:obj.n_regions
        %Q(obj.vars.R.i(i,:), obj.vars.R.i(i,:)) = eye(obj.N);
      %end
      obj = obj.addCost(Q, c, alpha);
    end

    function obj = setupOrientationSectors(obj)
      obj.th_constant = obj.rotation_max*linspace(-1, 1, obj.n_orientation_sectors+1);
      sth = sin(obj.th_constant);
      cth = cos(obj.th_constant);
      obj.m_sth = zeros(obj.n_orientation_sectors,1);
      obj.m_cth = zeros(obj.n_orientation_sectors,1);
      obj.b_sth = zeros(obj.n_orientation_sectors,1);
      obj.b_cth = zeros(obj.n_orientation_sectors,1);
      for i = 1:obj.n_orientation_sectors
        obj.m_sth(i) = diff(sth(i:i+1))/diff(obj.th_constant(i:i+1));
        obj.m_cth(i) = diff(cth(i:i+1))/diff(obj.th_constant(i:i+1));
        obj.b_sth(i) = sth(i) - obj.m_sth(i)*obj.th_constant(i);
        obj.b_cth(i) = cth(i) - obj.m_cth(i)*obj.th_constant(i);
      end
    end
    
    function obj = addDynamicsConstraints(obj)
      A = zeros(2, obj.nv);
      b = zeros(2, 1);
      % ASSUMPTION obj.dim = 2
      ncons = ((4)*obj.dim + 4)*(obj.N-1);
      % END_ASSUMPTION
      Aeq = zeros(ncons, obj.nv);
      beq = zeros(ncons, 1);
      g = [0; -1];
      h = obj.dt;
      offset = 0;
      offset_eq = 0;
      for n = 1:(obj.N-1)
        % Position
        % r_next - r - h*v = h^2/2*F + h^2*g/2
        ncons = obj.dim;
        indices = offset_eq + (1:ncons);
        Aeq(indices, obj.vars.F.i(:,n,:)) = -h^2/2*repmat(eye(ncons), 1, obj.num_legs);
        Aeq(indices, obj.vars.r.i(:,n+1)') = eye(ncons);
        Aeq(indices, obj.vars.r.i(:,n)') = -eye(ncons);
        Aeq(indices, obj.vars.v.i(:,n)') = -h*eye(ncons);
        beq(indices) = (h^2/2)*g;
        offset_eq = offset_eq + ncons;

        % Velocity
        % v_next - v = h/2*(F+F_next) + h*g
        ncons = obj.dim;
        indices = offset_eq + (1:ncons);
        Aeq(indices, obj.vars.v.i(:,n+1)') = eye(obj.dim);
        Aeq(indices, obj.vars.v.i(:,n)') = -eye(obj.dim);
        Aeq(indices, obj.vars.F.i(:,n:n+1,:)) = -h/2*repmat([eye(obj.dim), eye(obj.dim)], 1, obj.num_legs);
        beq(indices) = h*g;
        offset_eq = offset_eq + ncons;

        % Angular position
        % th_next - th - h*w - h^2/(2*I)*T = 0
        % ASSUMPTION obj.dim = 2
        ncons = 1;
        indices = offset_eq + ncons;
        Aeq(indices, obj.vars.T.i(:,n,:)) = -h^2/(2*obj.I)*repmat(eye(ncons), 1, obj.num_legs);
        Aeq(indices, obj.vars.th.i(:,n+1)') = eye(ncons);
        Aeq(indices, obj.vars.th.i(:,n)') = -eye(ncons);
        Aeq(indices, obj.vars.w.i(:,n)') = -h*eye(ncons);
        beq(indices) = 0;
        offset_eq = offset_eq + ncons;
        % END_ASSUMPTION

        % Angular Velocity
        % w_next - w = h/(2*I)*(T+T_next)
        ncons = 1;
        indices = offset_eq + (1:ncons);
        Aeq(indices, obj.vars.w.i(:,n+1)') = eye(ncons);
        Aeq(indices, obj.vars.w.i(:,n)') = -eye(ncons);
        Aeq(indices, obj.vars.T.i(:,n:n+1,:)) = -h/(2*obj.I)*repmat([eye(ncons), eye(ncons)], 1, obj.num_legs);
        beq(indices) = 0;
        offset_eq = offset_eq + ncons;

        for j = 1:obj.num_legs
          % Foot position
          % r_next + r_hip_next + p_next - r - p - r_hip - h*pd = 0
          ncons = obj.dim;
          indices = offset_eq + (1:ncons);
          Aeq(indices, obj.vars.r.i(:,n+1)') = eye(ncons);
          Aeq(indices, obj.vars.r.i(:,n)') = -eye(ncons);          
          Aeq(indices, obj.vars.r_hip.i(:,n+1,j)') = eye(ncons);
          Aeq(indices, obj.vars.r_hip.i(:,n,j)') = -eye(ncons);            
          Aeq(indices, obj.vars.p.i(:,n+1,j)') = eye(ncons);
          Aeq(indices, obj.vars.p.i(:,n,j)') = -eye(ncons);            
          Aeq(indices, obj.vars.pd.i(:,n,j)) = -h*eye(ncons);
          beq(indices) = 0;
          offset_eq = offset_eq + ncons;

          if obj.use_foot_acceleration
            % Foot velocity
            ncons = obj.dim;
            indices = offset_eq + (1:ncons);
            Aeq(indices, obj.vars.pd.i(:,n+1,j)) = eye(ncons);
            Aeq(indices, obj.vars.pd.i(:,n,j)) = -eye(ncons);            
            Aeq(indices, obj.vars.pdd.i(:,n:n+1,j)) = -h/2*[eye(ncons), eye(ncons)];
            beq(indices) = 0;
            offset_eq = offset_eq + ncons;
          end
        end

        if obj.minimize_integral_of_squared_power
          % Incremental mechanical work
          % W - h*P_mid = 0
          % W - h*(F*v_mid + T*w_mid) = 0
          % W - h*(F*0.5*(v+v_next) + T*0.5*(w + w_next) = 0
          % W - 0.5*h*(PF_mid + PT_mid) = 0
          % where
          %   PF_mid = F*(v+v_next) (approximately)
          %   PT_mid = T*(w+w_next) (approximately)
          ncons = 1;
          indices = offset_eq + (1:ncons);
          Aeq(indices, obj.vars.W.i(n)) = 1;
          Aeq(indices, obj.vars.PF.i(:,n)) = -0.5*obj.dt;
          Aeq(indices, obj.vars.PT.i(n)) = -0.5*obj.dt;
          beq(indices) = 0;
          offset_eq = offset_eq + ncons;

          % Approximate power terms
          makePoint = @(f, e) [f; e; f.*e];
          ind_to_split = [];
          dim_to_split = [];
          for i = 0:ceil(log2(obj.M))
            ind_to_split = [ind_to_split, 2^i:-1:1]; %#ok
            dim_to_split = [dim_to_split, repmat(2, 1, 2^i)]; %#ok
          end
          ind_to_split(obj.M:end) = [];
          dim_to_split(obj.M:end) = [];
          for k = 1:obj.dim
            % sum_m(BPF(k,n,:) = 1
            ncons = 1;
            indices = offset_eq + (1:ncons);
            Aeq(indices, obj.vars.BPF.i(k,n,:)) = ones(1, obj.M);
            beq(indices) = 1;
            offset_eq = offset_eq + ncons;

            vertices = [makePoint(obj.vars.v.lb(k,n) + obj.vars.v.lb(k,n+1), obj.vars.F.lb(k,n)), ...
              makePoint(obj.vars.v.ub(k,n) + obj.vars.v.ub(k,n+1), obj.vars.F.lb(k,n)), ...
              makePoint(obj.vars.v.ub(k,n) + obj.vars.v.ub(k,n+1), obj.vars.F.ub(k,n)), ...
              makePoint(obj.vars.v.lb(k,n) + obj.vars.v.lb(k,n+1), obj.vars.F.ub(k,n))];
            vertices_cell = obj.splitTetrahedron(vertices, dim_to_split, ind_to_split);
            for m = 1:numel(vertices_cell)
              % BPF(k,n,m) --> [p(k,n); b(i,n); c(k,n,i)] lies in chull(vertices_cell{m})
              [A_local, b_local] = vert2lcon(vertices_cell{m}');
              A_local = [A_local(:,1), A_local]; %#ok
              point_indices = [obj.vars.v.i(k,n), obj.vars.v.i(k,n+1), obj.vars.F.i(k,n), obj.vars.PF.i(k,n)];
              big_M = 2*obj.vars.v.ub(k,n) + obj.vars.F.ub(k,n) + obj.vars.PF.ub(k,n);
              ncons = 4;
              indices = offset + (1:ncons);
              A(indices, point_indices) = A_local;
              A(indices, obj.vars.BPF.i(k,n,m)) = big_M*ones(ncons,1);
              b(indices) = b_local + big_M*ones(ncons,1);
              offset = offset + ncons;
            end
          end
          vertices = [makePoint(obj.vars.w.lb(n) + obj.vars.w.lb(n+1), obj.vars.T.lb(n)), ...
            makePoint(obj.vars.w.ub(n) + obj.vars.w.ub(n+1), obj.vars.T.lb(n)), ...
            makePoint(obj.vars.w.ub(n) + obj.vars.w.ub(n+1), obj.vars.T.ub(n)), ...
            makePoint(obj.vars.w.lb(n) + obj.vars.w.lb(n+1), obj.vars.T.ub(n))];
          vertices_cell = obj.splitTetrahedron(vertices, ind_to_split);
          % sum_m(BPT(1,n,:) = 1
          ncons = 1;
          indices = offset_eq + (1:ncons);
          Aeq(indices, obj.vars.BPT.i(1,n,:)) = ones(1, obj.M);
          beq(indices) = 1;
          offset_eq = offset_eq + ncons;
          for m = 1:numel(vertices_cell)
            % BPT(k,n,m) --> [w(n); T(n); PT(n)] lies in chull(vertices_cell{m})
            [A_local, b_local] = vert2lcon(vertices_cell{m}');
            A_local = [A_local(:,1), A_local]; %#ok
            point_indices = [obj.vars.w.i(n), obj.vars.w.i(n+1), obj.vars.T.i(n), obj.vars.PT.i(n)];
            big_M = 2*obj.vars.w.ub(n) + obj.vars.T.ub(n) + obj.vars.PT.ub(n);
            ncons = 4;
            indices = offset + (1:ncons);
            A(indices, point_indices) = A_local;
            A(indices, obj.vars.BPT.i(1,n,m)) = big_M*ones(ncons,1);
            b(indices) = b_local + big_M*ones(ncons,1);
            offset = offset + ncons;
          end
        end
      end
      obj = obj.addLinearConstraints(A, b, Aeq, beq);
    end

    function obj = addOrientationConstraints(obj)
      ncons_eq = (1 + obj.dim)*obj.N;
      ncons = (2 + 2)*obj.n_orientation_sectors*obj.N;
      A = zeros(ncons, obj.nv);
      b = zeros(ncons, 1);
      offset = 0;
      offset_eq = 0;
      Aeq = zeros(ncons_eq, obj.nv);
      beq = zeros(ncons_eq, 1);
      for n = 1:obj.N
        Aeq(offset_eq+1, obj.vars.S.i(:, n)) = 1;
        beq(offset_eq+1) = 1;
        offset_eq = offset_eq + 1;

        for j = 1:obj.num_legs
          % Hip position
          %  r_hip == [cth, sth; -sth, cth]*hip
          %  r_hip == [cth*hip(1) + sth*hip(2); -sth*hip(1) + cth*hip(2)]
          %  r_hip == [hip(1), hip(2); hip(2), -hip(1)]*[cth; sth]
          ncons = obj.dim;
          indices = offset_eq + (1:ncons);
          Aeq(indices, obj.vars.r_hip.i(:, n, j)) = -eye(obj.dim);
          Aeq(indices, [obj.vars.cth.i(n), obj.vars.sth.i(n)]) = ...
            [obj.hip_in_body(1,j), obj.hip_in_body(2,j); ...
            obj.hip_in_body(2,j),  -obj.hip_in_body(1,j)];
          beq(indices) = 0;
          offset_eq = offset_eq + ncons;
        end

        
        for i = 1:obj.n_orientation_sectors
          % S(i,n) --> th^i <= th(n) <= th^(i+1)
          % formulated as
          %   -th(n) <= -th^i     + (1 - S(i,n))*big_M
          %    th(n) <=  th^(i+1) + (1 - S(i,n))*big_M
          ncons = 2;
          indices = offset + (1:ncons);
          big_M = 2*obj.rotation_max;
          A(indices, obj.vars.th.i(n)) = [-1; 1];
          A(indices, obj.vars.S.i(i,n)) = big_M*ones(ncons,1);
          b(indices) = [-obj.th_constant(i); obj.th_constant(i+1)] + big_M*ones(ncons, 1);
          offset = offset + ncons;
          
          % S(i,n) --> sth(n) - m_sth^i*th == b_sth^i 
          % formulated as
          %    sth(n) - m_sth^i*th(n) <=  b_sth^i + (1 - S(i,n))*big_M
          %   -sth(n) + m_sth^i*th(n) <= -b_sth^i + (1 - S(i,n))*big_M
          ncons = 2;
          indices = offset + (1:ncons);
          A(indices, obj.vars.sth.i(n)) = [1; -1];
          A(indices, obj.vars.th.i(n))  = obj.m_sth(i)*[-1; 1];
          A(indices, obj.vars.S.i(i,n)) = big_M*ones(ncons, 1);
          b(indices) = obj.b_sth(i)*[1; -1] + big_M*ones(ncons, 1);
          offset = offset + ncons;
          
          % S(i,n) --> cth(n) - m_cth^i*th == b_cth^i 
          % formulated as
          %    cth(n) - m_cth^i*th(n) <=  b_cth^i + (1 - S(i,n))*big_M
          %   -cth(n) + m_cth^i*th(n) <= -b_cth^i + (1 - S(i,n))*big_M
          ncons = 2;
          indices = offset + (1:ncons);
          A(indices, obj.vars.cth.i(n)) = [1; -1];
          A(indices, obj.vars.th.i(n))  = obj.m_cth(i)*[-1; 1];
          A(indices, obj.vars.S.i(i,n)) = big_M*ones(ncons, 1);
          b(indices) = obj.b_cth(i)*[1; -1] + big_M*ones(ncons, 1);
          offset = offset + ncons;
        end
      end
      obj = obj.addLinearConstraints(A, b, Aeq, beq);
    end
    
    function obj = addContactPointConstraints(obj)
      for n = 1:obj.N
        for k = 1:obj.num_legs
          A_cell = cell(1, obj.n_regions);
          b_cell = cell(1, obj.n_regions);
          for j = 1:numel(obj.regions)
            if ~isempty(obj.regions(j).A)
              A_cell{j} = obj.regions(j).A;
              b_cell{j} = obj.regions(j).b;
            end
            if ~isempty(obj.regions(j).Aeq)
              A_cell{j} = [A_cell{j}; obj.regions(j).Aeq];
              b_cell{j} = [b_cell{j}; obj.regions(j).beq];
              A_cell{j} = [A_cell{j}; -obj.regions(j).Aeq];
              b_cell{j} = [b_cell{j}; -obj.regions(j).beq];
            end
          end
          name = sprintf('foot_%d_region_at_knot_%d', k, n);
          obj = obj.addDisjunctiveConstraint({'r_foot'}, ...
                                             {{':',n,k}}, ...
                                             A_cell, b_cell, name, ...
                                             obj.vars.R.i(:, n, k));
        end
      end
      A = zeros(2, obj.nv);
      b = zeros(2, 1);
      offset = 0;
      Aeq = zeros(2, obj.nv);
      beq = zeros(2, 1);
      offset_eq = 0;
      for n = 1:obj.N
        n_plus_1_or_N = n;%min(n+1, obj.N);
        for k = 1:obj.num_legs
          % Define r_foot
          ncons = obj.dim;
          indices = offset_eq + (1:ncons);
          Aeq(indices, obj.vars.r_foot.i(:, n, k)) = -eye(obj.dim);
          Aeq(indices, obj.vars.r.i(:, n)) = eye(obj.dim);
          Aeq(indices, obj.vars.r_hip.i(:, n, k)) = eye(obj.dim);
          Aeq(indices, obj.vars.p.i(:, n, k)) = eye(obj.dim);
          beq(indices) = 0;
          offset_eq = offset_eq + ncons;

          for j = 1:numel(obj.regions)
            % Rj --> F - sum_i(bi*Fij) == 0
            % formulated as
            %   F - sum_i(bi*Fij) <= (1 - Rj)*M
            %  -F + sum_i(bi*Fij) <= (1 - Rj)*M
            big_M = (1 + obj.n_basis_vectors)*obj.force_max;
            ncons = 2*obj.dim;
            indices = offset + (1:ncons);
            A(indices, obj.vars.F.i(:,n,k)) = [eye(obj.dim); -eye(obj.dim)];
            A(indices, obj.vars.b.i(:,n,k)) = [-obj.regions(j).basis_vectors; obj.regions(j).basis_vectors];
            A(indices, obj.vars.R.i(j, n_plus_1_or_N, k)) = big_M*ones(ncons,1);
            b(indices) = big_M*ones(ncons,1);
            offset = offset + ncons;

            % ASSUME obj.dim = 2
            % Rj --> T - sum_i(-Fij(2)*ci(1) + Fij(1)*ci(2)) = 0
            % formulated as
            %    T + sum_i(Fij(2)*ci(1) - Fij(1)*ci(2)) <= (1 - Rj)*big_M
            %   -T - sum_i(Fij(2)*ci(1) - Fij(1)*ci(2)) <= (1 - Rj)*big_M
            ncons = 2;
            indices = offset + (1:ncons);
            big_M = obj.moment_max*(1 + obj.n_basis_vectors);
            A(indices, obj.vars.T.i(:, n, k)) = [1; -1];
            A(indices, obj.vars.c.i(1, n, :, k)) =  [obj.regions(j).basis_vectors(2,:); -obj.regions(j).basis_vectors(2,:)];
            A(indices, obj.vars.c.i(2, n, :, k)) =  [-obj.regions(j).basis_vectors(1,:); obj.regions(j).basis_vectors(1,:)];
            A(indices, obj.vars.R.i(j, n_plus_1_or_N, k)) = big_M*ones(ncons,1);
            b(indices) = big_M*ones(ncons,1);
            offset = offset + ncons;
            % END_ASSUME
            
            if ~isempty(obj.regions(j).A)
              if n < obj.N && isempty(obj.regions(j).normal)
                ncons = size(obj.regions(j).A, 1);
                indices = offset + (1:ncons);
                A(indices, obj.vars.r.i(:,n+1)) = obj.regions(j).A;
                A(indices, obj.vars.r_hip.i(:,n+1,k)) = obj.regions(j).A;
                A(indices, obj.vars.p.i(:,n+1,k)) = obj.regions(j).A;
                A(indices, obj.vars.R.i(j, n, k)) = big_M*ones(ncons,1);
                b(indices) = obj.regions(j).b + big_M;
                offset = offset + ncons;
              end
              if n > 1 && isempty(obj.regions(j).normal)
                ncons = size(obj.regions(j).A, 1);
                indices = offset + (1:ncons);
                A(indices, obj.vars.r.i(:,n-1)) = obj.regions(j).A;
                A(indices, obj.vars.r_hip.i(:,n-1,k)) = obj.regions(j).A;
                A(indices, obj.vars.p.i(:,n-1,k)) = obj.regions(j).A;
                A(indices, obj.vars.R.i(j, n, k)) = big_M*ones(ncons,1);
                b(indices) = obj.regions(j).b + big_M;
                offset = offset + ncons;
              end
            end
            if ~isempty(obj.regions(j).Aeq)
            end
            if ~isempty(obj.regions(j).normal)
              % R(j,n) --> pd == 0
              % formulated as
              %  pd <= big_M*(1 - R(j,n))
              %  pd <= big_M*(1 - R(j,n))
              big_M = 2*obj.velocity_max;
              ncons = 2*obj.dim;
              indices = offset + (1:ncons);
              A(indices, obj.vars.pd.i(:,n,k)) = [eye(obj.dim); -eye(obj.dim)];
              A(indices, obj.vars.R.i(j, n, k)) = big_M*ones(ncons,1);
              b(indices) = big_M*ones(ncons,1);
              offset = offset + ncons;
            end
          end
        end
      end
      obj = obj.addLinearConstraints(A, b, Aeq, beq);
    end
    
    function obj = addPositionConstraint(obj, time_index, lb, ub)
      obj.vars.r.lb(:, time_index) = lb;
      obj.vars.r.ub(:, time_index) = ub;
    end
    
    function obj = addOrientationConstraint(obj, time_index, lb, ub)
      obj.vars.th.lb(:, time_index) = lb;
      obj.vars.th.ub(:, time_index) = ub;
    end
    
    function obj = addVelocityConstraint(obj, time_index, lb, ub)
      obj.vars.v.lb(:, time_index) = lb;
      obj.vars.v.ub(:, time_index) = ub;
    end
    
    function obj = addAngularVelocityConstraint(obj, time_index, lb, ub)
      obj.vars.w.lb(:, time_index) = lb;
      obj.vars.w.ub(:, time_index) = ub;
    end
    
    function obj = addRegion(obj, A, b, Aeq, beq, normal, mu)
      j = numel(obj.regions)+1;
      if ~isempty(A)
        obj.regions(j).A = A;
      end
      if ~isempty(b)
        obj.regions(j).b = b;
      end
      if ~isempty(Aeq)
        obj.regions(j).Aeq = Aeq;
      end
      if ~isempty(beq)
        obj.regions(j).beq = beq;
      end
      if ~isempty(mu)
        obj.regions(j).mu = mu;
      else
        obj.regions(j).mu = 1;
      end
      if ~isempty(normal)
        normal = normalizeVec(normal);
        obj.regions(j).normal = normal;
        tangent = cross([0;1;0], [normal(1); 0; normal(2)]);
        tangent(2) = [];
        % HACK
        obj.regions(j).basis_vectors = [normalizeVec(obj.regions(j).mu*tangent + normal), normalizeVec(-obj.regions(j).mu*tangent + normal)];
%         obj.regions(j).basis_vectors = normal;
        % END_HACK
      else
        obj.regions(j).basis_vectors = zeros(obj.dim, obj.n_basis_vectors);
      end
      obj.regions(j).ncons = size(obj.regions(j).A, 1) ...
                            + 2*size(obj.regions(j).Aeq,1);
      obj.n_regions = j;
    end
  end
end
