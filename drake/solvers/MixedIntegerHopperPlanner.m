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
  end
  
  methods
    function obj = MixedIntegerHopperPlanner(I, N, dt)
      obj = obj@MixedIntegerConvexProgram(false);
      obj.I = I;
      obj.N = N;
      obj.dt = dt;
    end
    
    function obj = setupProblem(obj)
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
        [obj.dim, obj.N], -1, 1);
      obj = obj.addVariable('S', 'B', [obj.n_orientation_sectors, obj.N], 0, 1);
      obj = obj.addVariable('p', 'C', [obj.dim, obj.N], repmat(sqrt(2)/2*[-1; -1], [1, obj.N]), repmat(sqrt(2)/2*[1; -0.5], [1, obj.N]));
      obj = obj.addVariable('pd', 'C', [obj.dim, obj.N], -obj.velocity_max, obj.velocity_max);
      obj = obj.addVariable('F', 'C', ...
        [obj.dim, obj.N], -obj.force_max, obj.force_max);
      obj = obj.addVariable('T', 'C', ...
        [1, obj.N], -obj.moment_max, obj.moment_max);
      obj = obj.addVariable('b', 'C', ...
        [obj.n_basis_vectors, obj.N], 0, obj.force_max);
      obj = obj.addVariable('c', 'C', ...
        [obj.dim, obj.N, obj.n_basis_vectors], -obj.moment_max, obj.moment_max);
      obj = obj.addVariable('B', 'B', ...
        [obj.dim, obj.N, obj.M], 0, 1);
      obj = obj.addVariable('R', 'B', [obj.n_regions, obj.N], 0, 1);
      
      makePoint = @(p, b) [p; b; p.*b];
      default_ind_to_split = [];
      for i = 0:ceil(log2(obj.M))
        default_ind_to_split = [default_ind_to_split, 2^i:-1:1]; %#ok
      end
      default_ind_to_split(obj.M:end) = [];
      ncons_total = obj.dim*obj.N*obj.n_basis_vectors*obj.M*4;
      A = zeros(ncons_total, obj.nv);
      b = zeros(ncons_total, 1);
      ncons_total_eq = obj.dim*obj.N*obj.n_basis_vectors;
      Aeq = zeros(ncons_total_eq, obj.nv);
      beq = zeros(ncons_total_eq, 1);
      offset = 0;
      offset_eq = 0;
      for i = 1:obj.n_basis_vectors
        for n = 1:obj.N
          for k = 1:obj.dim
            % sum_m(B(k,n,:) = 1
            ncons = 1;
            indices = offset_eq + (1:ncons);
            Aeq(indices, obj.vars.B.i(k,n,:)) = ones(1, obj.M);
            beq(indices) = 1;
            offset_eq = offset_eq + ncons;

            if isempty(obj.c_approx_splits)
              ind_to_split = default_ind_to_split;
            else
              ind_to_split = obj.c_approx_splits{k,n,i};
              assert(numel(ind_to_split) == obj.M-1);
            end
            vertices = [makePoint(obj.vars.r_hip.lb(k,n) + obj.vars.p.lb(k,n), 0), ...
                        makePoint(obj.vars.r_hip.ub(k,n) + obj.vars.p.ub(k,n), 0), ...
                        makePoint(obj.vars.r_hip.ub(k,n) + obj.vars.p.ub(k,n), obj.vars.b.ub(i,n)), ...
                        makePoint(obj.vars.r_hip.lb(k,n) + obj.vars.p.lb(k,n), obj.vars.b.ub(i,n))];
            if isempty(ind_to_split)
              vertices_cell = {vertices};
            else
              vertices_cell = obj.splitTetrahedron(vertices, ind_to_split);
            end
            for m = 1:numel(vertices_cell)
              % B(k,n,m) --> [p(k,n); b(i,n); c(k,n,i)] lies in chull(vertices_cell{m})
              [A_local, b_local] = vert2lcon(vertices_cell{m}');
              A_local = [A_local(:,1), A_local]; %#ok
              point_indices = [obj.vars.r_hip.i(k,n), obj.vars.p.i(k,n), obj.vars.b.i(i,n), obj.vars.c.i(k,n,i)];
              big_M = obj.vars.p.ub(k,n) + obj.vars.b.ub(i,n) + obj.vars.c.ub(k,n,i);
              ncons = 4;
              indices = offset + (1:ncons);
              A(indices, point_indices) = A_local;
              A(indices, obj.vars.B.i(k,n,m)) = big_M*ones(ncons,1);
              b(indices) = b_local + big_M*ones(ncons,1);
              offset = offset + ncons;
            end
          end
        end
      end
      obj = obj.addLinearConstraints(A, b, Aeq, beq);

      obj = obj.setupOrientationSectors();
      
      obj = obj.addDynamicsConstraints();
      obj = obj.addForceConstraints();
      obj = obj.addContactPointConstraints(); 
      obj = obj.addOrientationConstraints();
      %obj = obj.addBinaryVariableConstraints(); 
      
      Q = zeros(obj.nv);
      c = zeros(obj.nv, 1);
      alpha = 0;
      Q(obj.vars.b.i(:), obj.vars.b.i(:)) = eye(numel(obj.vars.b.i));
      %Q(obj.vars.c.i(:), obj.vars.c.i(:)) = eye(numel(obj.vars.c.i));
      %Q(obj.vars.T.i(:), obj.vars.T.i(:)) = 10*eye(numel(obj.vars.T.i));
      %Q(obj.vars.pd.i(:), obj.vars.pd.i(:)) = eye(numel(obj.vars.pd.i));
      A = spdiags([ones(obj.N-1, 1), ones(obj.N-1, 1)], [0,1], sparse(obj.N-1, obj.N));
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
      offset = 0;
      % ASSUMPTION obj.dim = 2
      ncons = ((2+1)*obj.dim + 2)*(obj.N-1);
      % END_ASSUMPTION
      Aeq = zeros(ncons, obj.nv);
      beq = zeros(ncons, 1);
      g = [0; -1];
      h = obj.dt;
      for n = 1:(obj.N-1)
        % Position
        % r_next - r - h*v = h^2/2*F + h^2*g/2
        ncons = obj.dim;
        indices = offset + (1:ncons);
        Aeq(indices, obj.vars.F.i(:,n)) = -h^2/2*eye(ncons);
        Aeq(indices, obj.vars.r.i(:,n+1)') = eye(ncons);
        Aeq(indices, obj.vars.r.i(:,n)') = -eye(ncons);
        Aeq(indices, obj.vars.v.i(:,n)') = -h*eye(ncons);
        beq(indices) = (h^2/2)*g;
        offset = offset + ncons;
        
        % Velocity
        % v_next - v = h/2*(F+F_next) + h*g
        ncons = obj.dim;
        indices = offset + (1:ncons);
        Aeq(indices, obj.vars.v.i(:,n+1)') = eye(obj.dim);
        Aeq(indices, obj.vars.v.i(:,n)') = -eye(obj.dim);
        Aeq(indices, obj.vars.F.i(:,n:n+1)) = -h/2*[eye(obj.dim), eye(obj.dim)];
        beq(indices) = h*g;
        offset = offset + ncons;
        
        % Angular position
        % th_next - th - h*w - h^2/(2*I)*T = 0
        % ASSUMPTION obj.dim = 2
        ncons = 1;
        indices = offset + ncons;
        Aeq(indices, obj.vars.T.i(:,n)) = -h^2/(2*obj.I)*eye(ncons);
        Aeq(indices, obj.vars.th.i(:,n+1)') = eye(ncons);
        Aeq(indices, obj.vars.th.i(:,n)') = -eye(ncons);
        Aeq(indices, obj.vars.w.i(:,n)') = -h*eye(ncons);
        offset = offset + ncons;
        % END_ASSUMPTION
        
        % Angular Velocity
        % w_next - w = h/(2*I)*(T+T_next)
        ncons = 1;
        indices = offset + (1:ncons);
        Aeq(indices, obj.vars.w.i(:,n+1)') = eye(ncons);
        Aeq(indices, obj.vars.w.i(:,n)') = -eye(ncons);
        Aeq(indices, obj.vars.T.i(:,n:n+1)) = -h/(2*obj.I)*[eye(ncons), eye(ncons)];
        offset = offset + ncons;

        % Foot position
        % r_next + r_hip_next + p_next - r - p - r_hip - h/2*(pd + pd_next) = 0
        ncons = obj.dim;
        indices = offset + (1:ncons);
        Aeq(indices, obj.vars.r.i(:,n+1)') = eye(ncons);
        Aeq(indices, obj.vars.r.i(:,n)') = -eye(ncons);          
        Aeq(indices, obj.vars.r_hip.i(:,n+1)') = eye(ncons);
        Aeq(indices, obj.vars.r_hip.i(:,n)') = -eye(ncons);            
        Aeq(indices, obj.vars.p.i(:,n+1)') = eye(ncons);
        Aeq(indices, obj.vars.p.i(:,n)') = -eye(ncons);            
        Aeq(indices, obj.vars.pd.i(:,n:n+1)) = -h/2*[eye(ncons), eye(ncons)];
        offset = offset + ncons;
      end
      obj = obj.addLinearConstraints([], [], Aeq, beq);
    end
    
    function obj = addForceConstraints(obj)
      ncons = (2*obj.dim + 2)*obj.n_regions*obj.N;%; + (2 + 2)*obj.n_basis_vectors*(4*obj.M)*obj.N + (4 + 2*obj.dim)*obj.n_basis_vectors*obj.N;
      ncons_eq = obj.N;
      A = zeros(ncons, obj.nv);
      b = zeros(ncons, 1);
      Aeq = zeros(ncons_eq, obj.nv);
      beq = zeros(ncons_eq, 1);
      offset = 0;
      offset_eq = 0;
      for n = 1:obj.N
        n_plus_1_or_N = n;%min(n+1, obj.N);
        for j = 1:obj.n_regions
          % Rj --> F - sum_i(bi*Fij) == 0
          % formulated as
          %   F - sum_i(bi*Fij) <= (1 - Rj)*M
          %  -F + sum_i(bi*Fij) <= (1 - Rj)*M
          big_M = (1 + obj.n_basis_vectors)*obj.force_max;
          ncons = 2*obj.dim;
          indices = offset + (1:ncons);
          A(indices, obj.vars.F.i(:,n)) = [eye(obj.dim); -eye(obj.dim)];
          A(indices, obj.vars.b.i(:,n)) = [-obj.regions(j).basis_vectors; obj.regions(j).basis_vectors];
          A(indices, obj.vars.R.i(j,n_plus_1_or_N)) = big_M*ones(ncons,1);
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
          A(indices, obj.vars.T.i(:, n)) = [1; -1];
          A(indices, obj.vars.c.i(1, n, :)) =  [obj.regions(j).basis_vectors(2,:); -obj.regions(j).basis_vectors(2,:)];
          A(indices, obj.vars.c.i(2, n, :)) =  [-obj.regions(j).basis_vectors(1,:); obj.regions(j).basis_vectors(1,:)];
          A(indices, obj.vars.R.i(j,n_plus_1_or_N)) = big_M*ones(ncons,1);
          b(indices) = big_M*ones(ncons,1);
          offset = offset + ncons;
          % END_ASSUME
        end
        
        for i = 1:obj.n_basis_vectors
          % c <= slope*b
          % c >= -slope*b <--> -c <= slope*b
          %ncons = 4;
          %indices = offset + (1:ncons);
          %slope = obj.position_max;
          %A(indices, obj.vars.c.i(:, n, i)) = [eye(obj.dim); -eye(obj.dim)];
          %A(indices, obj.vars.b.i(i, n)) = -slope*ones(ncons, 1);
          %offset = offset + ncons;

          % g'*[p; b; c] <= f
          %for field = {'lb', 'ub'}
            %p_bound = field{1};
            %for k = 1:obj.dim
              %v1 = [0; obj.vars.b.ub(i,n); 0];
              %v2 = [obj.vars.p.(p_bound)(k,n); obj.vars.b.ub(i,n); obj.vars.p.(p_bound)(k,n)*obj.vars.b.ub(i,n)];
              %v3 = [obj.vars.p.(p_bound)(k,n); 0; 0];
              %g = cross(v2 - v1, v3 - v1);
              %f = g'*v1;

              %ncons = 1;
              %indices = offset + (1:ncons);
              %A(indices, [obj.vars.p.i(k,n), obj.vars.b.i(i,n), obj.vars.c.i(k,n,i)]) = g;
              %b(indices) = f;
              %offset = offset + ncons;
            %end
          %end

%         for m = 1:2*obj.M
%           %ci = bi*(r_hip + p)
%           % approximated by
%           %  (1) Bim --> b^(m-1) <= bi <= b^m for i = 1, ..., obj.n_basis_vectors
%           %   formulated as
%           %     -bi <= -b^(m-1) + (1 - Bim)*big_M
%           %     bi <= b^m + (1 - Bim)*big_M
%           % and
%           %   (2) Bim --> b^(m-1)*p + b^(m-1)*r_hip <= ci <= b^m*p + b^(m)*r_hip for i = 1, ..., obj.n_basis_vectors
%           %   formulated as
%           %      b^(m-1)*p + b^(m-1)*r_hip - ci <= (1 - Bim)*big_M
%           %     -b^m*p - b^(m)r_hip + ci <= (1 - Bim)*big_M
%           
%           % (1)
%           ncons = 2;
%           indices = offset + (1:ncons);
%           big_M = 2*obj.force_max;
%           A(indices, obj.vars.b.i(i, n)) = [-1; 1];
%           A(indices, obj.vars.B.i(m, n, i)) = big_M*ones(ncons, 1);
%           b(indices) = [-obj.b_constant(m); obj.b_constant(m+2)] + big_M;
%           offset = offset + ncons;

%           ncons = 2;
%           indices = offset + (1:ncons);
%           big_M = 2*obj.force_max;
%           A(indices, obj.vars.b.i(i, n)) = [-1; 1];
%           A(indices, obj.vars.Bz.i(m, n, i)) = big_M*ones(ncons, 1);
%           b(indices) = [-obj.b_constant(m); obj.b_constant(m+2)] + big_M;
%           offset = offset + ncons;
%           
%           % (2)
%           ncons = 2;
%           indices = offset + (1:ncons);
%           big_M = 2*obj.moment_max;
%           if mod(m, 2) == 1
%             A(indices, obj.vars.p.i(1,n)) = [obj.b_constant(m)*eye(1); ...
%                                              -obj.b_constant(m+2)*eye(1)];
%             A(indices, obj.vars.r_hip.i(1,n)) = [obj.b_constant(m)*eye(1); ...
%                                              -obj.b_constant(m+2)*eye(1)];
%           else
%             A(indices, obj.vars.p.i(1,n)) = [obj.b_constant(m+2)*eye(1); ...
%                                              -obj.b_constant(m)*eye(1)];
%             A(indices, obj.vars.r_hip.i(1,n)) = [obj.b_constant(m+2)*eye(1); ...
%                                              -obj.b_constant(m)*eye(1)];
%           end
%           %b_mid = mean(obj.b_constant(m:m+1));
%           %A(indices, obj.vars.p.i(:,n)) = [b_mid*eye(1); ...
%                                           %-b_mid*eye(1)];
%           A(indices, obj.vars.c.i(1, n, i)) = [-eye(1); eye(1)];
%           A(indices, obj.vars.B.i(m, n, i)) = big_M*ones(ncons, 1);
%           b(indices) = big_M;
%           offset = offset + ncons;

%           ncons = 2;
%           indices = offset + (1:ncons);
%           big_M = 2*obj.moment_max;
%           if mod(m, 2) == 1
%             A(indices, obj.vars.p.i(2,n)) = [obj.b_constant(m)*eye(1); ...
%                                              -obj.b_constant(m+2)*eye(1)];
%             A(indices, obj.vars.r_hip.i(2,n)) = [obj.b_constant(m)*eye(1); ...
%                                              -obj.b_constant(m+2)*eye(1)];
%           else
%             A(indices, obj.vars.p.i(2,n)) = [obj.b_constant(m+2)*eye(1); ...
%                                              -obj.b_constant(m)*eye(1)];
%             A(indices, obj.vars.r_hip.i(2,n)) = [obj.b_constant(m+2)*eye(1); ...
%                                              -obj.b_constant(m)*eye(1)];
%           end
%           %b_mid = mean(obj.b_constant(m:m+1));
%           %A(indices, obj.vars.p.i(:,n)) = [b_mid*eye(1); ...
%                                           %-b_mid*eye(1)];
%           A(indices, obj.vars.c.i(2, n, i)) = [-eye(1); eye(1)];
%           A(indices, obj.vars.Bz.i(m, n, i)) = big_M*ones(ncons, 1);
%           b(indices) = big_M;
%           offset = offset + ncons;
%         end
        end
        
        % sum_j(Rj) = 1
        Aeq(offset_eq + 1, obj.vars.R.i(:,n)) = ones(1, obj.n_regions);
        beq(offset_eq + 1) = 1;
        offset_eq = offset_eq + 1;
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

        % Hip position
        %  r_hip == [cth, sth; -sth, cth]*hip
        %  r_hip == [cth*hip(1) + sth*hip(2); -sth*hip(1) + cth*hip(2)]
        %  r_hip == [hip(1), hip(2); hip(2), -hip(1)]*[cth; sth]
        ncons = obj.dim;
        indices = offset_eq + (1:ncons);
        Aeq(indices, obj.vars.r_hip.i(:, n)) = -eye(obj.dim);
        Aeq(indices, [obj.vars.cth.i(n), obj.vars.sth.i(n)]) = ...
          [obj.hip_in_body(1), obj.hip_in_body(2); ...
           obj.hip_in_body(2),  -obj.hip_in_body(1)];
        offset_eq = offset_eq + ncons;

        
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
      big_M = obj.position_max;
      
      n_free_regions = sum(cellfun(@isempty, {obj.regions.normal}));
      n_contact_regions = numel(obj.regions) - n_free_regions;
      ncons = 2*sum([obj.regions.ncons])*(obj.N-1) + sum([obj.regions.ncons]) + obj.dim*n_contact_regions*obj.N;
      A = zeros(ncons, obj.nv);
      b = zeros(ncons, 1);
      offset = 0;
      for n = 1:obj.N
        for j = 1:numel(obj.regions)
          if ~isempty(obj.regions(j).A)
            % R(j,n) -> A*(r + r_hip + p) <= b
            % formulated as
            % A*r + A*r_hip + A*p <= b + big_M*(1 - R(j,n))
            ncons = size(obj.regions(j).A, 1);
            indices = offset + (1:ncons);
            A(indices, obj.vars.r.i(:,n)) = obj.regions(j).A;
            A(indices, obj.vars.r_hip.i(:,n)) = obj.regions(j).A;
            A(indices, obj.vars.p.i(:,n)) = obj.regions(j).A;
            A(indices, obj.vars.R.i(j,n)) = big_M*ones(ncons,1);
            b(indices) = obj.regions(j).b + big_M;
            offset = offset + ncons;

            if n < obj.N && isempty(obj.regions(j).normal)
              ncons = size(obj.regions(j).A, 1);
              indices = offset + (1:ncons);
              A(indices, obj.vars.r.i(:,n+1)) = obj.regions(j).A;
              A(indices, obj.vars.r_hip.i(:,n+1)) = obj.regions(j).A;
              A(indices, obj.vars.p.i(:,n+1)) = obj.regions(j).A;
              A(indices, obj.vars.R.i(j,n)) = big_M*ones(ncons,1);
              b(indices) = obj.regions(j).b + big_M;
              offset = offset + ncons;
            end
            if n > 1 && isempty(obj.regions(j).normal)
              ncons = size(obj.regions(j).A, 1);
              indices = offset + (1:ncons);
              A(indices, obj.vars.r.i(:,n-1)) = obj.regions(j).A;
              A(indices, obj.vars.r_hip.i(:,n-1)) = obj.regions(j).A;
              A(indices, obj.vars.p.i(:,n-1)) = obj.regions(j).A;
              A(indices, obj.vars.R.i(j,n)) = big_M*ones(ncons,1);
              b(indices) = obj.regions(j).b + big_M;
              offset = offset + ncons;
            end
          end
          if ~isempty(obj.regions(j).Aeq)
            % R(j,n) -> Aeq*(r + r_hip + p) == beq
            % formulated as
            % A*p + A*r + A*r_hip <= b + big_M*(1 - R(j,n))
            % -A*p - A*r - A*r_hip <= -b + big_M*(1 - R(j,n))
            ncons = 2*size(obj.regions(j).Aeq, 1);
            indices = offset + (1:ncons);
            A(indices, obj.vars.r.i(:,n)) = [obj.regions(j).Aeq; -obj.regions(j).Aeq];
            A(indices, obj.vars.r_hip.i(:,n)) = [obj.regions(j).Aeq; -obj.regions(j).Aeq];
            A(indices, obj.vars.p.i(:,n)) = [obj.regions(j).Aeq; -obj.regions(j).Aeq];
            A(indices, obj.vars.R.i(j,n)) = big_M*ones(ncons,1);
            b(indices) = [obj.regions(j).beq; -obj.regions(j).beq] + big_M;
            offset = offset + ncons;

            %if n < obj.N
              %ncons = 2*size(obj.regions(j).Aeq, 1);
              %indices = offset + (1:ncons);
              %A(indices, obj.vars.r.i(:,n+1)) = [obj.regions(j).Aeq; -obj.regions(j).Aeq];
              %A(indices, obj.vars.r_hip.i(:,n+1)) = [obj.regions(j).Aeq; -obj.regions(j).Aeq];
              %A(indices, obj.vars.p.i(:,n+1)) = [obj.regions(j).Aeq; -obj.regions(j).Aeq];
              %A(indices, obj.vars.R.i(j,n)) = big_M*ones(ncons,1);
              %b(indices) = [obj.regions(j).beq; -obj.regions(j).beq] + big_M;
              %offset = offset + ncons;
            %end
          end
          if ~isempty(obj.regions(j).normal)
            % R(j,n) --> v + pd == 0
            % formulated as
            %   v + pd <= big_M*(1 - R(j,n))
            %  -v - pd <= big_M*(1 - R(j,n))
            big_M = 2*obj.velocity_max;
            ncons = 2*obj.dim;
            indices = offset + (1:ncons);
%             A(indices, obj.vars.v.i(:,n)) = [eye(obj.dim); -eye(obj.dim)];
            A(indices, obj.vars.pd.i(:,n)) = [eye(obj.dim); -eye(obj.dim)];
            A(indices, obj.vars.R.i(j,n)) = big_M*ones(ncons,1);
            b(indices) = big_M*ones(ncons,1);
            offset = offset + ncons;
          end
        end
      end
      obj = obj.addLinearConstraints(A, b, [], []);
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
        obj.regions(j).normal = normal;
        % HACK
        obj.regions(j).basis_vectors = [normalizeVec([obj.regions(j).mu;1]), normalizeVec([-obj.regions(j).mu;1])];
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

  methods (Static)
    function vertices_cell = splitTetrahedron(vertices, ind_to_split)
      if nargin < 2
        mid_12 = mean(vertices(:,[1,2]),2);
        mid_34 = mean(vertices(:,[3,4]),2);
        vertices_cell{1} = [vertices(:,1), mid_12, mid_34, vertices(:,4)];
        vertices_cell{2} = [mid_12, vertices(:,2), vertices(:,3), mid_34];
      else
        if isnumeric(vertices), vertices = {vertices}; end
        if isscalar(ind_to_split)
          vertices_cell = [vertices(1:ind_to_split-1), MixedIntegerHopperPlanner.splitTetrahedron(vertices{ind_to_split}), vertices(ind_to_split+1:end)];
        else
          vertices_cell = MixedIntegerHopperPlanner.splitTetrahedron(MixedIntegerHopperPlanner.splitTetrahedron(vertices, ind_to_split(1)), ind_to_split(2:end));
        end
      end
    end
  end
end
