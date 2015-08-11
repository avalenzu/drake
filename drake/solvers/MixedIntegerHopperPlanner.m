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
    n_regions
    b_constant
    N
    M = 10% number of segments for approximating hyperbolic paraboloids
    dim = 2 % planar case
    dt
    I % dimensionless moment of inertia
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
      obj = obj.addVariable('p', 'C', [obj.dim, obj.N], repmat([-1; -1], [1, obj.N]), repmat([1; -0.5], [1, obj.N]));
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
        [2*obj.M, obj.N, obj.n_basis_vectors], 0, 1);
      obj = obj.addVariable('Bz', 'B', ...
        [2*obj.M, obj.N, obj.n_basis_vectors], 0, 1);
      obj = obj.addVariable('R', 'B', [obj.n_regions, obj.N], 0, 1);
      
      alpha = atan(obj.force_max);
      obj.b_constant = repmat(tan(alpha*linspace(0, 1, obj.M+1)), 2, 1);
      
      obj = obj.addDynamicsConstraints();
      obj = obj.addForceConstraints();
      obj = obj.addContactPointConstraints(); 
      %obj = obj.addBinaryVariableConstraints(); 
      
      Q = zeros(obj.nv);
      c = zeros(obj.nv, 1);
      alpha = 0;
      Q(obj.vars.b.i(:), obj.vars.b.i(:)) = 10*eye(numel(obj.vars.F.i));
      Q(obj.vars.T.i(:), obj.vars.T.i(:)) = 10*eye(numel(obj.vars.T.i));
      Q(obj.vars.pd.i(:), obj.vars.pd.i(:)) = eye(numel(obj.vars.pd.i));
      obj = obj.addCost(Q, c, alpha);
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
        % p_next - p - h/2*(pd + pd_next) = 0
        ncons = obj.dim;
        indices = offset + (1:ncons);
        Aeq(indices, obj.vars.p.i(:,n+1)') = eye(ncons);
        Aeq(indices, obj.vars.p.i(:,n)') = -eye(ncons);
        Aeq(indices, obj.vars.pd.i(:,n:n+1)) = -h/2*[eye(ncons), eye(ncons)];
        offset = offset + ncons;
      end
      obj = obj.addLinearConstraints([], [], Aeq, beq);
    end
    
    function obj = addForceConstraints(obj)
      ncons = (2*obj.dim + 2)*obj.n_regions*obj.N + (2 + 2)*obj.n_basis_vectors*(4*obj.M)*obj.N;
      ncons_eq = (1 + 2*obj.n_basis_vectors)*obj.N;
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
          % Rj --> T - sum_i(Fij(2)*ci(1) - Fij(1)*ci(2)) = 0
          % formulated as
          %    T - sum_i(Fij(2)*ci(1) - Fij(1)*ci(2)) <= (1 - Rj)*big_M
          %   -T + sum_i(Fij(2)*ci(1) - Fij(1)*ci(2)) <= (1 - Rj)*big_M
          ncons = 2;
          indices = offset + (1:ncons);
          big_M = obj.moment_max*(1 + obj.n_basis_vectors);
          A(indices, obj.vars.T.i(:, n)) = [1; -1];
          A(indices, obj.vars.c.i(1, n, :)) = -[obj.regions(j).basis_vectors(2,:); -obj.regions(j).basis_vectors(2,:)];
          A(indices, obj.vars.c.i(2, n, :)) =  [obj.regions(j).basis_vectors(1,:); -obj.regions(j).basis_vectors(1,:)];
          A(indices, obj.vars.R.i(j,n_plus_1_or_N)) = big_M*ones(ncons,1);
          b(indices) = big_M*ones(ncons,1);
          offset = offset + ncons;
          % END_ASSUME
        end
        
        for i = 1:obj.n_basis_vectors
          for m = 1:2*obj.M
            %ci(k) = bi*p(k)
            % approximated by
            %  (1) Bim --> b^(m-1) <= bi <= b^m for i = 1, ..., obj.n_basis_vectors
            %   formulated as
            %     -bi <= -b^(m-1) + (1 - Bim)*big_M
            %     bi <= b^m + (1 - Bim)*big_M
            % and
            %   (2) Bim --> b^(m-1)*p(k) <= ci(k) <= b^m*p(k) for i = 1, ..., obj.n_basis_vectors, k = 1, ..., obj.dim
            %   formulated as
            %      b^(m-1)*p(k) - ci(k) <= (1 - Bim)*big_M
            %     -b^m*p(k) + ci(k) <= (1 - Bim)*big_M
            
            % (1)
            ncons = 2;
            indices = offset + (1:ncons);
            big_M = 2*obj.force_max;
            A(indices, obj.vars.b.i(i, n)) = [-1; 1];
            A(indices, obj.vars.B.i(m, n, i)) = big_M*ones(ncons, 1);
            b(indices) = [-obj.b_constant(m); obj.b_constant(m+2)] + big_M;
            offset = offset + ncons;

            ncons = 2;
            indices = offset + (1:ncons);
            big_M = 2*obj.force_max;
            A(indices, obj.vars.b.i(i, n)) = [-1; 1];
            A(indices, obj.vars.Bz.i(m, n, i)) = big_M*ones(ncons, 1);
            b(indices) = [-obj.b_constant(m); obj.b_constant(m+2)] + big_M;
            offset = offset + ncons;
            
            % (2)
            ncons = 2;
            indices = offset + (1:ncons);
            big_M = 2*obj.moment_max;
            if mod(m, 2) == 1
              A(indices, obj.vars.p.i(1,n)) = [obj.b_constant(m)*eye(1); ...
                                               -obj.b_constant(m+2)*eye(1)];
            else
              A(indices, obj.vars.p.i(1,n)) = [obj.b_constant(m+2)*eye(1); ...
                                               -obj.b_constant(m)*eye(1)];
            end
            %b_mid = mean(obj.b_constant(m:m+1));
            %A(indices, obj.vars.p.i(:,n)) = [b_mid*eye(1); ...
                                            %-b_mid*eye(1)];
            A(indices, obj.vars.c.i(1, n, i)) = [-eye(1); eye(1)];
            A(indices, obj.vars.B.i(m, n, i)) = big_M*ones(ncons, 1);
            b(indices) = big_M;
            offset = offset + ncons;

            ncons = 2;
            indices = offset + (1:ncons);
            big_M = 2*obj.moment_max;
            if mod(m, 2) == 1
              A(indices, obj.vars.p.i(2,n)) = [obj.b_constant(m)*eye(1); ...
                                               -obj.b_constant(m+2)*eye(1)];
            else
              A(indices, obj.vars.p.i(2,n)) = [obj.b_constant(m+2)*eye(1); ...
                                               -obj.b_constant(m)*eye(1)];
            end
            %b_mid = mean(obj.b_constant(m:m+1));
            %A(indices, obj.vars.p.i(:,n)) = [b_mid*eye(1); ...
                                            %-b_mid*eye(1)];
            A(indices, obj.vars.c.i(2, n, i)) = [-eye(1); eye(1)];
            A(indices, obj.vars.Bz.i(m, n, i)) = big_M*ones(ncons, 1);
            b(indices) = big_M;
            offset = offset + ncons;
          end
          
          % sum_m(Bin) = 1
          ncons = 1;
          indices = offset_eq + (1:ncons);
          Aeq(indices, obj.vars.B.i(:,n,i)) = ones(1, 2*obj.M);
          beq(indices) = 1;
          offset_eq = offset_eq + ncons;

          ncons = 1;
          indices = offset_eq + (1:ncons);
          Aeq(indices, obj.vars.Bz.i(:,n,i)) = ones(1, 2*obj.M);
          beq(indices) = 1;
          offset_eq = offset_eq + ncons;
        end
        
        % sum_j(Rj) = 1
        Aeq(offset_eq + 1, obj.vars.R.i(:,n)) = ones(1, obj.n_regions);
        beq(offset_eq + 1) = 1;
        offset_eq = offset_eq + 1;
      end
      obj = obj.addLinearConstraints(A, b, Aeq, beq);
    end

    function obj = addBinaryVariableConstraints(obj)
      ncons = obj.n_basis_vectors*factorial(obj.M-3)*(obj.N-1);
      A = zeros(ncons, obj.nv);
      b = zeros(ncons, 1);
      offset = 0;
      for n = 1:obj.N-1
        for i = 1:obj.n_basis_vectors
          for m = 1:obj.M - 3
            % B(m, n, i) + B(j, n+1, i) <= 1 for j = m + 2, ..., obj.M
            ncons = obj.M - m - 2;
            indices = offset + (1:ncons);
            A(indices, obj.vars.B.i(m, n, i)) = ones(ncons, 1);
            A(indices, obj.vars.B.i(m+3:obj.M, n+1, i)) = eye(ncons);
            b(indices) = 1;
            offset = offset + ncons;
          end
        end
      end
      obj = obj.addLinearConstraints(A, b, [], []);
    end
    
    function obj = addContactPointConstraints(obj)
      big_M = obj.position_max;
      
      n_free_regions = sum(cellfun(@isempty, {obj.regions.normal}));
      n_contact_regions = numel(obj.regions) - n_free_regions;
      ncons = sum([obj.regions.ncons]) + obj.dim*n_contact_regions*obj.N;
      A = zeros(ncons, obj.nv);
      b = zeros(ncons, 1);
      offset = 0;
      for n = 1:obj.N
        for j = 1:numel(obj.regions)
          if ~isempty(obj.regions(j).A)
            % R(j,n) -> A*(r+p) <= b
            % formulated as
            % A*r + A*p <= b + big_M*(1 - R(j,n))
            ncons = size(obj.regions(j).A, 1);
            indices = offset + (1:ncons);
            A(indices, obj.vars.r.i(:,n)) = obj.regions(j).A;
            A(indices, obj.vars.p.i(:,n)) = obj.regions(j).A;
            A(indices, obj.vars.R.i(j,n)) = big_M*ones(ncons,1);
            b(indices) = obj.regions(j).b + big_M;
            offset = offset + ncons;
          end
          if ~isempty(obj.regions(j).Aeq)
            % R(j,n) -> Aeq*(r+p) == beq
            % formulated as
            % A*p + A*r <= b + big_M*(1 - R(j,n))
            % -A*p - A*r <= -b + big_M*(1 - R(j,n))
            ncons = 2*size(obj.regions(j).Aeq, 1);
            indices = offset + (1:ncons);
            A(indices, obj.vars.r.i(:,n)) = [obj.regions(j).Aeq; -obj.regions(j).Aeq];
            A(indices, obj.vars.p.i(:,n)) = [obj.regions(j).Aeq; -obj.regions(j).Aeq];
            A(indices, obj.vars.R.i(j,n)) = big_M*ones(ncons,1);
            b(indices) = [obj.regions(j).beq; -obj.regions(j).beq] + big_M;
            offset = offset + ncons;
          end
          if ~isempty(obj.regions(j).normal)
            % R(j,n) --> v + pd == 0
            % formulated as
            %   v + pd <= big_M*(1 - R(j,n))
            %  -v - pd <= big_M*(1 - R(j,n))
            big_M = 2*obj.velocity_max;
            ncons = 2*obj.dim;
            indices = offset + (1:ncons);
            A(indices, obj.vars.v.i(:,n)) = [eye(obj.dim); -eye(obj.dim)];
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
      if ~isempty(normal)
        obj.regions(j).normal = normal;
        % HACK
        obj.regions(j).basis_vectors = [normalizeVec([1;1]), normalizeVec([-1;1])];
%         obj.regions(j).basis_vectors = normal;
        % END_HACK
      else
        obj.regions(j).basis_vectors = zeros(obj.dim, obj.n_basis_vectors);
      end
      if ~isempty(mu)
        obj.regions(j).mu = mu;
      end
      obj.regions(j).ncons = size(obj.regions(j).A, 1) ...
                            + 2*size(obj.regions(j).Aeq,1);
      obj.n_regions = j;
    end
  end
end
