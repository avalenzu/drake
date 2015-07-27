classdef LeggedLocomotionPlanner < MixedIntegerConvexProgram
  properties
    I
    m
    N = 2
    position_max = 1e2
    velocity_max = 1e1
    force_max = 1e3
    foot_position_max = 2e0
    foot_velocity_max = 1e1
    slack_max = 1e5;
    foot_mass = 1;
    dt = 1e-3
    w_fixed_array
    z_fixed_array
    F_fixed_array
    M_fixed_array
    regions = struct('A', {}, 'b', {}, 'Aeq', {}, 'beq', {}, 'normal', {}, 'mu', {}, 'ncons', {});
    feet
    friction_cone_normals = [0.5, -0.25,  -0.25; ...
                                 0,    0.433, -0.433; ...
                                 0.5,  0.5,    0.5];
   
    fix_forces = false;
                              
  end

  methods
    function obj = LeggedLocomotionPlanner(I, m, N, dt, z_fixed_array, w_fixed_array, F_fixed_array, M_fixed_array, has_symbolic)
      obj = obj@MixedIntegerConvexProgram(has_symbolic);
      obj.N = N;
      obj.dt = dt;
      obj.I = I;
      obj.m = m;
      obj.w_fixed_array = w_fixed_array;
      obj.z_fixed_array = z_fixed_array;
      obj.F_fixed_array = F_fixed_array;
      obj.M_fixed_array = M_fixed_array;
    end
    
    function obj = addDefaultConstraintsAndCosts(obj)
      
      obj = obj.addPositionVariables();
      obj = obj.addVelocityVariables();
      obj = obj.addVariable('slack', 'C', [3, obj.N], -obj.slack_max, obj.slack_max);
      for i = 1:numel(obj.feet)
        % Add foot position & velocity variables
        obj = obj.addVariable(sprintf('r_foot%d',i), 'C', [3, obj.N], -obj.foot_position_max, obj.foot_position_max);
        obj = obj.addVariable(sprintf('v_foot%d',i), 'C', [3, obj.N], -obj.foot_velocity_max, obj.foot_velocity_max);
        obj = obj.addVariable(sprintf('F_foot%d',i), 'C', [3, obj.N], -obj.force_max, obj.force_max);
        
        % Add binary variables for contact region assignment
        obj = obj.addVariable(sprintf('R%d',i), 'B', [numel(obj.regions), obj.N], 0, 1);
        obj = obj.addVariable(sprintf('RR%d',i), 'B', [numel(obj.regions), obj.N-1], 0, 1);
        
        % Add continuous variables for contact forces and moments
        obj = obj.addVariable(sprintf('F%d',i), 'C', [3, obj.N], ...
          -obj.force_max, obj.force_max);
        
        obj = obj.addVariable(sprintf('M%d',i), 'C', [3, obj.N], ...
          -obj.force_max, obj.force_max);
      end
      obj = obj.addTranslationalDynamicConstraints(false);
      obj = obj.addRotationalDynamicConstraints(false);
      obj = obj.addContactPointConstraints(false);
      obj = obj.addContactForceConstraints(false);
      if obj.has_symbolic
        obj = obj.addSymbolicCost(sum((obj.vars.w.symb(:) - obj.w_fixed_array(:)).^2));
        obj = obj.addSymbolicCost(sum((obj.vars.z.symb(:) - obj.z_fixed_array(:)).^2));
        %obj = obj.addSymbolicCost(sum(obj.vars.slack.symb(:).^2));
        %obj = obj.addSymbolicCost(sum(obj.vars.slack.symb(:)));
        %obj = obj.addSymbolicCost(obj.vars.contact_point_slack.symb);
        for i = 1:numel(obj.feet)
          obj = obj.addSymbolicCost(sum(sum((obj.vars.(sprintf('F%d',i)).symb - obj.F_fixed_array(:,:,i)).^2)));
          obj = obj.addSymbolicCost(sum(sum((obj.vars.(sprintf('M%d',i)).symb - obj.M_fixed_array(:,:,i)).^2)));
          obj = obj.addSymbolicCost(sum(sum((obj.vars.(sprintf('r_foot%d',i)).symb - obj.feet(i).r_fixed).^2)));
          %obj = obj.addSymbolicCost(sum(sum(obj.vars.(sprintf('v_foot%d',i)).symb.^2)));
          %obj = obj.addSymbolicCost(1e3*sum(sum(obj.vars.(sprintf('F%d',i)).symb.^2)));
          %obj = obj.addSymbolicCost(1e3*sum(sum(obj.vars.(sprintf('M%d',i)).symb.^2)));
        end
      else
        Q = zeros(obj.nv);
        c = zeros(obj.nv, 1);
        Q(obj.vars.w.i(:), obj.vars.w.i(:)) = eye(numel(obj.w_fixed_array));
        c(obj.vars.w.i(:)) = -2*obj.w_fixed_array(:);
        Q(obj.vars.z.i(:), obj.vars.z.i(:)) = eye(numel(obj.z_fixed_array));
        c(obj.vars.z.i(:)) = -2*obj.z_fixed_array(:);
        alpha = 0;
        for i = 1:numel(obj.feet)
          Q(obj.vars.(sprintf('F%d',i)).i(:), obj.vars.(sprintf('F%d',i)).i(:)) = (1 + 1e0)*eye(numel(obj.F_fixed_array(:,:,i)));
          c(obj.vars.(sprintf('F%d',i)).i(:)) = -2*obj.F_fixed_array(:,:,i);
          Q(obj.vars.(sprintf('M%d',i)).i(:), obj.vars.(sprintf('M%d',i)).i(:)) = (1 + 1e0)*eye(numel(obj.M_fixed_array(:,:,i)));
          c(obj.vars.(sprintf('M%d',i)).i(:)) = -2*obj.M_fixed_array(:,:,i);
          Q(obj.vars.(sprintf('r_foot%d',i)).i(:), obj.vars.(sprintf('r_foot%d',i)).i(:)) = eye(numel(obj.feet(i).r_fixed));
          c(obj.vars.(sprintf('r_foot%d',i)).i(:)) = -2*obj.feet(i).r_fixed(:);
          alpha = alpha + sum(obj.feet(i).r_fixed(:).^2);
          Q(obj.vars.(sprintf('v_foot%d',i)).i(:), obj.vars.(sprintf('v_foot%d',i)).i(:)) = eye(numel(obj.feet(i).r_fixed));
          R_indices = obj.vars.(sprintf('R%d',i)).i(:,1:end-1); 
          R_plus_indices = obj.vars.(sprintf('R%d',i)).i(:,2:end); 
          Q(R_indices, R_indices) = eye(numel(R_indices));
          Q(R_plus_indices, R_plus_indices) = eye(numel(R_indices));
          Q(R_indices, R_plus_indices) = -eye(numel(R_indices));
          Q(R_plus_indices, R_indices) = -eye(numel(R_indices));
        end
        alpha = alpha ...
                + sum(obj.w_fixed_array(:).^2) ...
                + sum(obj.z_fixed_array(:).^2) ...
                + sum(obj.F_fixed_array(:).^2) ...
                + sum(obj.M_fixed_array(:).^2);
        %alpha = 0*alpha;
        obj = obj.addCost(Q, c, alpha);
      end
    end

    function obj = addTranslationalDynamicConstraints(obj, use_symbolic)
      if nargin < 2, use_symbolic = true; end
      h = obj.dt;
      I = obj.I; %#ok
      m = obj.m; %#ok
      g = [0; 0; -9.81];
      if obj.has_symbolic && use_symbolic
        for n = 1:(obj.N-1)
          r = obj.vars.r.symb(:, n);
          v = obj.vars.v.symb(:, n);

          r_next = obj.vars.r.symb(:, n+1);
          v_next = obj.vars.v.symb(:, n+1);

          F = m*g; %#ok
          F_next = m*g; %#ok
          for i = 1:numel(obj.feet)
            Fname = sprintf('F%d', i);
            rname = sprintf('r_foot%d', i);
            vname = sprintf('v_foot%d', i);
            F = F + obj.vars.(Fname).symb(:, n);
            F_next = F_next + obj.vars.(Fname).symb(:, n+1);
            r_foot = obj.vars.(rname).symb(:, n);
            r_foot_next = obj.vars.(rname).symb(:, n+1);
            v_foot = obj.vars.(vname).symb(:, n);
            v_foot_next = obj.vars.(vname).symb(:, n+1);
            v_foot_mid = 0.5*(v_foot + v_foot_next);
            r_foot_next_desired = r_foot + h*v_foot_mid;
            obj = obj.addSymbolicConstraints(r_foot_next == r_foot_next_desired);
          end
          
          v_mid = v + h/(2*m)*F; %#ok
          r_next_desired = r + h*v_mid;
          v_next_desired = v_mid + h/(2*m)*F_next; %#ok
          obj = obj.addSymbolicConstraints(r_next == r_next_desired);
          obj = obj.addSymbolicConstraints(v_next == v_next_desired);
%           obj = obj.addSymbolicCost(norm(r_next - r_next_desired));
%           obj = obj.addSymbolicCost(norm(v_next - v_next_desired));
        end
      else
        offset = 0;
        Aeq = zeros((6+6*numel(obj.feet))*(obj.N-1), obj.nv);
        beq = zeros((6+6*numel(obj.feet))*(obj.N-1), 1);
        for n = 1:(obj.N-1)

          % Position
          % r_next - r - h*v = h^2/(2*m)*sum(Fi) + h^2*g/2
          beq(1:3) = (h^2/2)*g;
          % Executed in velocity section below
          %for i = 1:numel(obj.feet)
            %Aeq(1:3, obj.vars.(sprintf('F%d',i)).i(:,n)) = -h^2/(2*m)*eye(3); %#ok
          %end
          Aeq(sub2ind(size(Aeq), offset+(1:3), obj.vars.r.i(:,n+1)')) = 1;
          Aeq(sub2ind(size(Aeq), offset+(1:3), obj.vars.r.i(:,n)')) = -1;
          Aeq(sub2ind(size(Aeq), offset+(1:3), obj.vars.v.i(:,n)')) = -h;

          % Velocity
          % v_next - v = h/(2*m)*sum(Fi+Fi_next) + h*g
          Aeq(sub2ind(size(Aeq), offset+(4:6), obj.vars.v.i(:,n+1)')) = 1;
          Aeq(sub2ind(size(Aeq), offset+(4:6), obj.vars.v.i(:,n)')) = -1;
          R_body_to_world = quat2rotmat(obj.z_fixed_array(:,n));
          R_body_to_world_next = quat2rotmat(obj.z_fixed_array(:,n+1));
          for i = 1:numel(obj.feet)
            Aeq(offset+(1:3), obj.vars.(sprintf('F%d',i)).i(:,n)) = -h^2/(2*m)*eye(3); %#ok
            Aeq(offset+(4:6), obj.vars.(sprintf('F%d',i)).i(:,n:n+1)) = -h/(2*m)*[eye(3), eye(3)]; %#ok

            % Foot position
            % R(z_f)*r_foot_next + r_next - R(z_f)*r_foot  - r - h*v_foot_mid == 0
            % where
            % v_foot_mid = v_foot + h/(2*m_foot)*(F_ext_i - Fi) + h*g/2
            % Fi is force exerted on body by foot
            % Thus we have
            % R(z_f_next)*r_foot_next + r_next - R(z_f)*r_foot - r - h*v_foot - h^2/(2*m_foot)*(F_ext_i - Fi) + h^2*g/2
            rname = sprintf('r_foot%d', i);
            vname = sprintf('v_foot%d', i);
            Fname = sprintf('F%d', i);
            Fextname = sprintf('F_foot%d', i);
            Aeq(offset+6+(i-1)*3+(1:3), obj.vars.r.i(:,n:n+1)) = [-eye(3), eye(3)];
            Aeq(offset+6+(i-1)*3+(1:3), obj.vars.(rname).i(:,n:n+1)) = [-R_body_to_world, R_body_to_world_next];
            Aeq(offset+6+(i-1)*3+(1:3), obj.vars.(vname).i(:,n)) = -h*eye(3);
            Aeq(offset+6+(i-1)*3+(1:3), obj.vars.(Fname).i(:,n)) = h/(2*obj.feet(i).m)*eye(3);
            Aeq(offset+6+(i-1)*3+(1:3), obj.vars.(Fextname).i(:,n)) = -h/(2*obj.feet(i).m)*eye(3);

            % Foot velocity
            % v_foot_next - v_foot_mid - h/(2*m)*(Fext_i_next - Fi_next) + h*g/2  == 0
            % Thus we have
            % v_foot_next - v_foot - h/(2*m_foot)*(Fext_i + Fext_i_next - Fi - Fi_next) + h*g
            Aeq(offset+6+(i-1)*3+(4:6), obj.vars.(vname).i(:,n:n+1)) = [-eye(3), eye(3)];
            Aeq(offset+6+(i-1)*3+(4:6), obj.vars.(Fname).i(:,n:n+1)) = h/(2*obj.feet(i).m)*[eye(3), eye(3)];
            Aeq(offset+6+(i-1)*3+(4:6), obj.vars.(Fextname).i(:,n:n+1)) = -h/(2*obj.feet(i).m)*[eye(3), eye(3)];
            beq(offset+6+(i-1)*3+(4:6)) = h*g;
          end
          beq(offset+(4:6)) = h*g;
          beq(offset+(7:9)) = h^2*g/2;
          offset = offset + 6 + 6*numel(obj.feet);
        end
        obj = obj.addLinearConstraints([], [], Aeq, beq);
      end
    end

    function obj = addRotationalDynamicConstraints(obj, use_symbolic)
      if nargin < 2, use_symbolic = true; end
      h = obj.dt;
      I = obj.I; %#ok
      m = obj.m; %#ok
      g = [0; 0; -9.81];
      if obj.has_symbolic && use_symbolic
        for n = 1:(obj.N-1)
          z = obj.vars.z.symb(:, n);
          w = obj.vars.w.symb(:, n);

          z_next = obj.vars.z.symb(:, n+1);
          w_next = obj.vars.w.symb(:, n+1);
          Y_next = I*w_next; %#ok
          
          z_fixed = obj.z_fixed_array(:, n);
          z_next_fixed = obj.z_fixed_array(:, n+1);
          w_fixed = obj.w_fixed_array(:, n);
          w_next_fixed = obj.w_fixed_array(:, n+1);
          w_mid_fixed = 0.5*(w_fixed + w_next_fixed);
          A = expmap2quat(-h/2*w_mid_fixed);
          B = expmap2quat(h/2*w_mid_fixed);
          T = zeros(3, 1);
          T_next = zeros(3, 1);
          for i = 1:numel(obj.feet)
            M_fixed = obj.M_fixed_array(:, n, i);
            F_fixed = obj.F_fixed_array(:, n, i);
            M_next_fixed = obj.M_fixed_array(:, n, i);
            F_next_fixed = obj.F_fixed_array(:, n, i);
            rname = sprintf('r_foot%d', i);
            r_foot = obj.vars.(rname).symb(:, n);
            r_foot_next = obj.vars.(rname).symb(:, n);
            r_foot_fixed = obj.feet(i).r_fixed(:, n);
            r_foot_next_fixed = obj.feet(i).r_fixed(:, n+1);
            Fname = sprintf('F%d', i);
            Mname = sprintf('M%d', i);
            %if obj.fix_forces
              %Ti = quatRotateVec(quatConjugate(z_fixed), obj.vars.(Mname).symb(:, n)) ...
                %+ cross(r_foot, quatRotateVec(quatConjugate(z_fixed), F_fixed));
%               Ti_next = quatRotateVec(quatConjugate(z_next_fixed), obj.vars.(Mname).symb(:, n+1)) ...
%                 + cross(r_foot_next, quatRotateVec(quatConjugate(z_next_fixed), F_next_fixed));
            %else
               Ti = quatRotateVec(quatConjugate(z_fixed), obj.vars.(Mname).symb(:, n)) ...
                 + cross(r_foot_fixed, quatRotateVec(quatConjugate(z_fixed), obj.vars.(Fname).symb(:, n)));
              Ti_next = quatRotateVec(quatConjugate(z_next_fixed), obj.vars.(Mname).symb(:, n+1)) ...
                + cross(r_foot_next_fixed, quatRotateVec(quatConjugate(z_next_fixed), obj.vars.(Fname).symb(:, n+1)));
%             end
            T = T + Ti;
            T_next = T_next + Ti_next;
          end
          Y_next_desired = quatRotateVec(A, ...
            quatRotateVec(A, I*w + h/2*T) ...
            + h/2*quatRotateVec(B, T_next)); %#ok
          z_next_desired = quatProduct(z, expmap2quat(h*w_mid_fixed));
          obj = obj.addSymbolicConstraints(z_next == z_next_desired);
          %obj = obj.addSymbolicCost(sum((z_next - z_next_desired).^2));
          obj = obj.addSymbolicConstraints(Y_next == Y_next_desired);
          %slack = obj.vars.slack.symb(n);
          %obj = obj.addSymbolicConstraints(Y_next_desired - slack <= Y_next <= Y_next_desired);
          %obj = obj.addSymbolicCost(sum((Y_next - Y_next_desired).^2));
        end
      else
        Aeq = zeros(7*obj.N-1, obj.nv);
        beq = zeros(7*obj.N-1, 1);

        offset = 0;
        for n = 1:(obj.N-1)
          T = zeros(3, 1);
          T_next = zeros(3, 1);
          z_fixed = obj.z_fixed_array(:, n);
          z_next_fixed = obj.z_fixed_array(:, n+1);
          w_fixed = obj.w_fixed_array(:, n);
          w_next_fixed = obj.w_fixed_array(:, n+1);
          w_mid_fixed = 0.5*(w_fixed + w_next_fixed);

          % quatProduct(z, expmap(h*w_mid_fixed)) - z_next= 0
          deltaZ = expmap2quat(h*w_mid_fixed);
          Aeq(offset+(1:4), obj.vars.z.i(:,n+1)) = -eye(4);
          Aeq(offset+1, obj.vars.z.i(1,n)) = deltaZ(1);
          Aeq(offset+1, obj.vars.z.i(2:4,n)) = -deltaZ(2:4);
          Aeq(offset+2, obj.vars.z.i(1:3,n)) = deltaZ([2,1,4]);
          Aeq(offset+2, obj.vars.z.i(4,n)) = -deltaZ(3);
          Aeq(offset+3, obj.vars.z.i([1,3,4],n)) = deltaZ([3,1,2]);
          Aeq(offset+3, obj.vars.z.i(2,n)) = -deltaZ(4);
          Aeq(offset+4, obj.vars.z.i([1,2,4],n)) = deltaZ([4,3,1]);
          Aeq(offset+4, obj.vars.z.i(3,n)) = -deltaZ(2);

          % R(A)'*I*w_next - R(A)*I*w - h/2*(R(A)*T + R(B)*T_next == 0)
          % where
          %   T = sum( R(z_f)'*Mi + pt_i x R(z_f)'*Fi )
          RA = quat2rotmat(expmap2quat(-h/2*w_mid_fixed));
          RB = quat2rotmat(expmap2quat(h/2*w_mid_fixed));
          R_world_to_body = quat2rotmat(z_fixed)';
          R_world_to_body_next = quat2rotmat(z_next_fixed)';
          for i = 1:numel(obj.feet)
            pt_cross = vectorToSkewSymmetric(obj.feet(i).r_fixed(:,n));
            pt_cross_next = vectorToSkewSymmetric(obj.feet(i).r_fixed(:,n+1));
            Aeq(offset + (5:7), obj.vars.(sprintf('M%d',i)).i(:,n)) = -h/2*RA*R_world_to_body;
            Aeq(offset + (5:7), obj.vars.(sprintf('F%d',i)).i(:,n)) = -h/2*RA*pt_cross*R_world_to_body;
            Aeq(offset + (5:7), obj.vars.(sprintf('M%d',i)).i(:,n+1)) = -h/2*RB*R_world_to_body_next;
            Aeq(offset + (5:7), obj.vars.(sprintf('F%d',i)).i(:,n+1)) = -h/2*RB*pt_cross_next*R_world_to_body_next;
          end
          Aeq(offset + (5:7), obj.vars.w.i(:,n+1)) = RA'*I; %#ok
          Aeq(offset + (5:7), obj.vars.w.i(:,n)) = -RA*I; %#ok
          Aeq(offset + (5:7), obj.vars.slack.i(:,n)) = eye(3);
          %beq(offset + (5:7)) = h/2*(RA*T + RB*T_next);
          offset = offset + 7;
        end
        obj = obj.addLinearConstraints([], [], Aeq, beq);
      end
    end

    function obj = addContactPointConstraints(obj, use_symbolic)
      z_fixed = obj.z_fixed_array;
      if use_symbolic
        r = obj.vars.r.symb;
        z = obj.vars.z.symb;
        for i = 1:numel(obj.feet)
          rname = sprintf('r_foot%d', i);
          R = obj.vars.(sprintf('R%d',i)).symb;
          obj = obj.addSymbolicConstraints(sum(R,1) == 1);
          for n = 1:obj.N
            r_foot = obj.vars.(rname).symb(:, n);
            %p = obj.quatRotateVec2(z(:,n), z_fixed(:,n), obj.feet(i).r_fixed(:,n)) + r(:,n);
            p = quatRotateVec(z_fixed(:,n), r_foot) + r(:,n);
            %p = 0.5*(p + quatRotateVec(z_fixed(:,n), r_foot) + r(:,n));
            for j = 1:numel(obj.regions)
              if ~isempty(obj.regions(j).A)
                 %obj = obj.addSymbolicConstraints(implies(R(j,n), ...
                   %obj.regions(j).A*p <= obj.regions(j).b));
                big_M = sqrt(3)*obj.position_max + max(abs(obj.regions(j).b));
                obj = obj.addSymbolicConstraints(binary_implies_linearnegativeconstraint(obj.regions(j).A*p - obj.regions(j).b, R(j,n), big_M));
              end
              if ~isempty(obj.regions(j).Aeq)
                 %obj = obj.addSymbolicConstraints(implies(R(j,n), ...
                   %obj.regions(j).Aeq*p == obj.regions(j).beq));
                  big_M = sqrt(3)*obj.position_max + max(abs(obj.regions(j).beq));
                  obj = obj.addSymbolicConstraints(binary_implies_linearnegativeconstraint(obj.regions(j).Aeq*p - obj.regions(j).beq,R(j,n),big_M));
                  obj = obj.addSymbolicConstraints(binary_implies_linearnegativeconstraint(-obj.regions(j).Aeq*p + obj.regions(j).beq,R(j,n),big_M));
              end
            end
            for j = 1:numel(obj.feet(i).constraints)
              center = obj.feet(i).constraints(j).center;
              radius = obj.feet(i).constraints(j).radius;
              obj = obj.addSymbolicConstraints(sum((r_foot - center).^2) <= radius.^2);
            end
          end
        end
      else
        
        n_contacts = numel(obj.feet);
        big_M = obj.position_max;

        Aeq = zeros(n_contacts*obj.N, obj.nv);
        beq = zeros(n_contacts*obj.N, 1);
        A = zeros(n_contacts*sum([obj.regions.ncons]), obj.nv);
        b = zeros(n_contacts*sum([obj.regions.ncons]), 1);
        ncons_quad_total = sum([obj.feet.ncons]);
        quadcon = struct('Qc', repmat({sparse(obj.nv, obj.nv)}, 1, ncons_quad_total), 'q', repmat({zeros(obj.nv, 1)}, 1, ncons_quad_total), 'rhs', repmat({0}, 1, ncons_quad_total));
        offset = 0;
        offset_eq = 0;
        offset_quad = 0;
        for i = 1:numel(obj.feet)
          rname = sprintf('r_foot%d', i);
          for n = 1:obj.N
            % sum(R) == 1
            Aeq(offset_eq+1, obj.vars.(sprintf('R%d',i)).i(:,n)) = 1;
            beq(offset_eq+1) = 1;
            offset_eq = offset_eq + 1;
            R_body_to_world = quat2rotmat(z_fixed(:,n));
            for j = 1:numel(obj.regions)
              if ~isempty(obj.regions(j).A)
                % R(j,n) -> A*p <= b
                % formulated as
                % A*p <= b + big_M*(1 - R(j,n))
                ncons = size(obj.regions(j).A, 1);
                indices = offset + (1:ncons);
                A(indices, obj.vars.r.i(:,n)) = obj.regions(j).A;
                A(indices, obj.vars.(sprintf('R%d',i)).i(j,n)) = big_M*ones(ncons,1);
                A(indices, obj.vars.(sprintf('r_foot%d',i)).i(:,n)) = obj.regions(j).A*R_body_to_world;
                %A(indices, obj.vars.contact_point_slack.i) = -1*ones(ncons,1);
                b(indices) = obj.regions(j).b + big_M;
                offset = offset + ncons;
              end
              if ~isempty(obj.regions(j).Aeq)
                % R(j,n) -> Aeq*p == beq
                % formulated as
                % A*p <= b + big_M*(1 - R(j,n))
                % -A*p <= -b + big_M*(1 - R(j,n))
                ncons = 2*size(obj.regions(j).Aeq, 1);
                indices = offset + (1:ncons);
                A(indices, obj.vars.r.i(:,n)) = [obj.regions(j).Aeq; -obj.regions(j).Aeq];
                A(indices, obj.vars.(sprintf('r_foot%d',i)).i(:,n)) = [obj.regions(j).Aeq; -obj.regions(j).Aeq]*R_body_to_world;
                A(indices, obj.vars.(sprintf('R%d',i)).i(j,n)) = big_M*ones(ncons,1);
                b(indices) = [obj.regions(j).beq; -obj.regions(j).beq] + big_M;
                offset = offset + ncons;
              end
            end
            ncons = obj.feet(i).ncons;
            for j = 1:ncons
              index = offset_quad + j;
              center = obj.feet(i).constraints(j).center;
              radius = obj.feet(i).constraints(j).radius;
              quadcon(index).Qc = sparse(obj.vars.(rname).i(:,n), obj.vars.(rname).i(:,n), ones(3,1), obj.nv, obj.nv);
              quadcon(index).q = full(sparse(obj.vars.(rname).i(:,n), ones(3,1), -2*center, obj.nv, 1));
              quadcon(index).rhs = radius^2 - center'*center;
            end
            offset_quad = offset_quad + ncons;
          end
        end
        obj = obj.addLinearConstraints(A, b, Aeq, beq);
        obj = obj.addQuadcon(quadcon);
      end
    end
    
    function obj = addContactForceConstraints(obj, use_symbolic)
      if use_symbolic
        r = obj.vars.r.symb;
        z = obj.vars.z.symb;            
        v = obj.vars.v.symb;            
        w = obj.vars.w.symb;

        z_fixed = obj.z_fixed_array;
        w_fixed = obj.w_fixed_array;
        for i = 1:numel(obj.feet)
          R = obj.vars.(sprintf('R%d',i)).symb;
          RR = obj.vars.(sprintf('RR%d',i)).symb;
          for n = 1:obj.N
            %p = obj.quatRotateVec2(z(:,n), z_fixed(:,n), obj.contact_pts(:,i)) + r(:,n);
            %p_next = obj.quatRotateVec2(z(:,n+1), z_fixed(:,n+1), obj.contact_pts(:,i)) + r(:,n+1);
            p = quatRotateVec(z_fixed(:,n), obj.vars.(sprintf('r_foot%d',i)).symb(:,n)) + r(:,n);
            if n < obj.N
              p_next = quatRotateVec(z_fixed(:,n+1), obj.vars.(sprintf('r_foot%d',i)).symb(:,n+1)) + r(:,n+1);
            end
            %pd = v(:,n) + cross(w(:,n), obj.feet(i).r_fixed(:,n));
            pd = v(:,n) + cross(w_fixed(:,n), obj.vars.(sprintf('r_foot%d',i)).symb(:,n));
            %pd = 0.5*(pd + v(:,n) + cross(w_fixed(:,n), obj.vars.(sprintf('r_foot%d',i)).symb(:,n)));
            F = obj.vars.(sprintf('F%d', i)).symb(:, n);
            M = obj.vars.(sprintf('M%d', i)).symb(:, n);
            for j = 1:numel(obj.regions)
              normal = obj.regions(j).normal;
              mu = obj.regions(j).mu;
              big_M = sqrt(3)*obj.force_max;
              if isempty(normal)
                 %obj = obj.addSymbolicConstraints(implies(R(j,n), F == 0));
                 %obj = obj.addSymbolicConstraints(implies(R(j,n), M == 0));
                obj = obj.addSymbolicConstraints(implies(R(j,n), ...
                  obj.vars.(sprintf('v_foot%d',i)).symb <= obj.swing_velocity_max));
                obj = obj.addSymbolicConstraints(implies(R(j,n), ...
                  obj.vars.(sprintf('v_foot%d',i)).symb >= -obj.swing_velocity_max));
                obj = obj.addSymbolicConstraints(binary_implies_linearnegativeconstraint(F, R(j,n), big_M));
                obj = obj.addSymbolicConstraints(binary_implies_linearnegativeconstraint(-F, R(j,n), big_M));
                obj = obj.addSymbolicConstraints(binary_implies_linearnegativeconstraint(M, R(j,n), big_M));
                obj = obj.addSymbolicConstraints(binary_implies_linearnegativeconstraint(-M, R(j,n), big_M));
              else
                obj = obj.addSymbolicConstraints(implies(R(j,n), ...
                  obj.vars.(sprintf('v_foot%d',i)).symb <= obj.stance_velocity_max));
                obj = obj.addSymbolicConstraints(implies(R(j,n), ...
                  obj.vars.(sprintf('v_foot%d',i)).symb >= -obj.stance_velocity_max));
                F_normal = dot(F, normal)*normal;
                F_tan = F - F_normal;
                pd_normal = dot(pd, normal)*normal;
                pd_tan = pd - pd_normal;
                % HACK
                %obj = obj.addSymbolicConstraints(cone(F_tan, mu*dot(F, obj.regions(j).normal)));
                 %obj = obj.addSymbolicConstraints(implies(R(j,n), ...
                                                  %-obj.friction_cone_normals'*F <= 0));
                obj = obj.addSymbolicConstraints(binary_implies_linearnegativeconstraint(-obj.friction_cone_normals'*F, R(j,n), big_M));
                 %obj = obj.addSymbolicConstraints(implies(R(j,n), M == 0));
                obj = obj.addSymbolicConstraints(binary_implies_linearnegativeconstraint(M, R(j,n), big_M));
                obj = obj.addSymbolicConstraints(binary_implies_linearnegativeconstraint(-M, R(j,n), big_M));
                % END_HACK
%                 obj = obj.addSymbolicConstraints(implies(R(j,n) + R(j,n+1) == 2, pd == 0));
                if n < obj.N
                  big_M = 5;
                  obj = obj.addSymbolicConstraints(R(j,n) + R(j,n+1) <= RR(j,n)+1);
                  %obj = obj.addSymbolicConstraints(RR(j,n) + R(j,n+1) <= R(j,n)+1);
                  %obj = obj.addSymbolicConstraints(RR(j,n) + R(j,n) <= R(j,n+1)+1);
                  %obj = obj.addSymbolicConstraints(RR(j,n) <= R(j,n));
                  %obj = obj.addSymbolicConstraints(RR(j,n) <= R(j,n+1));
                  %obj = obj.addSymbolicConstraints(big_M*RR(j,n) + R(j,n) + R(j,n+1) - 2 <= big_M);
                  %obj = obj.addSymbolicConstraints(big_M*RR(j,n) - R(j,n) - R(j,n+1) + 2 <= big_M);
                  big_M = 2*(obj.position_max+obj.foot_position_max);
                  %obj = obj.addSymbolicConstraints(implies(R(j,n) + R(j,n+1) == 2, p - p_next == 0));
                  obj = obj.addSymbolicConstraints(big_M*RR(j,n) + (p - p_next) <= big_M);
                  obj = obj.addSymbolicConstraints(big_M*RR(j,n) - (p - p_next) <= big_M);
                  %obj = obj.addSymbolicConstraints(binary_implies_linearnegativeconstraint(p - p_next, R(j,n)+R(j,n+1) == 2, big_M));
                  %obj = obj.addSymbolicConstraints(implies(R(j,n), p == p_next));
                end
                  %obj = obj.addSymbolicConstraints(implies(R(j,n), pd_tan == 0));
                %big_M = obj.velocity_max + obj.velocity_max*obj.foot_position_max;
                %obj = obj.addSymbolicConstraints(binary_implies_linearnegativeconstraint(pd_tan, R(j,n), big_M));
                %obj = obj.addSymbolicConstraints(binary_implies_linearnegativeconstraint(-pd_tan, R(j,n), big_M));
              end
            end
          end
        end
      else
        big_M = obj.force_max;
        n_free_regions = sum(cellfun(@isempty, {obj.regions.normal}));
        n_contact_regions = numel(obj.regions) - n_free_regions;
        n_contacts = numel(obj.feet);
        n_fc_faces = size(obj.friction_cone_normals,2);
        %ncons_total = n_contacts*(obj.N*12*n_free_regions + ((obj.N-1)*13 + 6 + obj.N*(n_fc_faces+6))*n_contact_regions);
        ncons_total = n_contacts*(obj.N*12*n_free_regions + ((obj.N-1)*0 + 0 + obj.N*(n_fc_faces+6+6))*n_contact_regions);
        A = zeros(ncons_total, obj.nv);
        b = zeros(ncons_total, 1);
        offset = 0;
        for i = 1:numel(obj.feet)
          for n = 1:obj.N-1
            n_or_n_plus_1 = min(n+1, obj.N);
            pt_cross = vectorToSkewSymmetric(obj.feet(i).r_fixed(:,n));
            R_body_to_world = quat2rotmat(obj.z_fixed_array(:,n));
            for j = 1:numel(obj.regions)
              normal = obj.regions(j).normal;
              mu = obj.regions(j).mu;
              if isempty(normal)
                % R(j,n+1) -> F <= 0 && 0 <= F
                % formulated as
                % F <= big_M*(1 - R(j,n))
                % and
                % -F <= big_M*(1 - R(j,n));
                ncons = 6;
                indices = offset + (1:ncons);
                A(indices, obj.vars.(sprintf('R%d',i)).i(j,n_or_n_plus_1)) = big_M*ones(6,1);
                A(indices, obj.vars.(sprintf('F_foot%d',i)).i(:,n)) = [eye(3); -eye(3)];
                b(indices) = big_M;
                offset = offset + ncons;

                % R(j,n+1) -> M <= 0 && 0 <= M
                % formulated as
                % M <= big_M*(1 - R(j,n))
                % and
                % -M <= big_M*(1 - R(j,n));
                ncons = 6;
                indices = offset + (1:ncons);
                A(indices, obj.vars.(sprintf('R%d',i)).i(j,n_or_n_plus_1)) = big_M*ones(6,1);
                A(indices, obj.vars.(sprintf('M%d',i)).i(:,n)) = [eye(3); -eye(3)];
                %A(indices, obj.vars.contact_point_slack.i) = -1*ones(ncons,1);
                b(indices) = big_M;
                offset = offset + ncons;
              else
                % HACK
                % R(j,n+1) -> -fc_normals'*F <= 0
                % formulated as
                % -fc_normals'*F <= big_M*(1 - R(j, n+1))
                fc_normals = obj.regions(j).friction_cone_normals;
                ncons = size(fc_normals, 2);
                indices = offset + (1:ncons);
                A(indices, obj.vars.(sprintf('R%d',i)).i(j, n_or_n_plus_1)) = big_M*ones(ncons, 1);
                A(indices, obj.vars.(sprintf('F_foot%d',i)).i(:, n)) = -fc_normals';
                b(indices) = big_M*ones(ncons, 1);
                offset = offset + ncons;

                ncons = 6;
                indices = offset + (1:ncons);
                A(indices, obj.vars.(sprintf('R%d',i)).i(j,n_or_n_plus_1)) = big_M*ones(6,1);
                A(indices, obj.vars.(sprintf('M%d',i)).i(:,n)) = [eye(3); -eye(3)];
                b(indices) = big_M;
                offset = offset + ncons;
                % END_HACK
                % R(j,n) -> (dp/dt)_tangential == 0
                % <=>
                % R(j,n) -> (v_foot_i)_tangential == 0
                % <=>
                % R(j,n) -> (v_foot_i) - normal'*(v_foot_i)*normal == 0
                % <=>
                % R(j,n) -> (I3 - normal*normal')*(v_foot_i) == 0
                % formulated as
                %  (I3 - normal*normal')*(v_foot_i) <= big_M*(1 - R(j,n))
                % -(I3 - normal*normal')*(v_foot_i) <= big_M*(1 - R(j,n))
                big_M = obj.velocity_max;
                ncons = 6;
                indices = offset + (1:ncons);
                C = eye(3) - normal*normal';
                A(indices, obj.vars.(sprintf('R%d',i)).i(j,n_or_n_plus_1)) = big_M*ones(6,1);
                A(indices, obj.vars.(sprintf('v_foot%d',i)).i(:,n)) = [C; -C];
                b(indices) = big_M*ones(ncons, 1);
                offset = offset + ncons;


                %big_M = obj.force_max;
                %ncons = 3;
                %n_or_n_plus_1 = min(n+1, obj.N);
                %indices = offset + (1:ncons);
                %A(indices, obj.vars.(sprintf('F%d',i)).i(:,n)) = eye(3);
                %A(indices, obj.vars.(sprintf('F_foot%d',i)).i(:,n)) = -eye(3);
                %A(indices, obj.vars.(sprintf('R%d',i)).i(j,n_or_n_plus_1)) = big_M*ones(3,1);
                %b(indices) = big_M*ones(ncons, 1);
                %offset = offset + ncons;

                % R(j,n) & R(j,n+1) --> p - p_next == 0
                % where 
                % p = r(n) + R_body_to_world(n)*r_foot(n)
                % p_next = r(n+1) + R_body_to_world(n+1)*r_foot(n+1)
                % formulated as
                % R(j,n) + R(j,n+1) <= RR(j,n) + 1
                % r(n) - r(n+1) + R_body_to_world(n)*r_foot(n) - R_body_to_world(n+1)*r_foot(n+1) <= M*(1 - RR(j,n))
                % -r(n) + r(n+1) - R_body_to_world(n)*r_foot(n) + R_body_to_world(n+1)*r_foot(n+1) <= M*(1 - RR(j,n))
                %if n < obj.N
                  %Rname = sprintf('R%d',i);
                  %RRname = sprintf('RR%d',i);
                  %A(offset + 1, obj.vars.(Rname).i(j,n:n+1)) = 1;
                  %A(offset + 1, obj.vars.(RRname).i(j,n)) = -1;
                  %b(offset + 1) = 1;
                  %offset = offset + 1;
                  %R_body_to_world_next = quat2rotmat(obj.z_fixed_array(:,n+1));
                  %big_M = 2*(obj.position_max+obj.foot_position_max);
                  %ncons = 6;
                  %indices = offset + (1:ncons);
                  %A(indices, obj.vars.r.i(:,n)) = [eye(3); -eye(3)];
                  %A(indices, obj.vars.r.i(:,n+1)) = [-eye(3); eye(3)];
                  %A(indices, obj.vars.(sprintf('r_foot%d',i)).i(:,n)) = [R_body_to_world; -R_body_to_world];
                  %A(indices, obj.vars.(sprintf('r_foot%d',i)).i(:,n+1)) = [-R_body_to_world_next; R_body_to_world_next];
                  %A(indices, obj.vars.(RRname).i(j,n)) = big_M*ones(ncons,1);
                  %b(indices) = big_M;
                  %offset = offset + ncons;
                %end
              end
            end
          end
        end
        obj = obj.addLinearConstraints(A, b, [], []);
      end
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
        obj.regions(j).friction_cone_normals = obj.friction_cone_normals;
      end
      if ~isempty(mu)
        obj.regions(j).mu = mu;
      end
      obj.regions(j).ncons = size(obj.regions(j).A, 1) ...
                            + 2*size(obj.regions(j).Aeq,1);
    end
    
    function obj = addFoot(obj, centers, radii, r_fixed)
      obj.feet(end+1).r_fixed = r_fixed;
      obj.feet(end).ncons = size(centers, 2);
      obj.feet(end).m = obj.foot_mass;
      for j = 1:obj.feet(end).ncons
        obj.feet(end).constraints(j).center = centers(:, j);
        obj.feet(end).constraints(j).radius = radii(j);
      end
    end
    
    function angular_momentum = extractAngularMomentum(obj)
      w = obj.vars.w.value;
      z = obj.vars.z.value;
      angular_momentum = 0*w;
      for n = 1:obj.N
        angular_momentum(:, n) = quatRotateVec(z(:,n), obj.I*w(:,n));
      end
    end
    
    function obj = addPositionConstraint(obj, time_index, lb, ub)
      obj.vars.r.lb(:, time_index) = lb;
      obj.vars.r.ub(:, time_index) = ub;
    end
    
    function obj = addOrientationConstraint(obj, time_index, val)
      obj.vars.z.lb(:, time_index) = val;
      obj.vars.z.ub(:, time_index) = val;
    end
    
    function obj = addVelocityConstraint(obj, time_index, lb, ub)
      obj.vars.v.lb(:, time_index) = lb;
      obj.vars.v.ub(:, time_index) = ub;
    end
    
    function obj = addAngularVelocityConstraint(obj, time_index, lb, ub)
      obj.vars.w.lb(:, time_index) = lb;
      obj.vars.w.ub(:, time_index) = ub;
    end
    
    function obj = addPositionVariables(obj)
      obj = obj.addVariable('r', 'C', ...
        [3, obj.N], -obj.position_max, obj.position_max);
      obj = obj.addVariable('z', 'C', [4, obj.N], -1, 1);
    end

    function obj = addVelocityVariables(obj)
      obj = obj.addVariable('v', 'C', ...
        [3,obj.N], -obj.velocity_max, obj.velocity_max);
      obj = obj.addVariable('w', 'C', ...
        [3,obj.N], -obj.velocity_max, obj.velocity_max);
    end
  end
  
  methods (Static)
    function v_rotated = quatRotateVec2(q1, q2, v)
      rotmat = [1 - 2*q1(3)*q2(3) - 2*q1(4)*q2(4), 2*q1(2)*q2(3) - 2*q1(4)*q2(1),     2*q1(2)*q2(4) + 2*q1(3)*q2(1); ...
        2*q2(2)*q1(3)+2*q2(4)*q1(1),       1 - 2*q1(2)*q2(2) - 2*q1(4)*q2(4), 2*q1(3)*q2(4) - 2*q1(2)*q2(1); ...
        2*q2(2)*q1(4) - 2*q2(3)*q1(1),     2*q2(3)*q1(4) + 2*q2(2)*q1(1)      1 - 2*q1(2)*q2(2) - 2*q1(3)*q2(3)];
      v_rotated = rotmat*v;
    end
  end
end
