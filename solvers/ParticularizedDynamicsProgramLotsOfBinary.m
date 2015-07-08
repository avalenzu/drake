classdef ParticularizedDynamicsProgramLotsOfBinary < MixedIntegerConvexProgram
  properties
    N = 2
    position_max = 100
    velocity_max = 10
    force_max = 1e2
    E = [1, 2; ...
         1, 3; ...
         1, 4; ...
         2, 3; ...
         2, 4; ...
         3, 4]'
    F = 1e1;
    dt = 1e-3;
    edge_length = [];
    vis_model
    mass = ones(4,1);
    inner_approx_shape = 'cube'
    C
    D
    n_sectors
    center_to_face
    n_force_magnitudes = 2;
    n_particles = 4;
  end

  methods
    function obj = ParticularizedDynamicsProgramLotsOfBinary(edge_length, N, dt, shape, n_particles)
      obj = obj@MixedIntegerConvexProgram(true);
      obj.edge_length = edge_length;
      obj.N = N;
      if nargin < 3, dt = 1e-3; end
      if nargin < 4, shape = 'cube'; end
      if nargin < 5, shape = 4; end
      obj.dt = dt;
      obj.n_particles = n_particles;
      obj.mass = ones(obj.n_particles,1);
      if obj.n_particles == 4
        obj.E = [1, 2; ...
          1, 3; ...
          1, 4; ...
          2, 3; ...
          2, 4; ...
          3, 4]';
      else
        obj.E = [1, 2; ...
          2, 3; ...
          3, 1]';
      end
      obj = obj.setInnerApproxShape(shape);
      obj = obj.addPositionVariables();
      obj = obj.addVelocityVariables();
      obj = obj.addForceVariables();
      obj = obj.addEdgeLengthSectorVariables();
      obj = obj.addDynamicsConstraints();
      obj = obj.addEdgeLengthConstraints();
      obj = obj.addForceConstraints();
      obj = obj.addForceCosts();
      options.floating = true;
      %urdf = fullfile(getDrakePath(), 'systems', 'plants', 'test', 'FallingBrick.urdf');
      urdf = 'particle.urdf';
      r = RigidBodyManipulator();
      c = [0.3, 1, 0, 0; ...
           0.3, 0, 1, 0; ...
           0.3, 0, 0, 1];
      for i = 1:obj.n_particles
        r = r.addRobotFromURDF(urdf, [], [], options);
        body = r.body(end);
        body.visual_geometry{1} = body.visual_geometry{1}.setColor(c(:,i));
        r = r.setBody(r.getNumBodies(), body);
      end
      r = r.compile();
      obj.vis_model = r;
    end

    function obj = setInnerApproxShape(obj, shape)
      switch shape
        case 'cube'
          obj.n_sectors = 6;
          C_ = eye(3);
          C_ = [C_; -C_]';
          obj.center_to_face = 1/sqrt(3);
        case 'dodecahedron'
          obj.n_sectors = 12;
          phi = (1+sqrt(5))/2;
          obj.center_to_face = 0.5;%sqrt(3)*phi/(3*sqrt(3-phi));
          face_to_vertex = sqrt(1 - obj.center_to_face^2);
          atan2(face_to_vertex, obj.center_to_face)*180/pi
          %obj.center_to_face = 1;
          C_ = [ ...
                0,  1,  phi; ...
                0,  1, -phi; ...
                0, -1, -phi; ...
                0, -1,  phi; ...
                 1,  phi, 0; ...
                 1, -phi, 0; ...
                -1, -phi, 0; ...
                -1,  phi, 0; ...
                 phi, 0,  1; ...
                -phi, 0,  1; ...
                -phi, 0, -1; ...
                 phi, 0, -1; ...
              ]';
        otherwise
          error('ParticularizedDynamicsProgram:badShape', ...
            '''%s'' is not a valid shape', shape);
      end
      D_ = zeros(size(C_,2));
      for i = 1:size(C_,2) 
        D_(i,:) = sqrt(sum(bsxfun(@minus, C_, C_(:,i)).^2, 1)); 
      end
      distances = unique(D_(1, :));
      for i = 1:numel(distances)
        D_(D_ == distances(i)) = i-1;
      end
      obj.C = bsxfun(@rdivide, C_, sqrt(sum(C_.^2, 1)));
      obj.D = D_;
      obj.inner_approx_shape = shape;
    end

    function obj = addPositionVariables(obj)
      for i = 1:obj.n_particles
        for n = 1:obj.N
          obj = obj.addVariable(obj.positionName(i, n), 'C', ...
                                [3,1], -obj.position_max, ...
                                obj.position_max);
        end
      end
    end

    function obj = addVelocityVariables(obj)
      for i = 1:obj.n_particles
        for n = 1:obj.N
          obj = obj.addVariable(obj.velocityName(i, n), 'C', ...
                                [3,1], -obj.velocity_max, ...
                                obj.velocity_max);
        end
      end
    end

    function obj = addForceVariables(obj)
      for e = obj.E
        for n = 1:obj.N
          obj = obj.addVariable(obj.forceName(e(1), e(2), n), 'C', ...
                                [3,1], -obj.force_max, obj.force_max);
          obj = obj.addVariable(obj.forceMagnitudeName(e(1), e(2), n), 'I', ...
                                [1,1], -obj.n_force_magnitudes, obj.n_force_magnitudes);
        end
      end
    end

    function obj = addEdgeLengthSectorVariables(obj)
      for e = obj.E
        obj = obj.addVariable(obj.edgeLengthSectorName(e(1), e(2)), 'B', ...
          [obj.n_sectors, obj.N], 0, 1);
      end
    end

    function obj = addDynamicsConstraints(obj)
      h = obj.dt;
      g = 0*[0; 0; -9.81];
      for n = 1:(obj.N - 1)
        for i = 1:obj.n_particles
          fi = obj.mass(i)*g;
          fi_next = obj.mass(i)*g;
          ri = obj.vars.(obj.positionName(i, n)).symb;
          ri_next = obj.vars.(obj.positionName(i, n+1)).symb;
          vi = obj.vars.(obj.velocityName(i, n)).symb;
          vi_next = obj.vars.(obj.velocityName(i, n+1)).symb;
          for e = obj.E
            if ismember(i, e)
              if i == e(1)
                j = e(2);
                gij = obj.vars.(obj.forceName(i, j, n)).symb;
                gij_next = obj.vars.(obj.forceName(i, j, n+1)).symb;
              elseif i == e(2)
                j = e(1);
                gij = -obj.vars.(obj.forceName(j, i, n)).symb;
                gij_next = -obj.vars.(obj.forceName(j, i, n+1)).symb;
              end
              rj = obj.vars.(obj.positionName(j, n)).symb;
              rj_next = obj.vars.(obj.positionName(j, n+1)).symb;
              fi = fi +  gij;
              fi_next = fi_next + gij_next;
              %fi = fi + obj.F*(ri-rj) +  gij;
              %fi_next = fi_next + obj.F*(ri_next-rj_next) + gij_next;
            end
          end
          vi_mid = vi + 0.5*h/obj.mass(i)*fi;
          ri_next_desired = ri + h*vi_mid;
          vi_next_desired = vi_mid + 0.5*h/obj.mass(i)*fi_next;
          obj = obj.addSymbolicConstraints(ri_next == ri_next_desired);
          obj = obj.addSymbolicConstraints(vi_next == vi_next_desired);
        end
      end
    end

    function obj = addInitialCOMPositionConstraint(obj, r_com_0_desired)
      r_com_0 = 0;
      for i = 1:obj.n_particles
        ri = obj.vars.(obj.positionName(i, 1)).symb;
        r_com_0 = r_com_0+ri/sum(obj.mass);
      end
      obj = obj.addSymbolicConstraints(r_com_0 == r_com_0_desired);
    end

    function obj = addParticlePositionConstraint(obj, r_desired, n, tol)
      if nargin < 4, tol = 0; end
      r = [];
      debug_idx = 1:obj.n_particles;
      for i = debug_idx
        r = [r, obj.vars.(obj.positionName(i, n)).symb]; %#ok<AGROW>
      end
      if tol > 0
        obj = obj.addSymbolicConstraints(r_desired - tol <= r <= r_desired+tol);
      else
        obj = obj.addSymbolicConstraints(r == r_desired(:,debug_idx));
      end
    end

    function obj = addInitialVelocityConstraints(obj, v0, idx)
      if nargin < 3
        idx = 1:obj.n_particles;
      else
        idx = reshape(idx, 1, []);
      end
      for i = idx
        vi_0 = obj.vars.(obj.velocityName(i, 1)).symb;
        obj = obj.addSymbolicConstraints(vi_0 == v0);
      end
    end

    function obj = addForceConstraints(obj)
      for n = 1:obj.N
        for l = 1:size(obj.E,2)
          e = obj.E(:,l);
          i = e(1);
          j = e(2);
          gij = obj.vars.(obj.forceName(i, j, n)).symb;
          Gij = obj.vars.(obj.forceMagnitudeName(i, j, n)).symb;
          ri = obj.vars.(obj.positionName(i, n)).symb;
          rj = obj.vars.(obj.positionName(j, n)).symb;
          for k = -obj.n_force_magnitudes:obj.n_force_magnitudes
            obj = obj.addSymbolicConstraints(implies(k-0.5 <= Gij <= k+0.5, gij == k*obj.force_max/(obj.n_force_magnitudes*obj.edge_length(l))*(rj-ri)));
          end
        end
      end
    end

    function obj = addForceCosts(obj)
      g = [];
      for n = 1:obj.N
        for e = obj.E
          i = e(1);
          j = e(2);
          g = [g;obj.vars.(obj.forceMagnitudeName(i, j, n)).symb];
          %gij = obj.vars.(obj.forceName(i, j, n)).symb;
          %obj = obj.addSymbolicCost(gij'*gij);
        end
      end
      obj = obj.addSymbolicCost(g'*g);
    end

    function obj = addEdgeLengthConstraints(obj)

      tol = 0.3;
      for k = 1:size(obj.E,2)
        e = obj.E(:, k);
        i = e(1);
        j = e(2);
        sector = obj.vars.(obj.edgeLengthSectorName(i, j)).symb;
        b = obj.edge_length(k)*obj.center_to_face;
        for n = 1:obj.N
          ri = obj.vars.(obj.positionName(i, n)).symb;
          rj = obj.vars.(obj.positionName(j, n)).symb;
          gij = obj.vars.(obj.forceName(i, j, n)).symb;
          %obj = obj.addSymbolicConstraints((rj-ri)'*(rj-ri) <= (1*obj.edge_length(k))^2);
          for sector_idx = 1:size(obj.C, 2)
            c = obj.C(:, sector_idx);
            obj = obj.addSymbolicConstraints(implies(sector(sector_idx,n), c'*(rj - ri) >= b));
            obj = obj.addSymbolicConstraints(c'*(rj - ri) <= b+tol);
            %obj = obj.addSymbolicConstraints(implies(sector(sector_idx,n), cross(c,gij) == 0));
            if n < obj.N
              non_adjacent_idx = obj.D(sector_idx,:) > 1;
              obj = obj.addSymbolicConstraints(sector(sector_idx, n)+sum(sector(non_adjacent_idx,n+1), 1) <= 1);
            end
            if n > 1
              non_adjacent_idx = obj.D(sector_idx,:) > 1;
              obj = obj.addSymbolicConstraints(sector(sector_idx, n)+sum(sector(non_adjacent_idx,n-1), 1) <= 1);
            end
          end
          obj = obj.addSymbolicConstraints(sum(sector(:,n), 1) == 1);
        end
      end
    end

    function xtraj = extractXtraj(obj)
      x_data = zeros(12*obj.n_particles, obj.N);
      t = cumsum(repmat(obj.dt, 1, obj.N));
      for n = 1:obj.N
        positions = zeros(6, obj.n_particles);
        velocities = zeros(6, obj.n_particles);
        for i = 1:obj.n_particles
          positions(1:3, i) = obj.vars.(obj.positionName(i, n)).value;
          velocities(1:3, i) = obj.vars.(obj.velocityName(i, n)).value;
        end
        x_data(:, n) = [positions(:); velocities(:)];
      end
      xtraj = PPTrajectory(foh(t, x_data));
      xtraj = xtraj.setOutputFrame(obj.vis_model.getStateFrame());
    end

    function r_data = extractRData(obj)
      r_data = zeros(obj.n_particles*3, obj.N);
      for n = 1:obj.N
        positions = zeros(3, obj.n_particles);
        for i = 1:obj.n_particles
          positions(1:3, i) = obj.vars.(obj.positionName(i, n)).value;
        end
        r_data(:, n) = positions(:);
      end
    end

    function v_data = extractVData(obj)
      v_data = zeros(obj.n_particles*3, obj.N);
      for n = 1:obj.N
        velocities = zeros(3, obj.n_particles);
        for i = 1:obj.n_particles
          velocities(1:3, i) = obj.vars.(obj.velocityName(i, n)).value;
        end
        v_data(:, n) = velocities(:);
      end
    end

    function ftraj = extractFtraj(obj)
      f_data = zeros(3*size(obj.E,2), obj.N);
      t = cumsum(repmat(obj.dt, 1, obj.N));
      for n = 1:obj.N
        for k = 1:size(obj.E,2)
          i = obj.E(1,k);
          j = obj.E(2,k);
          f_data(3*(k-1)+(1:3), n) = obj.vars.(obj.forceName(i, j, n)).value;
        end
      end
      ftraj = DTTrajectory(t, f_data);
    end

    function v = constructVisualizer(obj, varargin)
      v = obj.vis_model.constructVisualizer(varargin{:});
    end

    function plotAngularMomentum(obj)
      t = cumsum(repmat(obj.dt, 1, obj.N));
      r_data = obj.extractRData();
      v_data = obj.extractVData();
      r_data_3D = reshape(r_data, [3,obj.n_particles,obj.N]);
      v_data_3D = reshape(v_data, [3,obj.n_particles,obj.N]);
      com = (1/sum(obj.mass))*sum(bsxfun(@times, obj.mass', r_data_3D),2);
      v_com = (1/sum(obj.mass))*sum(bsxfun(@times, obj.mass', v_data_3D),2);
      r_data_3D = bsxfun(@minus, r_data_3D, com);
      v_data_3D = bsxfun(@minus, v_data_3D, v_com);
      momentum_3D = bsxfun(@times, obj.mass', v_data_3D);
      angular_momentum = squeeze(sum(cross(r_data_3D, momentum_3D),2));
      subplot(2,1,1)
      plot(t, angular_momentum);
      title('Angular momentum');
      subplot(2,1,2)
      plot(t, squeeze(v_com*sum(obj.mass)));
      title('Linear momentum');
    end

  end

  methods (Static)
    function name = positionName(particle_index, time_index)
      name = sprintf('r%d_%d', particle_index, time_index);
    end

    function name = velocityName(particle_index, time_index)
      name = sprintf('v%d_%d', particle_index, time_index);
    end

    function name = forceName(particle_i, particle_j, time_index)
      name = sprintf('g%d%d_%d', particle_i, particle_j, time_index);
    end

    function name = forceMagnitudeName(particle_i, particle_j, time_index)
      name = sprintf('f%d%d_%d', particle_i, particle_j, time_index);
    end

    function name = edgeLengthSectorName(particle_i, particle_j)
      name = sprintf('S%d%d', particle_i, particle_j);
    end

    function obj = fromVertices(vertices, varargin)
      if size(vertices,2) == 4
        assert(all(size(vertices) == [3,4]));
        E = [1, 2; ...
          1, 3; ...
          1, 4; ...
          2, 3; ...
          2, 4; ...
          3, 4]'; %#ok
      else
        E = [1, 2; ...
          2, 3; ...
          3, 1]'; %#ok
      end
      edges = vertices(:, E(2,:)) - vertices(:, E(1,:)); %#ok
      edge_length = sqrt(sum(edges.^2, 1)); %#ok
      obj = ParticularizedDynamicsProgramLotsOfBinary(edge_length, varargin{:}, size(vertices,2)); %#ok
      for i = 1:size(vertices,2)
        for n = 1:obj.N
          obj.vars.(obj.positionName(i, n)).start = vertices(:,i);
        end
      end
    end
  end
end
