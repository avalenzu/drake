classdef MixedIntegerConvexProgram
% This class is meant to represent general mixed-integer linear, quadratic, quadratically-constrained, and second-order-cone programs. It allows you to define symbolic constraints using Yalmip, which are typically easy to prototype, or to define low-level constraints by directly constructing the A, b, Aeq, beq, etc. matrices, which is typically much faster. For an example of usage, see the MixedIntegerFootstepPlanningProblem subclass. 
  properties
    vars = struct();

    % the number of variables
    nv = 0;

    % the linear cost vector c'x
    c = zeros(0, 1);

    % the quadratic cost matrix x'Qx
    Q = sparse(0, 0);

    % linear inequalities Ax <= b
    A = zeros(0, 0);
    b = zeros(0, 1);

    % linear equalities Ax == b
    Aeq = zeros(0, 0);
    beq = zeros(0, 1);

    % quadratic constraints x'Qc x + q' x <= rhs
    quadcon = struct('Qc', {}, 'q', {}, 'rhs', {});

    % constant term in the objective
    objcon = 0;

    % indices of second-order cones (see http://www.gurobi.com/documentation/5.6/reference-manual/matlab_gurobi)
    cones = struct('index', {});

    % indices of polygonal approximations of second-order cones. The structure of these constraints are designed to mimic the second-order constraints in obj.cones, but they use a polygonal linear outer approximation of the conic constraint. The number of pieces in each approximation is set by N.
    polycones = struct('index', {}, 'N', {});

    % disjunctive constraints
    % Each element of the struct array represents some portion of the decision
    % variables, x, being constrained to lie in the union of a set of convex
    % polyhedra defined by 
    %
    %     A_i*x <= b_i for i = 1, ..., k
    %
    % Implemented as
    %     
    %        A_i*x_i <= b_i*y_i for i = 1, ..., k
    %     sum_i(x_i) == x
    %     sum_i(y_i) == 1
    %            y_i is binary for i = 1, ..., k
    %
    disjunctive_constraints = struct('x', struct('i', {}, 'lb', {}, 'ub', {}), 'A_cell', {}, 'b_cell', {}, 'xi', struct('i', {}), 'y', struct('i', {}), 'ncons', {}, 'k', {}, 'n', {});

    % a list of symbolic constraints constructed with yalmip
    symbolic_constraints;

    % a symbolic objective term constructed in yalmip
    symbolic_objective = 0;

  end

  properties (SetAccess = protected)
    has_symbolic = false;
    symbolic_vars = [];
  end

  methods
    function obj = MixedIntegerConvexProgram(has_symbolic)
      % Construct a new mixed-integer convex program.
      % @param has_symbolic whether to create symbolic variables in yalmip corresponding to
      %                     all of the variables in the problem. If obj.has_symbolic is true,
      %                     you can use both symbolic and non-symbolic (and thus faster) 
      %                     constraints as you wish. If obj.has_symbolic is not true, then
      %                     you cannot, and you must instead construct all of your constraint
      %                     and objective matrices directly.
      if nargin < 1
        has_symbolic = false;
      end
      if has_symbolic
        checkDependency('yalmip');
        obj.symbolic_constraints = lmi();
      end

      checkDependency('gurobi');
      
      obj.has_symbolic = has_symbolic;
    end

    function obj = addVariable(obj, name, type_, size_, lb, ub, start_)
      % Build a struct to hold the sizes and indices of our decision variables
      % This is a new approach that I'm experimenting with, which should offer
      % a mix of the advantages of symbolic and matrix-based optimization
      % frameworks. The idea is that we have a single Matlab struct (named just
      % 'vars' for convenience) and each variable in the optimization has a
      % corresponding named field in vars. For each variable, we have subfields as
      % follows:
      % type: 'B', 'I', or 'C' for binary, integer, or continuous variables
      % size: a 2x1 vector describing the shape of the variable
      % i: the indices corresponding to the variable, as a matrix of the same size as 
      % the 'size' field above. 
      % lb: lower bound, as a matrix of the same size as 'size'
      % ub: upper bound, a matrix
      % start:  the initial values as a matrix of the same size as 'size'
      %
      % After optimization, there will be an additional field added to each variable, called
      % 'value', which will contain the final values after optimization.
      % 
      % The 'i' field of indices is useful because when
      % we actually set up the problem in gurobi or another solver, all of the
      % optimization variables are combined into one long vector. This index
      % field lets us easily address parts of that vector. For example, to set
      % the entry in a constraint matrix A corresponding to the jth row and kth column 
      % of variable 'foo' to 1, we can do the following:
      % A(1, v.foo.i(j,k)) = 1;
      if isfield(obj.vars, name)
        error('Drake:MixedIntegerConvexProgram:DuplicateVariableName', 'Cannot add a variable with the same name as an existing one');
      end
      obj.vars.(name) = struct();
      obj.vars.(name).type = type_;
      obj.vars.(name).size = size_;
      obj.vars.(name).i = reshape(obj.nv + (1:prod(obj.vars.(name).size)), obj.vars.(name).size);
      obj.nv = obj.nv + prod(obj.vars.(name).size);
      if isscalar(lb)
        lb = repmat(lb, obj.vars.(name).size);
      end
      if isscalar(ub)
        ub = repmat(ub, obj.vars.(name).size);
      end
      obj.vars.(name).lb = lb;
      obj.vars.(name).ub = ub;
      if nargin < 7
        start_ = [];
      end
      obj.vars.(name).start = nan(obj.vars.(name).size);
      if size(start_, 1) > obj.vars.(name).size(1)
        start_ = start_(1:obj.vars.(name).size(1),:,:);
      end
      if size(start_, 2) > obj.vars.(name).size(2)
        start_ = start_(:,1:obj.vars.(name).size(2),:);
      end
      obj.vars.(name).start(1:size(start_, 1), 1:size(start_, 2), 1:size(start_, 3)) = start_;

      % Add symbolic variables if we're doing that
      if obj.has_symbolic
        size_cell =num2cell(obj.vars.(name).size);
        if strcmp(obj.vars.(name).type, 'B')
          obj.vars.(name).symb = binvar(size_cell{:}, 'full');
        elseif strcmp(obj.vars.(name).type, 'I')
          obj.vars.(name).symb = intvar(size_cell{:}, 'full');
        else
          obj.vars.(name).symb = sdpvar(size_cell{:}, 'full');
        end
        if isempty(obj.symbolic_vars)
          obj.symbolic_vars = reshape(obj.vars.(name).symb, [], 1);
        else
          obj.symbolic_vars = [obj.symbolic_vars; reshape(obj.vars.(name).symb, [], 1)];
        end
      end

      num_new_vars = prod(obj.vars.(name).size);
      obj.c = [obj.c; zeros(num_new_vars, 1)];
      obj.Q = [obj.Q, sparse(size(obj.Q, 1), num_new_vars);
               sparse(num_new_vars, num_new_vars + size(obj.Q, 2))];
      obj.A = [obj.A, zeros(size(obj.A, 1), num_new_vars)];
      obj.Aeq = [obj.Aeq, zeros(size(obj.Aeq, 1), num_new_vars)];
      if length(obj.quadcon) > 10
        obj.biped.warning_manager.warnOnce('Drake:MixedIntegerConvexProgram:ReallocatingQuadraticConstraints', 'Reallocating matrices for many quadratic constraints. This may be inefficient. If possible, try to finish adding new variables before you start adding quadratic constraints');
      end
      for j = 1:length(obj.quadcon)
        obj.quadcon(j).Qc = [obj.quadcon(j).Qc, sparse(size(obj.quadcon(j).Qc, 1), num_new_vars);
          sparse(num_new_vars, num_new_vars + size(obj.quadcon(j).Qc, 2))];
        obj.quadcon(j).q = [obj.quadcon(j).q; zeros(num_new_vars, 1)];
      end
    end

    function obj = addVariableIfNotPresent(obj, varargin)
      name = varargin{1};
      if isfield(obj.vars, name)
        return
      else
        obj = obj.addVariable(varargin{:});
      end
    end

    function obj = addLinearConstraints(obj, A, b, Aeq, beq)
      obj.A = [obj.A; A];
      obj.b = [obj.b; b];
      obj.Aeq = [obj.Aeq; Aeq];
      obj.beq = [obj.beq; beq];
    end

    function obj = addCones(obj, cones)
      obj.cones = [obj.cones, cones];
    end

    function obj = addConesByIndex(obj, idx)
      obj = obj.addCones(struct('index', mat2cell(idx, size(idx, 1), ones(1, size(idx, 2)))));
    end

    function obj = addPolyCones(obj, polycones)
      % Add polygonal approximations of second-order cones
      obj.polycones = [obj.polycones, polycones];
    end

    function obj = addPolyConesByIndex(obj, idx, N)
      % Polycones only support approximations of cones with two variables on the left-hand side 
      % and one on the right-hand side. That is, we can only approximate the constraint that 
      % norm([x2, x3]) <= x1 
      % with the linear constraints
      % A[x2; x3] <= b
      sizecheck(idx, [3, nan]); 

      if length(N) == 1
        N = repmat(N, 1, size(idx, 2));
      else
        assert(length(N) == size(idx, 2));
      end
      obj = obj.addPolyCones(struct('index', mat2cell(idx, size(idx, 1), ones(1, size(idx, 2))), 'N', num2cell(N)));
    end

    function obj = addConesOrPolyConesByIndex(obj, idx, N)
      if nargin < 3 || isempty(N)
        N = 0;
      end
      if all(N == 0)
        obj = obj.addConesByIndex(idx);
      else
        assert(all(N ~= 0), 'Cannot mix cones and polycones in the same call');
        obj = obj.addPolyConesByIndex(idx, N);
      end
    end

    function obj = addQuadcon(obj, quadcon)
      obj.quadcon = [obj.quadcon, quadcon];
    end

    function obj = addDisjunctiveConstraint(obj, x_name, subs, A_cell, b_cell)
      % Each element of the struct array represents some portion of the decision
      % variables, x, being constrained to lie in the union of a set of convex
      % polyhedra defined by 
      %
      %     A_i*x <= b_i for i = 1, ..., k
      %
      % Implemented as
      %     
      %        A_i*x_i <= b_i*y_i for i = 1, ..., k
      %     sum_i(x_i) == x
      %     sum_i(y_i) == 1
      %            y_i is binary for i = 1, ..., k
      %
      if ~iscell(x_name) 
        x_name = {x_name};
        subs = {subs};
      end
      if ~iscell(subs)
        subs = {subs};
      end
      constraint.x.i = [];
      constraint.x.lb = [];
      constraint.x.ub = [];
      for i = 1:numel(x_name)
        constraint.x.i  = [constraint.x.i; reshape(obj.vars.(x_name{i}).i(subs{i}{:}), [], 1)];
        constraint.x.lb = [constraint.x.lb; reshape(obj.vars.(x_name{i}).lb(subs{i}{:}), [], 1)];
        constraint.x.ub = [constraint.x.ub; reshape(obj.vars.(x_name{i}).ub(subs{i}{:}), [], 1)];
      end

      % Check sizes of polyhedral constraints
      sizecheck(b_cell, size(A_cell));
      k = numel(A_cell);
      n = numel(constraint.x.i);
      for i = 1:k
        m = numel(b_cell{i});
        sizecheck(A_cell{i}, [m, n]);
        b_cell{i} =  b_cell{i} - A_cell{i}*constraint.x.lb;
      end
      constraint.A_cell = A_cell;
      constraint.b_cell = b_cell;

      % Create auxiliary variables
      j = numel(obj.disjunctive_constraints) + 1;

      xi_name = sprintf('disjunctive_constraint_%d_xi',j);
      xi_ub = repmat(constraint.x.ub - constraint.x.lb, [1, k]);
      obj = obj.addVariable(xi_name, 'C', [n, k], 0, xi_ub);
      constraint.xi.i = obj.vars.(xi_name).i;

      y_name = sprintf('disjunctive_constraint_%d_y',j);
      obj = obj.addVariable(y_name, 'B', [1, k], 0, 1);
      constraint.y.i = obj.vars.(y_name).i;

      constraint.ncons = cellfun(@numel,b_cell);
      constraint.k = k;
      constraint.n = n;

      obj.disjunctive_constraints(j) = constraint;
    end

    function [obj, vertices_cell] = addHyparApproximation(obj, x_name, x_inds, y_name, y_inds, z_name, z_inds, n_sectors)
      ind_to_split = [];
      dim_to_split = [];

      for i = 0:ceil(log2(n_sectors))
        ind_to_split = [ind_to_split, 2^i:-1:1]; %#ok
        dim_to_split = [dim_to_split, repmat(2, 1, 2^i)]; %#ok
      end
      ind_to_split(n_sectors:end) = [];
      dim_to_split(n_sectors:end) = [];
      vertices = zeros(3,4);
      if ~iscell(x_name) 
        x_name = {x_name};
        x_inds = {x_inds};
      end
      sizecheck(x_inds, size(x_name));
      for i = 1:numel(x_name)
        vertices(1,:) = vertices(1,:) + ...
          [obj.vars.(x_name{i}).lb(x_inds{i}{:}), ...
          obj.vars.(x_name{i}).ub(x_inds{i}{:}), ...
          obj.vars.(x_name{i}).ub(x_inds{i}{:}), ...
          obj.vars.(x_name{i}).lb(x_inds{i}{:})];
      end
      if ~iscell(y_name) 
        y_name = {y_name};
        y_inds = {y_inds};
      end
      sizecheck(y_inds, size(y_name));
      for i = 1:numel(y_name)
        vertices(2,:) = vertices(2,:) + ...
          [obj.vars.(y_name{i}).lb(y_inds{i}{:}), ...
          obj.vars.(y_name{i}).lb(y_inds{i}{:}), ...
          obj.vars.(y_name{i}).ub(y_inds{i}{:}), ...
          obj.vars.(y_name{i}).ub(y_inds{i}{:})];
      end
      vertices(3,:) = prod(vertices(1:2,:), 1);

      vertices_cell = splitTetrahedron(vertices, dim_to_split, ind_to_split);

      A_cell = cell(1, n_sectors);
      b_cell = cell(1, n_sectors);
      for m = 1:n_sectors
        [A_local, b_local] = vert2lcon(vertices_cell{m}');
        A_local = [repmat(A_local(:,1), [1, numel(x_name)]), ...
                   repmat(A_local(:,2), [1, numel(y_name)]), ...
                   A_local(:,3)];
        A_cell{m} = A_local;
        b_cell{m} = b_local;
      end
      obj = obj.addDisjunctiveConstraint([x_name, y_name, z_name], {x_inds{:}, y_inds{:}, z_inds}, A_cell, b_cell);
      function vertices_cell = splitTetrahedron(vertices, dim_to_split, ind_to_split)
        if nargin < 2, dim_to_split = 1; end
        if nargin < 3, ind_to_split = 1; end
        if nargin < 3
          if dim_to_split == 1
            near_vertices = [1, 2];
            far_vertices = [4,3];
          else
            near_vertices = [4, 1];
            far_vertices = [3, 2];
          end
          mid_near = mean(vertices(:, near_vertices),2);
          mid_far = mean(vertices(:,far_vertices),2);
          vertices_cell{1} = [vertices(:,near_vertices(1)), mid_near, mid_far, vertices(:,far_vertices(1))];
          vertices_cell{2} = [mid_near, vertices(:,near_vertices(2)), vertices(:,far_vertices(2)), mid_far];
        else
          if isnumeric(vertices), vertices = {vertices}; end
          sizecheck(dim_to_split, size(ind_to_split));
          if isscalar(ind_to_split)
            vertices_cell = [vertices(1:ind_to_split-1), splitTetrahedron(vertices{ind_to_split}, dim_to_split), vertices(ind_to_split+1:end)];
          else
            vertices_cell = splitTetrahedron(splitTetrahedron(vertices, dim_to_split(1), ind_to_split(1)), dim_to_split(2:end), ind_to_split(2:end));
          end
        end
      end
    end
    

    function obj = setLinearCost(obj, c)
      obj.c = c;
    end

    function obj = setLinearCostEntries(obj, idx, val)
      obj.c(idx) = val;
    end

    function obj = addCost(obj, Q, c, objcon)
      if ~isempty(Q)
        obj.Q = obj.Q + sparse(Q);
      end
      if ~isempty(c)
        obj.c = obj.c + c;
      end
      if ~isempty(objcon)
        obj.objcon = obj.objcon + objcon;
      end
    end

    function obj = addSymbolicConstraints(obj, expr)
      assert(obj.has_symbolic);
      obj.symbolic_constraints = [obj.symbolic_constraints, expr];
    end

    function obj = addSymbolicCost(obj, expr)
      assert(obj.has_symbolic);
      obj = obj.addSymbolicObjective(expr);
    end

    function obj = addSymbolicObjective(obj, expr)
      assert(obj.has_symbolic);
      obj.symbolic_objective = obj.symbolic_objective + expr;
    end

    function obj = convertPolyCones(obj)
      % Build linear constraints for our polygonal cone approximations
      nconstraints = sum([obj.polycones.N]);
      A = zeros(nconstraints, obj.nv);
      b = zeros(size(A, 1), 1);
      offset = 0;
      for j = 1:length(obj.polycones)
        assert(size(obj.polycones(j).index, 1) == 3, 'polygonal cone approximation only valid for cones with 3 entries (approximates x1 <= norm([x2; x3]))')
        N = obj.polycones(j).N;
        for k = 1:N
          th = (2*pi) / N * (k-1);
          ai = rotmat(th) * [1;0];
          A(offset+1, obj.polycones(j).index(2)) = ai(1);
          A(offset+1, obj.polycones(j).index(3)) = ai(2);
          A(offset+1, obj.polycones(j).index(1)) = -1;
          offset = offset+1;
        end
      end
      assert(offset == nconstraints);

      obj = obj.addLinearConstraints(A, b, [], []);
      obj.polycones = struct('index', {}, 'N', {});
    end

    function obj = convertDisjointConstraints(obj)
      % Build linear constraints for our disjunctive constraints
      ncons = sum([obj.disjunctive_constraints.ncons]);
      ncons_eq = sum([obj.disjunctive_constraints.n]) + numel(obj.disjunctive_constraints);
      A = zeros(ncons, obj.nv);
      b = zeros(ncons, 1);
      Aeq = zeros(ncons_eq, obj.nv);
      beq = zeros(ncons_eq, 1);
      offset = 0;
      offset_eq = 0;
      for j = 1:numel(obj.disjunctive_constraints)
        k = obj.disjunctive_constraints(j).k;
        n = obj.disjunctive_constraints(j).n;
        for i = 1:k
          Ai = obj.disjunctive_constraints(j).A_cell{i};
          bi = obj.disjunctive_constraints(j).b_cell{i};
          ncons = size(Ai, 1);
          indices = offset + (1:ncons);
          A(indices, obj.disjunctive_constraints(j).xi.i(:,i)) = Ai;
          A(indices, obj.disjunctive_constraints(j).y.i(i)) = -bi;
          b(indices) = 0;
          offset = offset + ncons;

          ncons = n;
          indices = offset + (1:ncons);
          A(indices, obj.disjunctive_constraints(j).xi.i(:,i)) = eye(ncons);
          A(indices, obj.disjunctive_constraints(j).y.i(i)) = -(obj.disjunctive_constraints(j).x.ub -obj.disjunctive_constraints(j).x.lb) ;
          b(indices) = 0;
          offset = offset + ncons;
        end

        ncons_eq = obj.disjunctive_constraints(j).n;
        indices = offset_eq + (1:ncons_eq);
        Aeq(indices, obj.disjunctive_constraints(j).x.i(:)) = -eye(ncons_eq);
        Aeq(indices, obj.disjunctive_constraints(j).xi.i(:)) = repmat(eye(ncons_eq), 1, k);
        beq(indices) = -obj.disjunctive_constraints(j).x.lb(:);
        offset_eq = offset_eq + ncons_eq;

        ncons_eq = 1;
        indices = offset_eq + (1:ncons_eq);
        Aeq(indices, obj.disjunctive_constraints(j).y.i(:)) = ones(1, k);
        beq(indices) = 1;
        offset_eq = offset_eq + ncons_eq;
      end
      obj = obj.addLinearConstraints(A, b, Aeq, beq);
    end

    function [obj, solvertime, objval] = solve(obj)
      if obj.has_symbolic
        [obj, solvertime, objval] = obj.solveYalmip();
      else
        [obj, solvertime, objval] = obj.solveGurobi();
      end
    end

    function [obj, solvertime, objval] = solveGurobi(obj, params)
      checkDependency('gurobi');
      if nargin < 2
        params = struct();
      end
      params = applyDefaults(params, struct('outputflag', 0));
      model = obj.getGurobiModel();
      result = gurobi(model, params);
      ok = ~(strcmp(result.status, 'INFEASIBLE') || strcmp(result.status, 'INF_OR_UNBD'));
      if ~ok
        error('Drake:MixedIntegerConvexProgram:InfeasibleProblem', 'The mixed-integer problem is infeasible.');
      end
      objval = result.objval;
      solvertime = result.runtime;
      obj = obj.extractResult(result.x);
    end

    function model = getGurobiModel(obj)
      obj = obj.convertPolyCones();
      obj = obj.convertDisjointConstraints();

      var_names = fieldnames(obj.vars);

      model = struct();
      model.A = sparse([obj.A; obj.Aeq]);
      model.rhs = [obj.b; obj.beq];
      model.sense = [repmat('<', size(obj.A, 1), 1); repmat('=', size(obj.Aeq, 1), 1)];
      model.start = nan(obj.nv, 1);
      model.obj = obj.c;
      model.Q = obj.Q;
      if ~isempty(obj.quadcon)
        model.quadcon = obj.quadcon;
      end
      model.objcon = obj.objcon;
      if ~isempty(obj.cones)
        model.cones = obj.cones;
      end

      % Set up defaults so we can fill them in from v
      model.vtype = repmat('C', obj.nv, 1);
      model.lb = -inf(obj.nv, 1);
      model.ub = inf(obj.nv, 1);
      for j = 1:length(var_names)
        name = var_names{j};
        i = reshape(obj.vars.(name).i, [], 1);
        model.vtype(i) = obj.vars.(name).type;
        model.lb(i) = reshape(obj.vars.(name).lb, [], 1);
        model.ub(i) = reshape(obj.vars.(name).ub, [], 1);
        model.start(i) = reshape(obj.vars.(name).start, [], 1);
      end
    end

    function obj = extractResult(obj, x)
      var_names = fieldnames(obj.vars);
      % Extract the solution
      for j = 1:length(var_names)
        name = var_names{j};
        i = reshape(obj.vars.(name).i, [], 1);
        if obj.vars.(name).type == 'I' 
          obj.vars.(name).value = reshape(round(x(i)), obj.vars.(name).size);
        elseif obj.vars.(name).type == 'B'
          obj.vars.(name).value = reshape(logical(round(x(i))), obj.vars.(name).size);
        else
          obj.vars.(name).value = reshape(x(i), obj.vars.(name).size);
        end
      end
    end

    function [obj, solvertime, objval] = solveYalmip(obj, params)
      checkDependency('gurobi');
      constraints = obj.symbolic_constraints;
      objective = obj.symbolic_objective;

      if nargin < 2 || isempty(params)
        params = sdpsettings('solver', 'gurobi', 'verbose', 0);
      end

      % Now add in any constraints or objectives which were declared non-symbolically
      objective = objective + obj.symbolic_vars' * obj.Q * obj.symbolic_vars + obj.c' * obj.symbolic_vars + obj.objcon;
      constraints = [constraints,...
        obj.Aeq * obj.symbolic_vars == obj.beq,...
        obj.A * obj.symbolic_vars <= obj.b,...
        ];
      var_names = fieldnames(obj.vars);
      for j = 1:length(var_names)
        name = var_names{j};
        constraints = [constraints,...
         obj.vars.(name).lb <= obj.vars.(name).symb,...
         obj.vars.(name).symb <= obj.vars.(name).ub];
       end
      for j = 1:length(obj.quadcon)
        constraints = [constraints,...
          obj.symbolic_vars' * obj.quadcon(j).Qc * obj.symbolic_vars + obj.quadcon(j).q' * obj.symbolic_vars <= obj.quadcon(j).rhs];
      end
      for j = 1:length(obj.cones)
        constraints = [constraints,...
          cone(obj.symbolic_vars(obj.cones(j).index(2:end)), obj.symbolic_vars(obj.cones(j).index(1)))];
      end
      for j = 1:length(obj.polycones)
        constraints = [constraints,...
          polycone(obj.symbolic_vars(obj.polycones(j).index(2:end)), obj.symbolic_vars(obj.polycones(j).index(1)), obj.polycones(j).N)];
      end

      diagnostics = optimize(constraints, objective, params);
      ok = diagnostics.problem == 0 || diagnostics.problem == -1;
      if ~ok
        error('Drake:MixedIntegerConvexProgram:InfeasibleProblem', 'The mixed-integer problem is infeasible.');
      end
      objval = double(objective);
      solvertime = diagnostics.solvertime;
      obj = obj.extractResult(double(obj.symbolic_vars));
    end
  end
end
