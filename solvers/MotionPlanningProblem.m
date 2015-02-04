classdef MotionPlanningProblem
  % Defines a feasible motion planning problem using Constraint
  % class objects
  %
  % Note: a motion planning problem is *not* a NonlinearProgram
  % because, for instance, it is not described by a fixed number
  % of decision variables.  Instead, we require each point in a
  % solution to be feasible given all of the constraints, and
  % therefore have a NonlinearProgram which tests that feasibility
  % as a member variable.

  properties
    num_vars
    var_names
    constraints={};  % a cell array of Constraint objects
    % todo: add differential constraints?  control affine only?
    % todo: add objectives?
    x_inds={}; % a cell array of indices into the search space. obj.constraints{i}
               % takes x(obj.x_inds{i}) as its input
  end

  methods
    function obj = MotionPlanningProblem(num_vars,var_names)
      % @num_vars the dimension of the search space (e.g. state or
      % configuration space)
      % @var_names optional variable names

      if(nargin<2)
        var_names = cellfun(@(i) sprintf('x%d',i),num2cell((1:obj.num_vars)'),'UniformOutput',false);
      else
        if(~iscellstr(var_names) || numel(var_names) ~= num_vars)
          error('Drake:MotionPlanningProblem:InvalidArgument','Argument x_name should be a cell containing %d strings',obj.num_vars);
        end
        var_names = var_names(:);
      end
      obj.num_vars = num_vars;
      obj.var_names = var_names;

    end

    function [T, path_ids, info] = rrtNew(obj, x_start, x_goal, T, options)
      defaultOptions.display_fcn = @MotionPlanningProblem.drawFirstTwoCoordinates;
      defaultOptions.display_after_every = 50;
      defaultOptions.goal_bias = .05;
      defaultOptions.N = 10000;
      options = applyDefaults(options, defaultOptions);
      if nargin < 4 || isempty(T) 
        T = CartesianMotionPlanningTree(@(q)obj.checkConstraints(q), options.max_edge_length);
        T = T.init(x_start);
      else
        T = T.init(x_start);
        typecheck(T, 'MotionPlanningTree');
        assert(T.isValidConfiguration(x_start))
        assert(T.isValidConfiguration(x_goal))
      end
      last_drawn_edge_num = 1;
      info = 2;
      while T.n < options.N
        try_goal = rand<options.goal_bias;
        if try_goal
          x_sample = x_goal;
        else
          x_sample = T.randomConfig();
        end
        [T, status] = extend(T, x_sample);
        if mod(T.n,options.display_after_every)==0 || (try_goal && status == T.REACHED)
          T = T.drawTree(last_drawn_edge_num);
          drawnow
          last_drawn_edge_num = T.n-1;
        end

        if try_goal && status == T.REACHED
          info = 1;
          path_ids = T.getPathToVertex(T.n);
          drawPath(T, path_ids);
          break;
        end
      end
      path_ids = T.getPathToVertex(T.n);
    end

    function [TA, TB,  path_ids_A, path_ids_B, info] = rrtConnect(obj, x_start, x_goal, TA, TB, options)
      if nargin < 5, options = struct(); end
      defaultOptions.display_fcn = @MotionPlanningProblem.drawFirstTwoCoordinates;
      defaultOptions.display_after_every = 50;
      defaultOptions.max_length_between_constraint_checks = 0.001;
      defaultOptions.goal_bias = .05;
      defaultOptions.N = 10000;
      options = applyDefaults(options, defaultOptions);
      if nargin < 4 || isempty(TA) 
        TA = CartesianMotionPlanningTree(@(q)obj.checkConstraints(q), options.max_edge_length);
        TA = TA.init(x_start);
      else
        typecheck(TA, 'MotionPlanningTree');
        TA = TA.init(x_start);
        assert(TA.isValidConfiguration(x_goal))
      end
      if nargin < 5 || isempty(TB) 
        TB = CartesianMotionPlanningTree(@(q)obj.checkConstraints(q), options.max_edge_length);
        TB = TB.init(x_goal);
      else
        typecheck(TB, 'MotionPlanningTree');
        TB = TB.init(x_goal);
        assert(TB.isValidConfiguration(x_goal))
      end
      info = 2;
      last_drawn_edge_num_A = 1;
      last_drawn_edge_num_B = 1;
      for n = 1:options.N
        if mod(n,2) == 0
          [TA, TB, path_ids_A, path_ids_B] = obj.rrtConnectIteration(TA, TB);
        else
          [TB, TA, path_ids_B, path_ids_A] = obj.rrtConnectIteration(TB, TA);
        end
        if mod(TA.n,options.display_after_every)==0 || mod(TB.n,options.display_after_every)==0 || ~isempty(path_ids_A)
          TA = TA.drawTree(last_drawn_edge_num_A);
          TB = TB.drawTree(last_drawn_edge_num_B);
          drawnow
          last_drawn_edge_num_A = TA.n-1;
          last_drawn_edge_num_B = TB.n-1;
        end
        if ~isempty(path_ids_A)
          assert(~isempty(path_ids_B))
          info = 1;
          drawPath(TA, path_ids_A, TB, path_ids_B);
          break;
        end
      end
    end

    function [TA, TB, path_ids_A, path_ids_B] = rrtConnectIteration(obj, TA, TB)
      x_sample = TA.randomConfig();
      [TA, status, id_new] = extend(TA, x_sample);
      path_ids_A = [];
      path_ids_B = [];
      if status ~= TA.TRAPPED
        [TB, status, id_last] = connect(TB, TA.getVertex(id_new));
        if status == TB.REACHED
          path_ids_A = TA.getPathToVertex(id_new);
          path_ids_B = TB.getPathToVertex(id_last);
        end
      end
    end

    function [xtraj,info,V,parent] = rrt(obj,x_start,x_goal,random_sample_fcn,options)
      % Simple RRT algorithm
      % @param x_start
      % @param x_goal
      % @param tolerance scalar distance that you must achieve to the goal
      % @param random_sample_fcn function handle of the form
      %    xs = random_sample_fcn
      % which generates a nX-by-1 random sample
      % @option distance_metric_fcn function handle of the form
      %    d = distance_metric_fcn(X,xs)
      % where X is a nX-by-N list of points, and xs is an nX-by-1 sample.
      % d should be a 1-by-N list of distances @default euclideanDistance
      % @option display_fcn @default drawFirstTwoCoordinates
      % @option display_after_every draws the tree after this many points
      % are added
      % @option max_edge_length a distance by which to trim
      % @options max_length_between_constraint_checks
      % @options goal_bias @default P(try_goal)=.05
      % @options N maximum number of nodes in the tree @default 10000
      % @options verbose print additional information @default false
      %
      % @retval xtraj search-space trajectory from x_start to x_goal
      % @retval info exit-flag indicating success (info = 1) or failure (info = 2)
      % @retval V nX-by-N list of nodes in the tree
      % @retval parent N-1 list of node indices V(:, parent(i)) is the parent
      %                node of V(:, i)

      % parameters
      sizecheck(x_start,[obj.num_vars,1]);
      sizecheck(x_goal,[obj.num_vars,1]);
      typecheck(random_sample_fcn,'function_handle');

      % options
      if nargin<5, options=struct(); end
      defaultOptions.distance_metric_fcn = @MotionPlanningProblem.euclideanDistance;
      defaultOptions.display_fcn = @MotionPlanningProblem.drawFirstTwoCoordinates;
      defaultOptions.interpolation_fcn = @MotionPlanningProblem.linearInterpolation;
      defaultOptions.display_after_every = 50;
      defaultOptions.max_edge_length = inf;  % because i don't have any sense of scale here
      defaultOptions.max_length_between_constraint_checks = inf;
      defaultOptions.goal_bias = .05;
      defaultOptions.V = [];
      defaultOptions.parent = [];
      defaultOptions.N = 10000;
      defaultOptions.verbose = false;
      options = applyDefaults(options,defaultOptions);
      typecheck(options.distance_metric_fcn,'function_handle');

      if isempty(options.V)
        N = options.N;  % for pre-allocating memory
        V = repmat(x_start,1,N);  % vertices
        parent = nan(1,N-1); % edges

        n=2;
      else
        n = size(options.V,2)+1;
        N = n + options.N - 1;  % for pre-allocating memory
        sizecheck(options.parent,[1,size(options.V,2)-1]);
        V = [options.V,repmat(x_start,1,N-size(options.V,2))];  % vertices
        parent = [options.parent,nan(1,N-size(options.V,2)-1)]; % edges
      end
      n_at_last_display=1;
      info = 2; xtraj = [];  % default return values
      while n<=N
        if options.verbose
          fprintf('n = %d\n',n);
        end
        try_goal = rand<options.goal_bias;
        if try_goal
          xs = x_goal;
        else
          xs = random_sample_fcn();
        end

        d = options.distance_metric_fcn(V(:,1:n-1),xs);
        [dmin,imin] = min(d);

        if try_goal && options.verbose
          fprintf('Min. dist to goal = %f\n',dmin);
        end

        if dmin>options.max_edge_length
          % then linearly interpolate along that distance
          [xs,valid_interp] =options.interpolation_fcn(V(:,imin),xs,(options.max_edge_length/dmin));
          dmin = options.max_edge_length;
          try_goal = false;
        else
          [xs,valid_interp] =options.interpolation_fcn(V(:,imin),xs,1);
        end
%         if ~(valid_interp && checkConstraints(obj,xs)), continue; end
        if ~valid_interp, continue; end
        if dmin>options.max_length_between_constraint_checks
          % then linearly interpolate along that distance
          num_intermediate_samples = 2+ceil(dmin/options.max_length_between_constraint_checks);
          valid = true;
          interp_values = linspace(0,1,num_intermediate_samples);
          for i=2:num_intermediate_samples-1
            [xss_i,valid_interp] = options.interpolation_fcn(V(:,imin),xs,interp_values(i));
            if ~(valid_interp && checkConstraints(obj,xss_i))
              valid=false;
              break;
            end
          end
          if ~valid, continue; end
        else
          if ~(valid_interp && checkConstraints(obj,xs)), continue; end
        end

        V(:,n) = xs;
        parent(n-1) = imin;

        if (try_goal)  % if I get here, then I successfully connected to the goal
          path = n;
          while path(1)>1
            path = [parent(path(1)-1),path];
          end
          xtraj = PPTrajectory(foh(1:length(path),V(:,path)));
          info = 1;
        end

        if mod(n,options.display_after_every)==0 || try_goal
          options.display_fcn(V(:,1:n),parent(1:n-1),n_at_last_display);
          drawnow;
          n_at_last_display = n;
        end

        if try_goal, return; end
        n=n+1;
      end
      d = options.distance_metric_fcn(V,x_goal);
      [~,path] = min(d);
      while path(1)>1
        path = [parent(path(1)-1),path];
      end
      xtraj = PPTrajectory(foh(1:length(path),V(:,path)));
    end

    function obj = addConstraint(obj,constraint,x_inds)
      typecheck(constraint,'Constraint');
      if nargin<3, x_inds = 1:obj.num_vars; end
      sizecheck(x_inds,constraint.xdim);

      obj.x_inds{end+1} = x_inds;
      obj.constraints = vertcat(obj.constraints,{constraint});
    end

    function valid = checkConstraints(obj,x,varargin)
      valid = true;
      for i=1:length(obj.constraints)
        y = eval(obj.constraints{i},x(obj.x_inds{i}),varargin{:});
        if any(y<obj.constraints{i}.lb) || any(y>obj.constraints{i}.ub)
          valid = false;
          return;
        end
      end
    end

  end

  methods (Static=true)
    function d = euclideanDistance(X,xs)
      d = sqrt(sum((X-repmat(xs,1,size(X,2))).^2,1));
    end

    function drawFirstTwoCoordinates(V,parent,last_drawn_edge_num)
      line([V(1,(last_drawn_edge_num+1):end);V(1,parent((last_drawn_edge_num+0):end))],[V(2,last_drawn_edge_num+1:end);V(2,parent(last_drawn_edge_num+0:end))],'Color',0.3*[1 1 1],'Marker','.');
      axis equal
    end

    function [x_interp, valid_interp] = linearInterpolation(x1,x2,alpha)
      x_interp = x1+alpha*(x2 - x1);
      valid_interp = true;
    end
  end

end
