function [x,F,info] = linearCombinationTest(n_goal,visualize)
  import expression.*
  import expression.euclidean.*
  if nargin < 1, n_goal = 4; end
  if nargin < 2, visualize = false; end

  if visualize
    lcmgl = LCMGLClient('relativePositionTest');
  end

  N = 4;
  R3 = frames.R(3);
  weights_frame = frames.R(N);
  lin_comb_expr = LinearCombination(N,R3);
  weights_expr = Identity(weights_frame);
  weight_sum_expr = Affine(weights_frame,frames.R(1),ones(1,N),0);
  distance_from_origin_expr = expression.euclidean.NormSquared(R3);
  squared_dist_between_pts_expr = ...
    compose(expression.euclidean.NormSquared(R3), ...
            expression.Identity(R3)-expression.Identity(R3));

  goal = rand(3,n_goal);

  for i = 1:n_goal
    goal_in_span_cnstr{i} = ExpressionConstraint(goal(:,i),goal(:,i),lin_comb_expr);
  end
  weight_bounds_cnstr = ExpressionConstraint(zeros(N,1),ones(N,1),weights_expr);
  weight_sum_cnstr = ExpressionConstraint(1,1,weight_sum_expr);
  distance_cost = ExpressionConstraint(-inf,inf,distance_from_origin_expr);
  squared_distance_cost = ExpressionConstraint(-inf,inf,squared_dist_between_pts_expr);

  prog = NonlinearProgramWConstraintObjects((3+n_goal)*N);
  w_inds = reshape(1:n_goal*N,N,[]);
  r_inds = reshape(n_goal*N+(1:3*N),3,N);
  for i = 1:n_goal
    prog = prog.addConstraint(goal_in_span_cnstr{i},[w_inds(:,i);r_inds(:)]);
    prog = prog.addConstraint(weight_bounds_cnstr,w_inds(:,i));
    prog = prog.addConstraint(weight_sum_cnstr,w_inds(:,i));
  end
  for i = 1:N
    %prog = prog.addCost(distance_cost,r_inds(:,i));
    for j = i+1:N
      prog = prog.addCost(squared_distance_cost,r_inds(:,[i,j]));
    end
  end

  if visualize
    prog = prog.addDisplayFunction(@(r)displayCallback(lcmgl,goal,r),r_inds(:));
  end

  [x,F,info] = solve(prog,rand(5*N));
end

function displayCallback(lcmgl,goal,r)
  r = reshape(r,3,[]);
  lcmgl.glDrawAxes();
  lcmgl.glColor4f(1,0,0,1);
  for pt = goal
    lcmgl.sphere(pt,0.01,20,20);
  end
  lcmgl.glColor4f(0,0,1,0.5);
  for pt = r
    lcmgl.sphere(pt,0.02,20,20);
  end
  lcmgl.switchBuffers();
end
