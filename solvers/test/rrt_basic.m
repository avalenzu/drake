function rrt_basic

figure(1); clf; hold on;
x_start = rand(2,1);
x_goal = rand(2,1);
plot(x_start(1),x_start(2),'bx',x_goal(1),x_goal(2),'gx','MarkerSize',20,'LineWidth',3);

prob = MotionPlanningProblem(2);
options.max_edge_length = .1;
%xtraj = prob.rrt(x_start,x_goal,@uniformSamples,options);
%[T, path_ids, info] = prob.rrtNew(x_start, x_goal, [], options);
[TA, TB,  path_ids_A, path_ids_B, info] = prob.rrtConnect(x_start, x_goal, [], options);
xtraj = extractTrajectory(TA, path_ids_A, TB, path_ids_B);

fnplt(xtraj);

end

function xs = uniformSamples
xs = rand(2,1);
end
