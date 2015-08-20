function testDisjointConstraints(visualize, n_polygons)
  if nargin < 1, visualize = true; end
  if nargin < 2, n_polygons = 3; end
  polygons = struct('vertices', {}, 'patch', {}, 'A', {}, 'b', {});
  if visualize
    cla
  end
  for i = 1:n_polygons
    polygons(i).vertices = 0.5*rand(10, 2);
    polygons(i).vertices = polygons(i).vertices(convhull(polygons(i).vertices), :);
    polygons(i).vertices = bsxfun(@plus, polygons(i).vertices, 1*(2*rand(1,2)-1));
    if visualize
      polygons(i).patch = patch('XData', polygons(i).vertices(:, 1), 'YData', polygons(i).vertices(:, 2), 'FaceVertexCData', i, 'FaceColor', 'flat');
      hold on
    end
    [polygons(i).A, polygons(i).b] = vert2lcon(polygons(i).vertices);
  end
  prog = MixedIntegerConvexProgram();
  prog = prog.addVariable('x', 'C', [2,1], -5, 5);
  prog = prog.addCost(eye(2), [], []);
  prog = prog.addDisjunctiveConstraint('x', {':'}, {polygons.A}, {polygons.b});

  prog = prog.solve();

  if visualize
    r = norm(prog.vars.x.value);
    th = linspace(0,2*pi);
    x = r*cos(th);
    y = r*sin(th);
    plot(0, 0, 'bo', 'MarkerSize', 10, 'LineWidth', 3);
    plot(prog.vars.x.value(1), prog.vars.x.value(2), 'rx', 'MarkerSize', 10, 'LineWidth', 3);
    plot(x, y, '--')
    axis equal
    hold off
  end
end
