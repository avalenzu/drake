function testHyparApproximation(visualize)
  x0 = [0.1; 0.1; 0.5];
  prog = MixedIntegerConvexProgram();
  prog = prog.addVariable('x', 'C', [3,1], -1, 1);
  Q = zeros(prog.nv);
  c = zeros(prog.nv, 1);
  Q(prog.vars.x.i, prog.vars.x.i) = eye(3);
  c(prog.vars.x.i) = -2*x0;
  prog = prog.addCost(Q, c, []);

  n_sectors = 4;
  
  [prog, vertices_cell] = prog.addHyparApproximation('x', {1}, 'x', {2}, 'x', {3}, n_sectors);

  prog = prog.solve();
  x = prog.vars.x.value;

  if visualize
    plot3(x0(1), x0(2), x0(3), 'o', x(1), x(2), x(3), 'o')
    for i = 1:numel(vertices_cell)
      drawTetrahedron(vertices_cell{i});
    end
  end
end

function drawTetrahedron(vertices, alpha)
  if nargin <2, alpha = 0.5; end
  faces = [1, 2, 3; 1, 2, 4; 1, 3, 4; 2, 3, 4];
  patch('Vertices',vertices','Faces',faces,...
          'FaceVertexCData',hsv(4),'FaceColor','flat', 'FaceAlpha', alpha)
end
