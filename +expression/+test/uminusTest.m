function uminusTest()
  import expression.*
  frame = CoordinateFrame('x',5);
  expr = -Identity(frame);
  x = rand(5,1);
  [f,df] = expr.eval(x);
  valuecheck(f,-x);
  valuecheck(df,-eye(frame.dim));

  [f,df] = geval(@expr.eval,x,struct('grad_method',{{'user','taylorvar','numerical'}}));
end

