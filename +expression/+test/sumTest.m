function sumTest()
  import expression.*
  frame = CoordinateFrame('x',5);
  expr = Sum(frame);
  x = rand(5,1);
  y = rand(5,1);
  [f,df] = expr.eval([x;y]);
  valuecheck(f,x+y);
  valuecheck(df,[eye(frame.dim),eye(frame.dim)]);

  [f,df] = geval(@expr.eval,[x;y],struct('grad_method',{{'user','taylorvar','numerical'}}));

  expr2 = Identity(frame);
  expr3 = expr2 + expr2;
  [f3,df3] = expr3.eval([x;y]);
  valuecheck(f3,f);
  valuecheck(df3,df);
end
