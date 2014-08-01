function vertCatTest()
  import expression.*
  frame1 = CoordinateFrame('x',5);
  frame2 = CoordinateFrame('y',2);
  expr1 = Sum(frame1);
  expr2 = Identity(frame2);
  expr = [expr1;expr2];
  x = rand(frame1.dim,1);
  y = rand(frame2.dim,1);
  [f,df] = expr.eval([x;x;y]);
  valuecheck(f,[x+x;y]);
  valuecheck(df,blkdiag([eye(frame1.dim),eye(frame1.dim)],eye(frame2.dim)));
end
