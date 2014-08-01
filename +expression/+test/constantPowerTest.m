function constantPowerTest()
  import expression.*
  frame = CoordinateFrame('x',5);
  expr = Identity(frame).^2;
  x = rand(5,1);
  [f,df] = expr.eval(x);
  valuecheck(f,x.^2);
  valuecheck(df,2*diag(x));

  [f,df] = geval(@expr.eval,x,struct('grad_method',{{'user','taylorvar','numerical'}},'tol',1e-6));
end

