function testExpMap2Quat()
  import drakeFunction.geometry.*
  fun = ExpMap2Quat();
 
  x = rand(fun.dim_input, 1);

  [f,df] = geval(@(x) eval(fun,x), x,struct('grad_method',{{'user','taylorvar','numerical'}}, 'tol', 1e-7));
end
