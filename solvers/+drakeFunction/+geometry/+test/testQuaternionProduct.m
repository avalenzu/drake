function testQuaternionProduct()
  import drakeFunction.geometry.*
  fun = QuaternionProduct();
 
  x = rand(fun.dim_input, 1);

  [f,df] = geval(@(x) eval(fun,x), x,struct('grad_method',{{'user','numerical'}}, 'tol', 1e-7));
end


