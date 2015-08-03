function testCross()
  import drakeFunction.geometry.*
  fun1 = Cross();
 
  x = rand(3,1);
  y = rand(3,1);
  [f,df] = fun1([x;y]);
  valuecheck(f,cross(x,y));

  [f,df] = geval(@(x) eval(fun1,x),[x;y],struct('grad_method',{{'user','taylorvar','numerical'}}));
end
