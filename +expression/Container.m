classdef Container
  properties (SetAccess = private)
    expr1
    expr2
    dim1
    dim2
  end
  methods
    function obj = Container(expr1,expr2)
      typecheck(expr1,'expression.Base');
      typecheck(expr2,'expression.Base');
      obj.expr1 = expr1;
      obj.expr2 = expr2;
      obj.dim1 = obj.expr1.input_frame.dim;
      obj.dim2 = obj.expr2.input_frame.dim;
    end
  end
end
