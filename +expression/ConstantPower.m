classdef ConstantPower < expression.Base
  properties
    power
    expr
  end
  methods
    function obj = ConstantPower(expr,power)
      if ~isscalar(power)
        sizecheck(power,[expr.output_frame.dim,1]);
      end
      obj = obj@expression.Base(expr.input_frame,expr.output_frame);
      obj.expr = expr;
      obj.power = power;
    end

    function [f,df] = fastEval(obj,x)
      [f,df] = obj.expr.fastEval(x);
      f = f.^obj.power;
      df = obj.power*diag(x.^(obj.power-1))*df;
    end
  end
end
