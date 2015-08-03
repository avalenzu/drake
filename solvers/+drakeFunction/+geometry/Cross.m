classdef Cross < drakeFunction.DrakeFunction
  methods
    function obj = Cross()
      dim_input = 6;
      dim_output = 3;
      obj = obj@drakeFunction.DrakeFunction(dim_input, dim_output);
    end

    function [f, df] = eval(~, x)
      f = cross(x(1:3), x(4:6));
      df = [-vectorToSkewSymmetric(x(4:6)), vectorToSkewSymmetric(x(1:3))];
    end
    
    function obj = setSparsityPattern(obj)
      [~, df] = obj.eval(ones(6,1));
      [obj.iCfun, obj.jCvar] = find(df);
    end
  end
end
