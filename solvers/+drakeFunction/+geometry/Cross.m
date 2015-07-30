classdef Cross < drakeFunction.DrakeFunction
  methods
    function obj = Cross()
      dim_input = 6;
      dim_output = 3;
      obj = obj@drakeFunction.DrakeFunction(dim_input, dim_output);
    end

    function [f, df] = eval(x)
      f = cross(x(1:3), x(4:6));
      df = [-vectorToSkewSymmetric(x(1:3)), vectorToSkewSymmetric(x(4:6))];
    end
  end
end
