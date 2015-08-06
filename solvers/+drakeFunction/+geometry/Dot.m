classdef Dot < drakeFunction.DrakeFunction
  methods
    function obj = Dot()
      dim_input = 6;
      dim_output = 1;
      obj = obj@drakeFunction.DrakeFunction(dim_input, dim_output);
    end

    function [f, df] = eval(~, x)
      f = x(1:3)'*x(4:6);
      df = [x(4:6)', x(1:3)'];
    end
  end
end
