classdef ScalarProduct < drakeFunction.DrakeFunction
  properties (SetAccess = immutable)
    dim_vector
  end
  methods
    function obj = ScalarProduct(dim_vector)
      dim_input = 1 + dim_vector;
      dim_output = dim_vector;
      obj = obj@drakeFunction.DrakeFunction(dim_input, dim_output);
      obj.dim_vector = dim_vector;
    end

    function [f, df] = eval(obj, x)
      f = x(1)*x(1:obj.dim_vector);
      df = [x(1:obj.dim_vector); x(1)*eye(obj.dim_vector)];
    end
  end
end
