classdef ScalarProduct < drakeFunction.DrakeFunction
  properties (SetAccess = immutable)
    vector_indices
  end
  methods
    function obj = ScalarProduct(dim_vector)
      dim_input = 1 + dim_vector;
      dim_output = dim_vector;
      obj = obj@drakeFunction.DrakeFunction(dim_input, dim_output);
      obj.vector_indices = 2:(dim_vector + 1);
    end

    function [f, df] = eval(obj, x)
      f = x(1)*x(obj.vector_indices);
      df = [x(obj.vector_indices), x(1)*eye(numel(obj.vector_indices))];
    end
  end
end
