classdef QuaternionValued < drakeFunction.DrakeFunction
  methods
    function obj = QuaternionValued(dim_input)
      dim_output = 4;
      obj = obj@drakeFunction.DrakeFunction(dim_input, dim_output);
    end
  end
end
