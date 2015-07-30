classdef RotateVectorByQuaternion < drakeFunction.DrakeFunction
  methods
    function obj = RotateVectorByQuaternion()
      dim_input = 7;
      dim_output = 3;
      obj = obj@drakeFunction.DrakeFunction(dim_input, dim_output);
    end

    function [varargout] = eval(~, x)
      varargout = cell(1, max(nargout,1));
      [varargout{:}] = quatRotateVec(x(1:4), x(5:end));
    end
  end
end
