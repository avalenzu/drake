classdef Constant < expression.Base
  properties
    value
  end
  methods
    function obj = Constant(frame,value)
      if isscalar(value)
        value = value*ones(frame.dim,1);
      else
        sizecheck(value,[frame.dim,1]);
      end
      obj = obj@expression.Base(CoordinateFrame('x',0),frame);
      obj.value = value;
    end
    function [f,df] = fastEval(obj,x)
      f = obj.value;
      df = zeros(numel(obj.value),0);
    end
  end
end
