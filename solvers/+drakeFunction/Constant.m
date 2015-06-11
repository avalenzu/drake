classdef Constant < drakeFunction.DrakeFunction
  properties
    value
  end
  methods
    function obj = Constant(value, frame)
      if nargin < 2
        frame = drakeFunction.frames.realCoordinateSpace(numel(value));
      end
      valuecheck(numel(value), frame.dim);
      obj = obj@drakeFunction.DrakeFunction(drakeFunction.frames.Null(), frame);
      obj.value = value;
    end
    
    function [c, dc] = eval(obj, ~)
      c = obj.value;
      dc = zeros(obj.output_frame.dim, 0);
    end
  end
end