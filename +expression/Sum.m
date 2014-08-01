classdef Sum < expression.Base
  properties (SetAccess = private)
    dim
  end
  methods
    function obj = Sum(frame)
      typecheck(frame,'CoordinateFrame');
      input_frame = MultiCoordinateFrame({frame,frame});
      output_frame = frame;
      obj = obj@expression.Base(input_frame,output_frame);
      obj.dim = frame.dim;
    end
    function [f,df] = fastEval(obj,x)
      f = x(1:obj.dim) + x(obj.dim+(1:obj.dim));
      df = [eye(obj.dim),eye(obj.dim)];
    end
  end
end
