classdef ElementwiseProduct < expression.Base
  properties (SetAccess = private)
    dim
  end
  methods
    function obj = ElementwiseProduct(frame)
      typecheck(frame,'CoordinateFrame');
      input_frame = MultiCoordinateFrame({frame,frame});
      output_frame = frame;
      obj = obj@expression.Base(input_frame,output_frame);
      obj.dim = frame.dim;
    end
    function [f,df] = fastEval(obj,x)
      f = x(1:obj.dim) .* x(obj.dim+(1:obj.dim));
      df = [diag(x(obj.dim+(1:obj.dim))),diag(x(1:obj.dim))];
    end
  end
end
