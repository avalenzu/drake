classdef Affine < expression.Base
  properties (SetAccess = protected)
    A
    b
  end
  methods
    function obj = Affine(input_frame,output_frame,A,b)
      sizecheck(A,[output_frame.dim,input_frame.dim]);
      sizecheck(b,[output_frame.dim,1]);
      obj = obj@expression.Base(input_frame,output_frame);
      obj.A = A;
      obj.b = b;
    end

    function [f,df] = fastEval(obj,x)
      f = obj.A*x + obj.b;
      df = obj.A;
    end

    function expr = vertcat(obj,other)
      if isa(other,'expression.Affine') ...
          && obj.input_frame == other.input_frame
        output_frame = MultiCoordinateFrame({obj.output_frame, ...
                                            other.output_frame});
        expr = expression.Affine(obj.input_frame,output_frame, ...
                                 [obj.A;other.A], [obj.b;other.b]);
      else
        expr = vertcat@expression.Base(obj,other);
      end
    end

    function expr = horzcat(obj,other)
      if isa(other,'expression.Affine') ...
          && obj.output_frame == other.output_frame
        input_frame = MultiCoordinateFrame({obj.input_frame, ...
                                            other.input_frame});
        expr = expression.Affine(input_frame,obj.output_frame, ...
                                 [obj.A,other.A], obj.b+other.b);
      else
        expr = horzcat@expression.Base(obj,other);
      end
    end
  end
end
