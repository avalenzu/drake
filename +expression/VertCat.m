classdef VertCat < expression.Base & expression.Container
  properties
    input_idx1
    input_idx2
    output_idx1
    output_idx2
  end
  methods
    function obj = VertCat(expr1, expr2,same_input)
      if nargin < 3, same_input = false; end
      if same_input
        assert(isequal_modulo_transforms(expr1.getInputFrame(),expr2.getInputFrame()));
        input_frame = expr1.getInputFrame();
      else
        input_frame = MultiCoordinateFrame({expr1.input_frame,expr2.input_frame});
      end
      output_frame = MultiCoordinateFrame({expr1.output_frame,expr2.output_frame});
      obj = obj@expression.Base(input_frame,output_frame);
      obj = obj@expression.Container(expr1,expr2);
      if same_input
        obj.input_idx1 = 1:obj.dim1;
        obj.input_idx2 = obj.input_idx1;
      else
        obj.input_idx1 = 1:obj.dim1;
        obj.input_idx2 = obj.dim1+(1:obj.dim2);
      end
      obj.output_idx1 = 1:obj.expr1.getOutputFrame().dim;
      obj.output_idx2 = obj.expr1.getOutputFrame().dim + (1:obj.expr2.getOutputFrame().dim);
    end
    function [f,df] = fastEval(obj,x)
      [f1,df1] = obj.expr1.fastEval(x(obj.input_idx1));
      [f2,df2] = obj.expr2.fastEval(x(obj.input_idx2));
      try
        f = [f1;f2];
      catch ex
        keyboard
      end
      df = zeros(obj.output_frame.dim,obj.input_frame.dim);
      df(obj.output_idx1,obj.input_idx1) = df1;
      df(obj.output_idx2,obj.input_idx2) = df2;
    end
  end
end
