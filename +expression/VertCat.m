classdef VertCat < expression.Base & expression.Container
  methods
    function obj = VertCat(expr1, expr2)
      input_frame = MultiCoordinateFrame({expr1.input_frame,expr2.input_frame});
      output_frame = MultiCoordinateFrame({expr1.output_frame,expr2.output_frame});
      obj = obj@expression.Base(input_frame,output_frame);
      obj = obj@expression.Container(expr1,expr2);
    end
    function [f,df] = fastEval(obj,x)
      [f1,df1] = obj.expr1.fastEval(x(1:obj.dim1));
      [f2,df2] = obj.expr2.fastEval(x(obj.dim1+(1:obj.dim2)));
      f = [f1;f2];
      df = blkdiag(df1,df2);
    end
  end
end
