classdef Composed < expression.Base & expression.Container
  methods
    function obj = Composed(expr1,expr2)
      assert(isequal_modulo_transforms(expr1.input_frame,expr2.output_frame));
      input_frame = expr2.input_frame;
      output_frame = expr1.output_frame;
      obj = obj@expression.Base(input_frame,output_frame);
      obj = obj@expression.Container(expr1,expr2);
    end

    function [f,df] = fastEval(obj,x)
      [f2,df2_dx] = obj.expr2.fastEval(x);
      [f,df_df2] = obj.expr1.fastEval(f2);
      df = df_df2*df2_dx;
    end
  end
end
