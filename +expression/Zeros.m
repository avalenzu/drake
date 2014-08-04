classdef Zeros < expression.Base
  methods
    function obj = Zeros(input_frame,output_frame)
      if nargin < 2, output_frame = input_frame; end
      obj = obj@expression.Base(input_frame,output_frame);
    end
    function [f,df] = fastEval(obj,x)
      f = zeros(obj.output_frame.dim,1);
      df = zeros(obj.output_frame.dim,obj.input_frame.dim);
    end
  end
end
