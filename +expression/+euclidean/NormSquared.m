classdef NormSquared < expression.Base
  properties
    Q
    is_weighted
  end
  methods
    function obj = NormSquared(input_frame,Q)
      n = input_frame.dim;
      if nargin < 2, 
        Q = []; 
      else
        sizecheck(Q,[n,n]);
      end
      output_frame = expression.frames.R(1);
      obj = obj@expression.Base(input_frame,output_frame);
      obj.Q = Q;
      obj.is_weighted = ~isempty(Q);
    end

    function [a,da] = fastEval(obj,r)
      if obj.is_weighted
        a = r'*obj.Q*r;
        da = 2*r'*obj.Q;
      else
        a = r'*r;
        da = 2*r';
      end
    end
  end
end
