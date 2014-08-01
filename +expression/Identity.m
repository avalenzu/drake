classdef Identity < expression.Base
  methods
    function obj = Identity(frame)
      obj = obj@expression.Base(frame,frame);
    end
    function [f,df] = fastEval(obj,x)
      f = x;
      df = eye(numel(x));
    end
  end
end
