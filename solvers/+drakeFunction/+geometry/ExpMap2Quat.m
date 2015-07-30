classdef ExpMap2Quat < drakeFunction.geometry.QuaternionValued
  methods
    function obj = ExpMap2Quat()
      dim_input = 3;
      obj = obj@drakeFunction.geometry.QuaternionValued(dim_input);
    end

    function [varargout] = eval(~, x)
      varargout = cell(1, max(nargout,1));
      [varargout{:}] = expmap2quat(x);
    end
  end
end
