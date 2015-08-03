classdef QuaternionConjugate < drakeFunction.geometry.QuaternionValued
  methods
    function obj = QuaternionConjugate()
      dim_input = 4;
      obj = obj@drakeFunction.geometry.QuaternionValued(dim_input);
    end

    function [f, df] = eval(~, x)
      [f, df] = quatConjugate(x);
    end
  end
end
