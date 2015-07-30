classdef QuaternionProduct < drakeFunction.geometry.QuaternionValued
  methods
    function obj = QuaternionProduct()
      dim_input = 8;
      obj = obj@drakeFunction.geometry.QuaternionValued(dim_input);
    end

    function [f, df] = eval(~, x)
      [f, df] = quatProduct(x(1:4), x(5:8));
    end
  end
end
