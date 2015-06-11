classdef QuaternionConjugate < drakeFunction.geometry.QuaternionValuedFunction
  methods
    function obj = QuaternionConjugate()
      import drakeFunction.frames.*
      obj = obj@drakeFunction.geometry.QuaternionValuedFunction(Quaternion());
    end
    
    function [q, dq] = eval(obj, q)
      [q, dq] = quatConjugate(q);
    end
  end
end