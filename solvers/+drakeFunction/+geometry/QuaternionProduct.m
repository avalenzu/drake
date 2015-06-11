classdef QuaternionProduct < drakeFunction.geometry.QuaternionValuedFunction
  methods
    function obj = QuaternionProduct()
      import drakeFunction.frames.*
      input_frame = MultiCoordinateFrame(repmat({Quaternion()}, 1, 2));
      obj = obj@drakeFunction.geometry.QuaternionValuedFunction(input_frame);
    end
    
    function [q, dq] = eval(obj, x)
      [q1, q2] = obj.input_frame.splitCoordinates(x);
      [q, dq] = quatProduct(q1, q2);
    end
  end
end