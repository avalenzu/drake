classdef QuaternionRotateVector < drakeFunction.DrakeFunction
  methods
    function obj = QuaternionRotateVector()
      import drakeFunction.frames.*
      R3 = realCoordinateSpace(3);
      input_frame = MultiCoordinateFrame({Quaternion(), R3});
      obj = obj@drakeFunction.DrakeFunction(input_frame, R3);
    end
    
    function [v_rotated, dv_rotated] = eval(obj, x)
      [q, v] = obj.input_frame.splitCoordinates(x);
      [v_rotated, dv_rotated] = quatRotateVec(q, v);
    end
  end
end