classdef Exp2Quat < drakeFunction.DrakeFunction
  methods
    function obj = Exp2Quat()
      input_frame = drakeFunction.frames.realCoordinateSpace(3);
      output_frame = drakeFunction.frames.Quaternion();
      obj = obj@drakeFunction.DrakeFunction(input_frame, output_frame);
    end

    function [q, dq] = eval(obj, v)
      [q, dq] = expmap2quat(v);
    end
  end
end
