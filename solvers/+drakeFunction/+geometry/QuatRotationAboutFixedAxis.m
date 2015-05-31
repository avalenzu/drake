classdef QuatRotationAboutFixedAxis < drakeFunction.DrakeFunction
  properties (SetAccess=immutable)
    rotation_axis
  end

  methods
    function obj = QuatRotationAboutFixedAxis(rotation_axis)
      input_frame = drakeFunction.frames.realCoordinateSpace(1);
      output_frame = drakeFunction.frames.Quaternion();
      obj = obj@drakeFunction.DrakeFunction(input_frame,output_frame);
      obj.rotation_axis = normalizeVec(rotation_axis);
    end

    function [q, dq] = eval(obj, theta)
      q = [cos(0.5*theta); sin(0.5*theta)*obj.rotation_axis];
      if nargout > 1
        dq = [-0.5*sin(0.5*theta); 0.5*cos(0.5*theta)*obj.rotation_axis];
      end
    end
  end
end
