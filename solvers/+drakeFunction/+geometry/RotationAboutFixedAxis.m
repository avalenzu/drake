classdef RotationAboutFixedAxis < drakeFunction.geometry.RigidTransformValuedFunction
  properties (SetAccess=immutable)
    rotation_axis
  end

  methods
    function obj = RotationAboutFixedAxis(rotation_axis)
      input_frame = drakeFunction.frames.realCoordinateSpace(1);
      obj = obj@drakeFunction.geometry.RigidTransformValuedFunction(input_frame);
      obj.rotation_axis = normalizeVec(rotation_axis);
    end

    function [T, dT] = eval(obj, theta)
      T = [zeros(3,1); cos(0.5*theta); sin(0.5*theta)*obj.rotation_axis];
      if nargout > 1
        dT = [zeros(3,1); -0.5*sin(0.5*theta); 0.5*cos(0.5*theta)*obj.rotation_axis];
      end
    end
  end
end
