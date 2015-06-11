classdef RigidTransform < MultiCoordinateFrame & SingletonCoordinateFrame

  methods
    function obj = RigidTransform()
      import drakeFunction.frames.*
      obj = obj@SingletonCoordinateFrame('RigidTransform', 7);
      obj = obj@MultiCoordinateFrame({realCoordinateSpace(3), Quaternion()});
    end
  end
  
end

