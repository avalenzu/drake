classdef Quaternion < SingletonCoordinateFrame
  methods
    function obj = Quaternion()
      obj = obj@SingletonCoordinateFrame('Quaternion',4,[], {'w','x','y','z'});
    end
  end
end
