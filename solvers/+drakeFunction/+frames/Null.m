classdef Null < SingletonCoordinateFrame
  methods
    function obj = Null()
      obj = obj@SingletonCoordinateFrame('null', 0);
    end
  end
end