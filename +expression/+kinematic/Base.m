classdef Base < expression.Base
  properties
    rbm %RigidBodyManipulatorObject
  end
  methods
    function obj = Base(rbm,output_frame)
      input_frame = CoordinateFrame('q',rbm.getNumPositions());
      obj = obj@expression.Base(input_frame,output_frame);
      obj.rbm = rbm;
    end
  end
end
