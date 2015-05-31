function frame = spatialTransform()
  frames = {drakeFunction.frames.quaternion(), ...
            drakeFunction.frames.realCoordinateSpace(3)};
  frame = MultiCoordinateFrame(frames);
end
