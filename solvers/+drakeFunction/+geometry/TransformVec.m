function fcn = TransformVec()
  R3 = drakeFunction.frames.realCoordinateSpace(3);
  rotation_fcn = drakeFunction.geometry.QuatRotateVec();
  translation_fcn = drakeFunction.Sum(R3,2);
  fcn = drakeFunction.Identity(R3) + rotation_fcn;
end
