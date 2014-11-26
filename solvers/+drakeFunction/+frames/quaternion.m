function frame = quaterion()
  % frame = drakeFunction.frames.realCoordinateSpace(n) returns a singleton frame
  %   representing the space of quaternions
  %
  % @retval frame   --  SingletonCoordinateFrame representing the space
  %                     of quaternions
  name = 'quaternion';
  frame = SingletonCoordinateFrame(name,4,[],{'w','x','y','z'});
end
