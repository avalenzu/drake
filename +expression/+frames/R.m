function frame = R(n)
  name = sprintf('R%d',n);
  frame = SingletonCoordinateFrame(name,n,'e');
end
