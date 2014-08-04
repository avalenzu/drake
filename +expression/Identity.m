classdef Identity < expression.Affine
  methods
    function obj = Identity(frame)
      obj = obj@expression.Affine(frame,frame,eye(frame.dim),zeros(frame.dim,1));
    end
  end
end
