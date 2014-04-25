classdef RelativeFootPositionPolygon
  % For a contact bodyB, specify the polygonal admissible region of the xy position of
  % bodyB in the bodyA's coordinate.
  properties(SetAccess = protected)
    num_vertices  % A integer. The number of the vertices of the polygon
    vertices % A 2 x obj.num_vertices matrix. vertices (:,i) is the xy position of the i'th vertex in the bodyA coordinate
  end
  
  methods
    function obj = RelativeFootPositionPolygon(vertices)
      % @param vertices   A 2 x n matrix. vertices(:,i) is the xy position of the i'th
      % vertex in the bodyA coordinate
      if(~isnumeric(vertices))
        error('Drake:RelativeFootPositionPolygon:vertices should be numeric');
      end
      sizecheck(vertices,[2,nan]);
      conv_idx = convhull(vertices(1,:),vertices(2,:));
      obj.num_vertices = length(conv_idx)-1;
      obj.vertices = vertices(:,conv_idx);
      obj.vertices = obj.vertices(:,1:end-1);
    end
  end
end