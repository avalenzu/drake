classdef RelativeFootPositionPolygon
  % For a contact bodyB, specify the polygonal admissible region of the xy position of
  % bodyB in the bodyA's coordinate.
  properties(SetAccess = protected)
    num_vertices  % A integer. The number of the vertices of the polygon
    vertices % A 2 x obj.num_vertices matrix. vertices (:,i) is the xy position of the i'th vertex in the bodyA coordinate
  end
  
  properties(Access = protected)
    A_body % A obj.num_vertices x 2 matrix
    b_body % A obj.num_vertices x 1 matrix. A_body*[x;y]<= b_body indicates point [x;y] is in the admissible polygon, where [x;y] is in the bodyA frame.
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
      edge = diff(obj.vertices,1,2);
      edge_norm = sqrt(sum(edge.*edge,1));
      normal = [edge(2,:);-edge(1,:)]./bsxfun(@times,ones(2,1),edge_norm);
      normal = normal.*bsxfun(@times,ones(2,1),sign(sum(normal.*[edge(:,2:end) edge(:,1)],1)));
      obj.A_body = -normal';
      obj.b_body = -sum(normal.*obj.vertices(:,1:end-1),1)';
      obj.vertices = obj.vertices(:,1:end-1);
    end
    
    function [A,b] = halfspace(obj,yaw)
      % The half space A*[xA;yA;xB;yB] <= b represents [xB;yB] is in the admissible
      % polygon relative to [xA;yA], after rotating by the yaw angle. [xB;yB], [xA;yA] are
      % in the world coordinate
      % @param yaw  A scalar. The yaw angle of bodyA
      if(~isnumeric(yaw) || numel(yaw) ~= 1)
        error('Drake:RelativeFootPositionPolygon:yaw should be a double scalar');
      end
      rotmat = [cos(yaw) -sin(yaw);sin(yaw) cos(yaw)];
      A = obj.A_body*rotmat';
      A = [-A A];
      b = obj.b_body;
    end
  end
end