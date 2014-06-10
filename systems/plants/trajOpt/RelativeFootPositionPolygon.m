classdef RelativeFootPositionPolygon < NonlinearConstraint
  % For a contact bodyB, specify the polygonal admissible region of the xy position of
  % bodyB in the bodyA's coordinate.
  properties(SetAccess = protected)
    twoD_flag % A boolean. True if the polygon is in 2D, false if it is in 3D.
    A_body % For 2D, A n x 3 matrix. For 3D, A n x 6 matrix
    b_body % A n x 1 matrix. For 2D, A_body*[x;y;yaw]<= b_body indicates bodyB, with pose [x;y;yaw] in the bodyA frame, satisfies the polygon constraint
           % For 3D, A_body*[x;y;z;roll;pitch;yaw]<=b_body indicates bodyB, with pose
           % [x;y;z;roll;pitch;yaw] in bodyA frame, satisfies the polytope constraint
  end
  
  methods
    function obj = RelativeFootPositionPolygon(polygon)
      % @param polygon   A struct with either V representation or H representation
      % The polygon can be either in 2D on [x;y;yaw], or on 3D on [x;y;z;roll;pitch;yaw]
      if(~isstruct(polygon))
        error('Drake:RelativeFootPositionPolygon:Input should be a struct')
      end
      if(isfield(polygon,'A') && isfield(polygon,'b'))
        if(size(polygon.A,2) == 3)
          is2D = true; % 2D polygon on [x;y;yaw]
        elseif(size(polygon.A,2) == 6)
          is2D = false; % 3D polygon on [x;y;z;roll;pitch;yaw]
        else
          error('Drake:RelativeFootPositionPolygon: size(polygon.A ,2) == 3 or 6');
        end
        if(size(polygon.A,1) ~= size(polygon.b,1) || size(polygon.b,2) ~= 1)
          error('Drake:RelativeFootPositionPolygon: A,b size are not compatible');
        end
      elseif(isfield(polygon,'V'))
        if(size(polygon.V,1) == 3)
          is2D = true;
          H = cddmex('hull',struct('V',polygon.V'));
          Hred = cddmex('reduce_h',H);
          polygon.A = Hred.A;
          polygon.b = Hred.B;
        elseif(size(polygon.V,1) == 6)
          error('Drake:RelativeFootPositionPolygon:not supported yet');
        else
          error('Drake:RelativeFootPositionPolygon: V should have either 3 rows or 6 rows');
        end
      end
      num_faces = size(polygon.A,1);
      if(is2D)
        num_vars = 6;
      else
        num_vars = 12;
      end
      obj = obj@NonlinearConstraint(-inf(num_faces,1),zeros(num_faces,1),num_vars);
      obj.A_body = polygon.A;
      obj.b_body = polygon.b;
      obj.twoD_flag = is2D;
    end
    
    function [c,dc] = eval(obj,x)
      % @param x  if obj.twoD_flag = true; x is a 6 x 1 vector. x =
      % [x1;y1;yaw1;x2;y2;yaw2]. Where [x1;y1;yaw1] is the coordinate of the reference
      % body
      if(obj.is2D)
        posA = x(1:2);
        yawA = x(3);
        posB = x(4:5);
        yawB = x(6);
        rotmatA = [cos(yawA) -sin(yawA);sin(yawA) cos(yawA)];
        drotmatdyawA = [-sin(yawA) -cos(yawA);cos(yawA) -sin(yawA)];
        pos_rel = rotmatA'*(posB-posA);
        dpos_reldx = [-rotmatA' drotmatdyawA'*(posB-posA) rotmatA' zeros(2,1)];
        yaw_rel = yawB-yawA;
        dyaw_reldx = [0 0 -1 0 0 1];
        c = obj.A_body*[pos_rel;yaw_rel]-obj.b_body;
        dc = obj.A_body*[dpos_reldx;dyaw_reldx];
      else
        posA = x(1:3);
        rpyA = x(4:6);
        posB = x(7:9);
        rpyB = x(10:12);
        rotmatA = rpy2rotmat(rpyA);
        rotmatB = rpy2rotmat(rpyB);
        rpy_rel = rotmat2rpy(rotmatA'*rotmatB);
        pos_rel = rotmatA'*(posB-posA);
        c = obj.A_body*[pos_rel;rpy_rel]-obj.b_body;
      end
    end
    
    function [A,b] = halfspace(obj,angleA,angleB)
      % If the polygon is in 2D, then angleA = yawA,angleB=yawB; A*[xA;yA;xB;yB]<=b
      % encodes the polygonal constraint on the relative position of bodyB to the
      % reference bodyA
      % If the polygon is in 3D, then angleA = rpyA,angleB = rpyB;
      % A*[xA;yA;zA;xB;yB;zB]<=b encodes the polygonal constraint on the relative position
      % of bodyB to reference bodyA
      if(obj.twoD_flag)
        sizecheck(angleA,[1,1]);
        sizecheck(angleB,[1,1]);
        rotmatA = [cos(angleA) -sin(angleA);sin(angleA) cos(angleA)];
        angle_rel = angleB-angleA;
        A = [-obj.A_body(:,1:2)*rotmatA' obj.A_body(:,1:2)*rotmatA'];
        b = obj.b_body-obj.A_body(:,3)*angle_rel;
      else
        sizecheck(angleA,[3,1]);
        sizecheck(angleB,[3,1]);
        rotmatA = rpy2rotmat(angleA);
        rotmatB = rpy2rotmat(angleB);
        angle_rel = rotmat2rpy(rotmatA'*rotmatB);
        A = [-obj.A_body(:,1:3)*rotmatA' obj.A_body(:,1:3)*rotmatA'];
        b = obj.b_body-obj.A_body(:,4:6)*angle_rel;
      end
      
    end
  end
end