classdef RigidBodyMeshPoints < RigidBodyMesh
  % RigidBodyMeshPoints   Represents the convex hull of a set of points
  % This class allows for the programatic creation of geometries
  % consisting of the convex hull of a set of points. Visualization is
  % not yet supported for this class.
  %
  % RigidBodyMeshPoints properties:
  %   points - 3 x m array in which the i-th column specifies the
  %            location of the i-th point in body-frame.
  %
  % See also RigidBodyGeometry, RigidBodyMesh
  methods
    function obj = RigidBodyMeshPoints(points)
      obj = obj@RigidBodyMesh('');
      obj.points = points;
    end
    
    function points = getPoints(obj)
      points = obj.points;
    end
    
    function geometry = serializeToLCM(obj)
      fname = [tempname,'.obj'];
      writeOBJ(obj,fname);
      geometry = drake.lcmt_viewer_geometry_data();
      geometry.type = geometry.MESH;
      geometry.string_data = fname;
      geometry.num_float_data = 1;
      geometry.float_data = 1;  % scale

      geometry.position = obj.T(1:3,4);
      geometry.quaternion = rotmat2quat(obj.T(1:3,1:3));
      geometry.color = [obj.c(:);1.0];
    end

    function writeOBJ(obj,filename)
      % writes the mesh to an alias wavefront file (e.g. for the viewers to
      % parse)
      % adapted from http://www.aleph.se/Nada/Ray/saveobjmesh.m
      obj.vertface2obj(obj.points',convhull(obj.points'),filename);
    end
    function vertface2obj(obj,v,f,name)
      % VERTFACE2OBJ Save a set of vertice coordinates and faces as a
      % Wavefront/Alias Obj file
      % % VERTFACE2OBJ(v,f,fname)
      % %     v is a Nx3 matrix of vertex coordinates.
      % %     f is a Mx3 matrix of vertex indices. 
      % %     fname is the filename to save the obj file.
      %
      fid = fopen(name,'w');

      for i=1:size(v,1)
        fprintf(fid,'v %f %f %f\n',v(i,1),v(i,2),v(i,3));
      end

      fprintf(fid,'g foo\n');

      for i=1:size(f,1);
        fprintf(fid,'f %d %d %d\n',f(i,1),f(i,2),f(i,3));
      end
      fprintf(fid,'g\n');

      fclose(fid);
    end
  end
  
  properties
    points
  end
  
end
