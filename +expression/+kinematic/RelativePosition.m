classdef RelativePosition < expression.kinematic.Base
  properties (SetAccess = private)
    frameA
    frameB
    pts_in_A
    n_pts
  end

  methods
    function obj = RelativePosition(rbm,frameA,frameB,pts_in_A)
      if nargin < 4
        pts_in_A = zeros(3,1);
      end
      sizecheck(pts_in_A,[3,NaN]);
      n_pts = size(pts_in_A,2);
      output_frame = MultiCoordinateFrame.constructFrame(repmat({expression.frames.R(3)},1,n_pts));
      obj = obj@expression.kinematic.Base(rbm,output_frame);
      obj.frameA = obj.rbm.parseBodyOrFrameID(frameA);
      if obj.frameA == 0
        valuecheck(pts_in_A,zeros(3,1));
      end
      obj.frameB = obj.rbm.parseBodyOrFrameID(frameB);
      obj.pts_in_A = pts_in_A;
      obj.n_pts = n_pts;
    end

    function [pos,J] = fastEval(obj,q)
      kinsol = obj.rbm.doKinematics(q);
      if obj.frameA == 0
        [pts_in_world,JA] = getCOM(obj.rbm,kinsol);
      else
        [pts_in_world,JA] = forwardKin(obj.rbm,kinsol,obj.frameA,obj.pts_in_A,0);
      end
      [T_B_to_world,dT_B_to_world] = forwardKin(obj.rbm,kinsol,obj.frameB,[0;0;0],2);
      [quat_world_to_B,dquat_world_to_B] = quatConjugate(T_B_to_world(4:7));
      dquat_world_to_B = dquat_world_to_B*dT_B_to_world(4:7,:);
      [xyz_world_to_B,dxyz_world_to_B] = quatRotateVec(quat_world_to_B,T_B_to_world(1:3));
      xyz_world_to_B = -xyz_world_to_B;
      dxyz_world_to_B = -dxyz_world_to_B*[dquat_world_to_B;dT_B_to_world(1:3,:)];

      pts_in_B = zeros(3,obj.n_pts)*q(1);
      J = zeros(3*obj.n_pts,obj.rbm.getNumDOF())*q(1);
      for i = 1:obj.n_pts
        [pts_in_B1,dpts_in_B1] = quatRotateVec(quat_world_to_B,pts_in_world(:,i));
        dpts_in_B1 = dpts_in_B1*[dquat_world_to_B;JA(3*(i-1)+(1:3),:)];
        pts_in_B(:,i) = pts_in_B1+xyz_world_to_B;
        J(3*(i-1)+(1:3),:) = dpts_in_B1+dxyz_world_to_B;
      end
      pos = reshape(pts_in_B,[],1);
    end
  end
end
