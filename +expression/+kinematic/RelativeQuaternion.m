classdef RelativeQuaternion < expression.kinematic.Base
  properties (SetAccess = private)
    frame_A
    frame_B
  end
  methods
    function obj = RelativeQuaternion(rbm,frame_A,frame_B)
      obj = obj@expression.kinematic.Base(rbm,expression.frames.R(4));
      obj.frame_A = obj.rbm.parseBodyOrFrameID(frame_A);
      obj.frame_B = obj.rbm.parseBodyOrFrameID(frame_B);
    end
    function [quat,dquat] = fastEval(obj,q)
      kinsol = obj.rbm.doKinematics(q);
      [pos_A,J_A] = forwardKin(obj.rbm,kinsol,obj.frame_A,[0;0;0],2);
      [pos_B,J_B] = forwardKin(obj.rbm,kinsol,obj.frame_B,[0;0;0],2);
      quat_a2w = pos_A(4:7,1);
      dquat_a2w = J_A(4:7,:);
      quat_b2w = pos_B(4:7,1);
      dquat_b2w = J_B(4:7,:);
      [quat_w2b,dquat_w2b] = quatConjugate(quat_b2w);
      dquat_w2b = dquat_w2b*dquat_b2w;

      [quat,dquat_a2b] = quatProduct(quat_w2b,quat_a2w);
      dquat = dquat_a2b*[dquat_w2b;dquat_a2w];
    end
  end
end
