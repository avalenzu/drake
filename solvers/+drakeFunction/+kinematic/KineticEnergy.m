classdef KineticEnergy < drakeFunction.RigidBodyManipulatorFunction
  methods
    function obj = KineticEnergy(rbm)
      input_frame = rbm.getStateFrame();
      output_frame = drakeFunction.frames.realCoordinateSpace(1);
      obj = obj@drakeFunction.RigidBodyManipulatorFunction(rbm,input_frame,output_frame);
    end

    function [T,dT] = eval(obj,x)
      nq = obj.rbm.getNumPositions();
      [q,v] = obj.getInputFrame().splitCoordinates(x);
      [H,~,~,dH] = obj.rbm.manipulatorDynamics(q,v);
      %dH = dH(:,1:obj.rbm.getNumPositions());
      T = 0.5*v'*H*v;
      dHv = matGradMult(dH,v);
      dT = matGradMult(dHv,v,true) + [zeros(1,nq), v'*H];
    end
  end
end
