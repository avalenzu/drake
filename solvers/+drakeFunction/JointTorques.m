classdef JointTorques < drakeFunction.RigidBodyManipulatorFunction
  properties
    contact_wrench
    B;
  end
  methods
    function obj = JointTorques(rbm,contact_wrench,num_lambdas)
      %num_lambdas = 0;
      %for i = 1:numel(contact_wrench)
        %num_lambdas = num_lambdas + contact_wrench{i}.num_pt_F*contact_wrench{i}.num_pts;
      %end
      timestep_frame = drakeFunction.frames.realCoordinateSpace(1);
      position_frame = rbm.getPositionFrame();
      velocity_frame = rbm.getVelocityFrame();
      lambda_frame = drakeFunction.frames.realCoordinateSpace(num_lambdas);
      input_frame = MultiCoordinateFrame({timestep_frame,position_frame,velocity_frame,velocity_frame,lambda_frame});
      output_frame = position_frame;
      obj = obj@drakeFunction.RigidBodyManipulatorFunction(rbm,input_frame,output_frame);
      obj.B = obj.rbm.getB();
      obj.contact_wrench = contact_wrench;
    end

    function [f,df] = eval(obj,x)
      [h,q,v_l,v_r,lambda] = obj.input_frame.splitCoordinates(x);
      vd = (v_r-v_l)/h;
      nQ = numel(q);
      nV = numel(v_l);
      nX = nQ+nV;
      nL = numel(lambda);
      dh = [1, zeros(1,nQ+2*nV+nL)];
      dq = [zeros(nQ,1),eye(nQ),zeros(nQ,2*nV+nL)];
      dv_l = [zeros(nV,1+nQ),eye(nV),zeros(nV,nV+nL)];
      dv_r = [zeros(nV,1+nQ+nV),eye(nV),zeros(nV,nL)];
      dvd_dv_r = 1/h*eye(nV);
      dvd_dv_l = -1/h*eye(nV);
      dvd_dh = -vd/h;
      dvd = dvd_dh*dh + dvd_dv_r*dv_r + dvd_dv_l*dv_l;
      dlambda = [zeros(nL,1+nQ+2*nV),eye(nL)];
      [H,C,~,dH,dC_dx,~] = obj.rbm.manipulatorDynamics(q,v_r);
      dC = dC_dx*[dv_r;dq];
      %dH = dH(:,1:obj.rbm.getNumPositions());
      kinsol = obj.rbm.doKinematics(q,true);
      JtransposeForce = zeros(nQ,1);
      dJtransposeForce = zeros(nQ,nL);
      lambda_count = 0;
      for i = 1:numel(obj.contact_wrench)
        num_pts_i = obj.contact_wrench{i}.num_pts;
        num_lambda_i = obj.contact_wrench{i}.num_pt_F*num_pts_i;
        lambda_idx_i = lambda_count+(1:num_lambda_i);
        A_force = obj.contact_wrench{i}.force();
        force_i = reshape(A_force*lambda(lambda_idx_i),3,num_pts_i);
        [~,J,dJ] = obj.rbm.forwardKin(kinsol,obj.contact_wrench{i}.body,obj.contact_wrench{i}.body_pts,0);
        JtransposeForce = JtransposeForce + J'*reshape(force_i,[],1);
        dJtransposeForce_dq = dJtransposeForce_dq + reshape(matGradMult(dJ,reshape(force_i,[],1),true),nQ,nQ); 
        dJtransposeForce_dL = J';
        dJtransposeForce = dJtransposeForce_dq*dq + dJtransposeForce_dL*dlambda(lambda_idx_i,:);
      end
      dH_reshaped = reshape(dH,[nQ,nQ,nX]);
      dH_flat = reshape(permute(dH_reshaped,[2,1,3]),[nQ,nQ*nX])';
      dHvd_dx = reshape(dH_flat*qdd,[nQ,nX]);
      dHvd = dHvd_dx*dx + H*dvd;

      f = H*vd + C - obj.B*u - JtransposeForce;
      df = dHvd + dC - obj.B*du - dJtransposeForce; 
    end
  end
end
