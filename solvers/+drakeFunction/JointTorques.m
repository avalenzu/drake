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
      input_frame = MultiCoordinateFrame({timestep_frame,timestep_frame,position_frame,velocity_frame,velocity_frame,lambda_frame});
      %output_frame = drakeFunction.frames.realCoordinateSpace(rbm.getNumPositions-6);
      output_frame = position_frame;
      obj = obj@drakeFunction.RigidBodyManipulatorFunction(rbm,input_frame,output_frame);
      obj.B = obj.rbm.getB();
      obj.contact_wrench = contact_wrench;
    end

    function [f,df] = eval(obj,x)
      [h_l,h_r,q,v_l,v_r,lambda] = obj.input_frame.splitCoordinates(x);
      h = 0.5*(h_l+h_r);
      vdh = v_r-v_l;
      nH = 2;
      nQ = numel(q);
      nV = numel(v_l);
      nX = nQ+nV;
      nL = numel(lambda);
      dh = [0.5*ones(1,nH), zeros(1,nQ+2*nV+nL)];
      dq = [zeros(nQ,nH),eye(nQ),zeros(nQ,2*nV+nL)];
      dv_l = [zeros(nV,nH+nQ),eye(nV),zeros(nV,nV+nL)];
      dv_r = [zeros(nV,nH+nQ+nV),eye(nV),zeros(nV,nL)];
      dx = [dq;dv_l];
      dvdh_dv_r = eye(nV);
      dvdh_dv_l = -eye(nV);
      dvdh = dvdh_dv_r*dv_r + dvdh_dv_l*dv_l;
      dlambda = [zeros(nL,nH+nQ+2*nV),eye(nL)];
      %dlambda = [zeros(nL,nH+nQ+2*nV+nL)];
      [H,C,~,dH,dC_dx,~] = obj.rbm.manipulatorDynamics(q,v_l);
      dC = dC_dx*dx;
      %dH = dH(:,1:obj.rbm.getNumPositions());
      kinsol = obj.rbm.doKinematics(q,true);
      JtransposeForce = zeros(nQ,1);
      dJtransposeForce = zeros(nQ,nH+nQ+2*nV+nL);
      dJtransposeForce_dq = zeros(nQ,nQ);
      lambda_count = 0;
      %lambda_count = 6;
      for i = 1:numel(obj.contact_wrench)
      %for i = 2
        num_pts_i = obj.contact_wrench{i}.num_pts;
        num_lambda_i = obj.contact_wrench{i}.num_pt_F*num_pts_i;
        lambda_idx_i = lambda_count+(1:num_lambda_i);
        lambda_count = lambda_count+num_lambda_i;
        A_force = obj.contact_wrench{i}.force();
        force_i = A_force*lambda(lambda_idx_i);
        [~,J,dJ] = obj.rbm.forwardKin(kinsol,obj.contact_wrench{i}.body,obj.contact_wrench{i}.body_pts,0);
        JtransposeForce = JtransposeForce + J'*force_i;
        dJtransposeForce_dq = reshape(matGradMult(reshape(dJ,[],nQ),force_i,true),nQ,nQ); 
        dJtransposeForce_dL = J'*A_force;
        dJtransposeForce = dJtransposeForce + dJtransposeForce_dq*dq + dJtransposeForce_dL*dlambda(lambda_idx_i,:);
      end
      dHvdh_dx = matGradMult(dH,vdh);
      dHvdh = dHvdh_dx*dx + H*dvdh;

      f = H*vdh + C*h - JtransposeForce*h;
      %f = f(7:end);
      df = dHvdh + dC*h + C*dh - dJtransposeForce*h - JtransposeForce*dh; 
      %df = df(7:end,:);
    end
  end
end
