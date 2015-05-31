classdef MaximalDynamicsPlant < DrakeSystem
  properties
    NB = 0; % Number of rigid bodies
    m;  % 1 x NB array
    h;  % 3 x NB array
    c;  % 3 x NB array
    I;  % 9 x NB array
    g = [0;0;0;0;0;-9.81];
    body;
  end
  methods
    function obj = MaximalDynamicsPlant(rbm)
      if nargin < 1, return; end
      typecheck(rbm,'RigidBodyManipulator');
      obj = MaximalDynamicsPlant();
      % Start from 2 here, since we don't want a body for the world
      for i = 2:rbm.getNumBodies()
        obj = obj.addBody(rbm.getBody(i));
        obj.body = [obj.body; rbm.getBody(i)];
      end
    end

    function flag = isTI(obj)
      flag = true;
    end

    function nq = getNumPositions(obj);
      nq = obj.NB*7;
    end

    function body = getBody(obj,index)
      body = obj.body(index);
    end

    function num_bodies = getNumBodies(obj)
      num_bodies = obj.NB;
    end

    function obj = addBody(obj, varargin)
      % addBody(obj, body) adds a body to the plant based on the RigidBody body
      %
      % addBody(obj, m, c, I) adds a body to the plant with the mass properties
      % specified by m, c, and I
      if nargin > 2
        valuecheck(nargin,4);
        sizecheck(varargin{1},1);
        sizecheck(varargin{2},[3,1]);
        sizecheck(varargin{3},[3,3]);
        m_body = varargin{1};
        c_body = varargin{2};
        IC_body = varargin{3};
      else
        valuecheck(nargin,2);
        typecheck(varargin{1},'RigidBody');
        m_body = varargin{1}.mass;
        c_body = varargin{1}.com;
        IC_body = varargin{1}.inertia;
      end
      n_position = 7;
      n_velocity = 6;
      n_force = 6;
      h_body = m_body*c_body;
      I_body = IC_body - obj.crossmat(c_body)*obj.crossmat(c_body);
      obj.m = [obj.m, m_body];
      obj.h = [obj.h, h_body];
      obj.c = [obj.c, c_body];
      obj.I = [obj.I, I_body(:)];
      obj.NB = obj.NB + 1;
      obj = obj.setNumContStates(obj.NB*(n_position+n_velocity));
      obj = obj.setStateFrame(obj.constructStateFrame());
      obj = obj.setNumOutputs(obj.NB*(n_position+n_velocity));
      obj = obj.setOutputFrame(obj.constructStateFrame());
      obj = obj.setNumInputs(obj.NB*n_force);
      obj = obj.setInputFrame(obj.constructInputFrame);
    end

    function fr = constructStateFrame(obj)
      quat_frame = CoordinateFrame('quat',4,[],{'quat_w','quat_x','quat_y','quat_z'});
      xyz_frame = CoordinateFrame('xyz',3,[],{'x','y','z'});
      omega_frame = CoordinateFrame('omega',3,'w');
      v_frame = CoordinateFrame('v',3);
      position_frame = MultiCoordinateFrame({quat_frame,xyz_frame});
      velocity_frame = MultiCoordinateFrame({omega_frame,v_frame});
      fr = MultiCoordinateFrame([repmat({position_frame},1,obj.NB), ...
        repmat({velocity_frame},1,obj.NB)]);
    end

    function fr = constructInputFrame(obj)
      moment_frame = CoordinateFrame('moment',3,'n');
      force_frame = CoordinateFrame('force',3,'f');
      spatial_force_frame = MultiCoordinateFrame({moment_frame,force_frame});
      fr = MultiCoordinateFrame.constructFrame(repmat({spatial_force_frame},1,obj.NB));
    end

    function [xdot,dxdot] = dynamics(obj,~,x,u)
      % @param x  -- [quat,r,omega,v] for each body
      % @param u  -- wrench on each body
      n_position = 7;
      n_velocity = 6;
      n_force = 6;
      f = reshape(u,[n_force, obj.NB]);
      df = [zeros(obj.NB*n_force,1+obj.NB*(n_position+n_velocity)), eye(obj.NB*n_force)];
      position = reshape(x(1:obj.NB*n_position),n_position,obj.NB);
      velocity = reshape(x(obj.NB*n_position + (1:obj.NB*n_velocity)),n_velocity,obj.NB);
      n_quat = 4;
      quat = position(1:n_quat,:);
      r = position(n_quat+(1:3),:);
      dposition = [zeros(obj.NB*n_position,1), eye(obj.NB*n_position), zeros(obj.NB*n_position,obj.NB*(n_velocity+n_force))];
      quat_idx = bsxfun(@plus,0:n_position:obj.NB*n_position-1,(1:n_quat)');
      dquat = dposition(quat_idx(:),:);
      r_idx = bsxfun(@plus,0:n_position:obj.NB*n_position-1,(n_quat+(1:3))');
      dr = dposition(r_idx(:),:);
      dvelocity = [zeros(obj.NB*n_velocity,1+obj.NB*n_position), eye(obj.NB*n_velocity), zeros(obj.NB*n_velocity,obj.NB*n_force)];
      omega = velocity(1:3,:);
      omega_idx = bsxfun(@plus,0:n_velocity:obj.NB*n_velocity-1,(1:3)');
      domega = dvelocity(omega_idx(:),:);
      v_idx = bsxfun(@plus,0:n_velocity:obj.NB*n_velocity-1,(4:6)');
      v = velocity(4:6,:);
      dv = dvelocity(v_idx(:),:);
      [I_world,h_world,dI_world_dposition,dh_world_dposition] = transformI(obj.m,obj.h,obj.I,[quat;r]);
      a = zeros(6,obj.NB);
      da = zeros(6*obj.NB,1+obj.NB*(n_position+n_velocity+n_force));
      quatdot = zeros(n_quat, obj.NB);
      dquatdot = sparse(obj.NB*n_quat,1+obj.NB*(n_position+n_velocity+n_force));
      for i = 1:obj.NB
        %[I_world_i,h_world_i,dI_world_dposition_i,dh_world_dposition_i] = transformI(obj.m(i),obj.h(:,i),obj.I(:,i),[quat(:,i);r(:,i)]);
        I_world_i = I_world(:,i);
        h_world_i = h_world(:,i);
        dI_world_dposition_i = dI_world_dposition(:,i);
        dh_world_dposition_i = dh_world_dposition(:,i);
        [hcross, dhcross_dh_world] = obj.crossmat(h_world_i);
        I_i = [reshape(I_world_i,3,3), hcross; ...
              -hcross,                  obj.m(:,i)*eye(3)]; 
        dI_dposition_i = sparse(36,7);
        dposition_i = dposition((i-1)*n_position+(1:n_position),:);
        dvelocity_i = dvelocity((i-1)*n_velocity+(1:n_velocity),:);
        dI_dposition_i = setSubMatrixGradient(dI_dposition_i, reshape(dI_world_dposition_i,9,7), 1:3, 1:3, [6,6]);
        dI_dposition_i = setSubMatrixGradient(dI_dposition_i, dhcross_dh_world*reshape(dh_world_dposition_i,3,7), 1:3, 4:6, [6,6]);
        dI_dposition_i = setSubMatrixGradient(dI_dposition_i, -dhcross_dh_world*reshape(dh_world_dposition_i,3,7), 4:6, 1:3, [6,6]);
        dI_i = dI_dposition_i*dposition_i;

        dquat_i = dquat((i-1)*n_quat+(1:n_quat),:);

        Iv = I_i*velocity(:,i);
        dIv = matGradMultMat(I_i,velocity(:,i),dI_i,dvelocity_i);
        p = crf(velocity(:,i))*Iv;
        dp = dcrf(velocity(:,i),Iv,dvelocity_i,dIv);

        f_minus_p = f(:,i) - p;
        df_minus_p = getSubMatrixGradient(df,1:6,i,[n_force,obj.NB]) - dp;
        a(:,i) = I_i\f_minus_p - obj.g;
        da = setSubMatrixGradient(da, matGradMultMat(inv(I_i),f_minus_p,invMatGrad(I_i,dI_i),df_minus_p), 1:n_velocity, i, [n_velocity, obj.NB]);

        [omega2quatdot, domega2quatdot_dquat_i] = angularvel2quatdotMatrix(quat(:,i));
        domega_i = domega((i-1)*3+(1:3),:);
        quatdot(:,i) = omega2quatdot*omega(:,i);
        dquatdot = setSubMatrixGradient(dquatdot, matGradMultMat(omega2quatdot,omega(:,i),domega2quatdot_dquat_i*dquat_i,domega_i), 1:n_quat,i,[n_quat,obj.NB]);
      end
      xdot = [reshape([quatdot;v],[obj.NB*n_position,1]); reshape(a,[obj.NB*n_velocity,1])];
      dxdot = zeros(obj.NB*(n_position+n_velocity),1+obj.NB*(n_position+n_velocity+n_force));
      dxdot = setSubMatrixGradient(dxdot, dquatdot, quat_idx(:), 1, [obj.NB*(n_position+n_velocity),1]);
      dxdot = setSubMatrixGradient(dxdot, dv, r_idx(:), 1, [obj.NB*(n_position+n_velocity),1]);
      dxdot = setSubMatrixGradient(dxdot, da, obj.NB*n_position+(1:obj.NB*n_velocity), 1, [obj.NB*(n_position+n_velocity),1]);
    end

  end

  % Utilities
  methods (Static = true)
    function [vcross,dvcross] = crossmat(v)
      vcross = [0,    -v(3), v(2); ...
                v(3),  0,   -v(1); ...
               -v(2),  v(1), 0];
      dvcross = [ 0, 0, 0; ...
                  0, 0, 1; ...
                  0,-1, 0; ...
                  0, 0,-1; ...
                  0, 0, 0; ...
                  1, 0, 0; ...
                  0, 1, 0; ...
                 -1, 0, 0; ...
                  0, 0, 0];
    end
  end

  % Tests
  methods (Static = true)
    function p = noargConstructorTest()
      p = MaximalDynamicsPlant();
    end

    function p = rbmConstructorTest()
      r = RigidBodyManipulator(fullfile(getDrakePath(),'examples','Acrobot','Acrobot.urdf'));
      p = MaximalDynamicsPlant(r);
      valuecheck(p.getNumStates(),2*(7+6));
      valuecheck(p.getNumInputs(),2*6);
    end

    function addBodyTest()
      p = MaximalDynamicsPlant();
      m_test = 1;
      c_test = [1;1;1];
      I_test = eye(3);
      p = p.addBody(m_test, c_test, I_test);
    end

    function dynamicsTest()
      rng(1)
      r = RigidBodyManipulator(fullfile(getDrakePath(),'examples','Acrobot','Acrobot.urdf'));
      %r = RigidBodyManipulator(fullfile(getDrakePath(),'examples','Atlas','urdf','atlas_minimal_contact.urdf'));
      p = MaximalDynamicsPlant(r);
      position = zeros(7,p.NB);
      for i = 1:p.NB
        position(:,i) = [normalizeVec(rand(4,1));rand(3,1)];
      end
      x = zeros(p.getNumStates(),1);
      x(1:7*p.NB) = position;
      u = zeros(p.getNumInputs(),1);
      geval_options.grad_method = {'user','numerical'};
      geval_options.diff_type = 'central';
      geval_options.da = 1e-6;
      geval_options.tol = 1e-6;
      [xdot,dxdot] = geval(@p.dynamics,0,x,u,geval_options);
      %[xdot,dxdot] = p.dynamics(0,x,u);
    end

    function runPassive()
      r = RigidBodyManipulator(fullfile(getDrakePath(),'examples','Acrobot','Acrobot.urdf'));
      p = MaximalDynamicsPlant(r);
      [ytraj,xtraj] = p.simulate([0,0.1]);
    end
  end
end
