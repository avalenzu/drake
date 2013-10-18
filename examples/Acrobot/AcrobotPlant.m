classdef AcrobotPlant < Manipulator 
  
  properties
    % parameters from Spong95 (except inertias are now relative to the
    % joints)
    % axis)
    l1 = 1; l2 = 2;  
    m1 = 1; m2 = 1;  
    g = 9.81;
    b1=.1;  b2=.1;
%    b1=0; b2=0;
    lc1 = .5; lc2 = 1; 
    Ic1 = .083;  Ic2 = .33;
    
    xG
    uG
  end
  
  methods
    function obj = AcrobotPlant
      obj = obj@Manipulator(2,2);
      obj = setInputLimits(obj,-10,10);

      obj = setInputFrame(obj,CoordinateFrame('AcrobotInput',2,'u',{'tau1','tau2'}));
      obj = setStateFrame(obj,CoordinateFrame('AcrobotState',4,'x',{'theta1','theta2','theta1dot','theta2dot'}));
      obj = setOutputFrame(obj,obj.getStateFrame);
      
      obj.xG = Point(obj.getStateFrame,[pi;0;0;0]);
      obj.uG = Point(obj.getInputFrame,0);
      
%      obj = setParamFrame(obj,CoordinateFrame('AcrobotParams',6,'p',...
%        { 'b1','b2','lc1','lc2','Ic1','Ic2' }));     
%      obj = setParamFrame(obj,CoordinateFrame('AcrobotParams',6,'p',...
%        { 'b1','b2','m1','m2','Ic1','Ic2' }));    
%      obj = setParamFrame(obj,CoordinateFrame('AcrobotParams',2,'p',...
%        { 'b1','b2' }));
     %obj = setParamFrame(obj,CoordinateFrame('AcrobotParams',10,'p',...
%       { 'l1','l2','m1','m2','b1','b2','lc1','lc2','Ic1','Ic2' }));
%    obj = setParamFrame(obj,CoordinateFrame('AcrobotParams',9,'p',...
%      { 'l1','m1','m2','b1','b2','lc1','lc2','Ic1','Ic2' }));
     obj = setParamFrame(obj,CoordinateFrame('AcrobotParams',8,'p',...
       { 'm1','m2','b1','b2','lc1','lc2','Ic1','Ic2' }));
%      obj = setParamFrame(obj,CoordinateFrame('AcrobotParams',1,'p',...
%        { 'lc2' }));
      obj = setParamLimits(obj,zeros(obj.getParamFrame.dim,1));
    end

    function [H,C,B] = manipulatorDynamics(obj,q,qd)
      % keep it readable:
      m1=obj.m1; m2=obj.m2; l1=obj.l1; g=obj.g; lc1=obj.lc1; lc2=obj.lc2; b1=obj.b1; b2=obj.b2;
      I1 = obj.Ic1 + obj.m1*obj.lc1^2; I2 = obj.Ic2 + obj.m2*obj.lc2^2;
      m2l1lc2 = m2*l1*lc2;  % occurs often!

      c = cos(q(1:2,:));  s = sin(q(1:2,:));  s12 = sin(q(1,:)+q(2,:));
      
      h12 = I2 + m2l1lc2*c(2);
      H = [ I1 + I2 + m2*l1^2 + 2*m2l1lc2*c(2), h12; h12, I2 ];
      
      C = [ -2*m2l1lc2*s(2)*qd(2), -m2l1lc2*s(2)*qd(2); m2l1lc2*s(2)*qd(1), 0 ];
      G = g*[ m1*lc1*s(1) + m2*(l1*s(1)+lc2*s12); m2*lc2*s12 ];
            
      % accumate total C and add a damping term:
      C = C*qd + G + [b1;b2].*qd;

%       B = [0; 1];
      B = eye(2);
    end
    
    % todo: also implement sodynamics here so that I can keep the
    % vectorized version?
    
    function [f,df,d2f,d3f] = dynamics(obj,t,x,u)
      f = dynamics@Manipulator(obj,t,x,u);
      if (nargout>1)
        [df,d2f,d3f]= dynamicsGradients(obj,t,x,u,nargout-1);
      end
    end
    
    function x = getInitialState(obj)
      x = .1*randn(4,1);
    end
    
    function [c,V]=balanceLQR(obj)
      Q = diag([10,10,1,1]); R = 1;
      if (nargout<2)
        c = tilqr(obj,obj.xG,obj.uG,Q,R);
      else
        if any(~isinf([obj.umin;obj.umax]))
          error('currently, you must disable input limits to estimate the ROA');
        end
        [c,V] = tilqr(obj,obj.xG,obj.uG,Q,R);
        pp = feedback(obj.taylorApprox(0,obj.xG,obj.uG,3),c);
        options.method='levelSet';
        V=regionOfAttraction(pp,V,options);
      end
    end
    
    function [utraj,xtraj]=swingUpTrajectory(obj)
      x0 = zeros(4,1); tf0 = 4; xf = double(obj.xG);

      con.u.lb = obj.umin;
      con.u.ub = obj.umax;
      con.x0.lb = x0;
      con.x0.ub = x0;
      con.xf.lb = xf;
      con.xf.ub = xf;
      con.T.lb = 2;
      con.T.ub = 6;

      options.method='dircol';
      %options.grad_test = true;
      info=0;
      while (info~=1)
        utraj0 = PPTrajectory(foh(linspace(0,tf0,21),randn(1,21)));
        tic
        [utraj,xtraj,info] = trajectoryOptimization(obj,@cost,@finalcost,x0,utraj0,con,options);
        toc
      end

      function [g,dg] = cost(t,x,u);
        R = 1;
        g = sum((R*u).*u,1);
        dg = [zeros(1,1+size(x,1)),2*u'*R];
        return;
        
        xd = repmat([pi;0;0;0],1,size(x,2));
        xerr = x-xd;
        xerr(1,:) = mod(xerr(1,:)+pi,2*pi)-pi;
        
        Q = diag([10,10,1,1]);
        R = 100;
        g = sum((Q*xerr).*xerr + (R*u).*u,1);
        
        if (nargout>1)
          dgdt = 0;
          dgdx = 2*xerr'*Q;
          dgdu = 2*u'*R;
          dg = [dgdt,dgdx,dgdu];
        end
      end
      
      function [h,dh] = finalcost(t,x)
        h = t;
        dh = [1,zeros(1,size(x,1))];
        return;
        
        xd = repmat([pi;0;0;0],1,size(x,2));
        xerr = x-xd;
        xerr(1,:) = mod(xerr(1,:)+pi,2*pi)-pi;
        
        Qf = 100*diag([10,10,1,1]);
        h = sum((Qf*xerr).*xerr,1);
        
        if (nargout>1)
          dh = [0, 2*xerr'*Qf];
        end
      end  

    end

      
      function V = getPotentialEnergy(obj,q)
        q1 = q(1); q2 = q(2);
        x_1 = [sin(q1);-cos(q1)];
        r_c1 = obj.lc1*x_1;
        r_1 = obj.l1*x_1;
        x_2 = [sin(q1+q2); -cos(q1+q2)];
        r_c2 = r_1 + obj.lc2*x_2;
        V = (obj.m1*r_c1 + obj.m2*r_c2)'*[0;obj.g];
      end
      
      function T = getKineticEnergy(obj,q,qd)
        H = manipulatorDynamics(obj,q,qd);
        T = qd'*H*qd;
      end
      
      function [Kb,h_minimal] = getBaseInertialParameters(obj)
        q=msspoly('q',obj.num_q);
        s=msspoly('s',obj.num_q);
        c=msspoly('c',obj.num_q);
        qt=TrigPoly(q,s,c);
        qd=msspoly('qd',obj.num_q);
        E = obj.getTotalEnergy(qt,qd); 
        [a,b,h]=decomp(getmsspoly(E),[q;s;c;qd]);
        coeff = msspoly(ones(size(b,1),1));
        deg_zero_ind=-1;
        for i=1:size(b,1)
          % would use prod(a'.^b(i,:)) if msspoly implemented prod (bug 1712)
          for k=find(b(i,:))
            coeff(i) = coeff(i).*(a(k)^b(i,k));
          end
          if (deg(coeff(i))==0)
            if (deg_zero_ind>0) error('should only get a single degree zero term'); end
            deg_zero_ind = i;
          end
        end
        
        if (deg_zero_ind>0)
          hb = h(:,deg_zero_ind)*double(coeff(deg_zero_ind));
          h = h(:,[1:deg_zero_ind-1,deg_zero_ind+1:end]);
          lp = coeff([1:deg_zero_ind-1,deg_zero_ind+1:end]);
        else
          hb = zeros(size(E,1));
          lp = coeff;
        end
        h = h+0*qt(1); % There must be a better way to go from msspoly to TrigPoly . . .
        [beta,independent_idx] = baseParameters(h);
        Kb = beta*lp;
        h_minimal = h(independent_idx);
      end
      
      function E = getTotalEnergy(obj,q,qd)
        E = obj.getPotentialEnergy(q) + obj.getKineticEnergy(q,qd);
      end
      
  end
  
end
