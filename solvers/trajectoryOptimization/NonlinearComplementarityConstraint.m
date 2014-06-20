classdef NonlinearComplementarityConstraint < ConstraintManager
  % NonlinearComplementarityConstraint
  % A constraint of the form z >= 0, f(x,z) >= 0, <z,f(x,z)> = 0
  %
  % Constraints are applied to the stacked vector [x;z;gamma]
  %   wherever there are slack variables gamma
  %
  % mode 1: (default)
  %         z >= 0 (bb),
  %         f(x,z) >= 0 (nl),
  %         <z,f(x,z)> = 0 (nl) (elementwise)
  %
  % mode 2: (slack variable for nonlinear function)
  %         z >= 0 (bb)
  %         gamma >= 0 (bb, slack var)
  %         f(x,z) - gamma = 0 (nl)
  %         <z,gamma> = 0 (nl)  (elementwise)
  %
  % mode 3: (Fischer-Burmeister)
  %         z + f(x,z) - sqrt(z^2 + f(x,z)^2) (elementwise)
  % mode 4: (prox)
  %         z - max(0,z - r*f(x,z)) for some r
  methods
    
    function obj = NonlinearComplementarityConstraint(fun,xdim,zdim,mode,slack)
      if nargin < 4
        mode = 1;
      end
      if nargin < 5
        slack = 0;
      end
      lincon = {};
      nlcon = {};
      bcon = {};
      n = 0;
      switch mode
        case 1
          bcon = BoundingBoxConstraint([-inf(xdim,1);zeros(zdim,1)],inf(zdim+xdim,1));
          nlcon{1} = NonlinearConstraint(zeros(zdim,1),inf(zdim,1),xdim+zdim,fun);
          nlcon{2} = NonlinearConstraint(zeros(zdim,1),zeros(zdim,1)+slack,xdim+zdim,@prodfun);
        case 2
          bcon = BoundingBoxConstraint([-inf(xdim,1);zeros(2*zdim,1)],inf(2*zdim+xdim,1));
          nlcon{1} = NonlinearConstraint(zeros(zdim,1),zeros(zdim,1),xdim+2*zdim,@slackeq);
          nlcon{2} = NonlinearConstraint(zeros(zdim,1),zeros(zdim,1)+slack,xdim+2*zdim,@slackprod);
          n = zdim;
        case 3
          nlcon = NonlinearConstraint(zeros(zdim,1),zeros(zdim,1),xdim+zdim,@fbfun);
        case 4
          nlcon = NonlinearConstraint(zeros(zdim,1),zeros(zdim,1),xdim+zdim,@proxfun);
      end
      function [f,df] = prodfun(y)
        z = y(xdim+1:xdim+zdim);
        [g,dg] = fun(y);
        f = z.*g;
        df = diag(z)*dg + [zeros(zdim,xdim) diag(g)];
      end
      
      function [f,df] = slackeq(y)
        x = y(1:xdim);
        z = y(xdim+1:xdim+zdim);
        gamma = y(xdim+zdim+1:end);
        [f,df] = fun([x;z]);
        f = f - gamma;
        df = [df zeros(zdim)] - [zeros(zdim,zdim+xdim) eye(zdim)];
      end
      
      function [f,df] = slackprod(y)
        x = y(1:xdim);
        z = y(xdim+1:xdim+zdim);
        gamma = y(xdim+zdim+1:end);
        
        f = z.*gamma;
        df = [zeros(zdim,xdim) diag(gamma) diag(z)];
      end
      
      function [f,df] = fbfun(y)
        x = y(1:xdim);
        z = y(xdim+1:xdim+zdim);
        
        [g,dg] = fun([x;z]);
        
        f = z + g  - sqrt(z.^2 + g.^2);
        df = [zeros(zdim, xdim) eye(zdim)] + dg - diag(1./sqrt(z.^2 + g.^2 + 1e-6)) * ([zeros(zdim, xdim) diag(z)] + diag(g)*dg);
      end
      
      function [f,df] = proxfun(y)
        x = y(1:xdim);
        z = y(xdim+1:xdim+zdim);
        r = 1;
        
        [g,dg] = fun([x;z]);
        
        f = z - max(0,z - r*g);
        df = [zeros(zdim, xdim) eye(zdim)];
        
        I_pos = find(z - r*g >= 0);
        df(I_pos,zdim+I_pos) = 0;
        df(I_pos,:) = df(I_pos,:)-r*dg(I_pos,:);
      end
      
      obj = obj@ConstraintManager(lincon, nlcon, bcon, n);
    end
  end
end