function  [C] = G( model, q, grav_accn )

% HandC  Calculate coefficients of equation of motion.
% [H,C]=HandC(model,q,qd,f_ext,grav_accn) calculates the coefficients of
% the joint-space equation of motion, tau=H(q)qdd+C(d,qd,f_ext), where q,
% qd and qdd are the joint position, velocity and acceleration vectors, H
% is the joint-space inertia matrix, C is the vector of gravity,
% external-force and velocity-product terms, and tau is the joint force
% vector.  Algorithm: recursive Newton-Euler for C, and
% Composite-Rigid-Body for H.  f_ext is a cell array specifying external
% forces acting on the bodies.  If f_ext == {} then there are no external
% forces; otherwise, f_ext{i} is a spatial force vector giving the force
% acting on body i, expressed in body i coordinates.  Empty cells in f_ext
% are interpreted as zero forces.  grav_accn is a 3D vector expressing the
% linear acceleration due to gravity.  The arguments f_ext and grav_accn
% are optional, and default to the values {} and [0,0,-9.81], respectively,
% if omitted.
%
% UPDATED by russt:  f_ext is an empty or (sparse) 6 x model.NB matrix

qd = zeros(size(q));

if nargin < 5
  a_grav = [0;0;0;0;0;-9.81];
else
  a_grav = [0;0;0;grav_accn(1);grav_accn(2);grav_accn(3)];
end

for i = 1:model.NB
  %DEBUG
%   fprintf('First loop, body %d\n',i);
  %END_DEBUG
  n = model.dofnum(i);
  [ XJ, S{i} ] = jcalc( model.pitch(i), q(n) );
  Xup{i} = XJ * model.Xtree{i};
  if model.parent(i) == 0
    avp{i} = Xup{i} * -a_grav;
  else
    avp{i} = Xup{i}*avp{model.parent(i)};
  end
  Xup{i} = cleanIfNecessary(Xup{i});
  avp{i} = cleanIfNecessary(avp{i});
  fvp{i} = model.I{i}*avp{i};
  fvp{i} = cleanIfNecessary(fvp{i});
end

if any(cellfun(@(obj) isa(obj,'msspoly'), fvp))
  C = msspoly(zeros(model.NB,1)*q(1));
else
  C = zeros(model.NB,1)*q(1);
end
for i = model.NB:-1:1
  %DEBUG
%   fprintf('Second loop, body %d\n',i);
  %END_DEBUG
  n = model.dofnum(i);
  C(n,1) = S{i}' * fvp{i};
  if model.parent(i) ~= 0
    fvp{model.parent(i)} = cleanIfNecessary(fvp{model.parent(i)} + Xup{i}'*fvp{i});
%     fvp{model.parent(i)} = cleanIfNecessary(fvp{model.parent(i)});
  end
end


function p = cleanIfNecessary(p)
  [m,n] = size(p);
  tol = 1e-4;
  if isa(p,'TrigPoly')
    if deg(getmsspoly(p)) == 0
      p = double(getmsspoly(p));
    else
      p = clean(p,tol);
      % p(1)*0 is a hack to get us back to a TrigPoly if we started with one
      p = p(1)*0 + mss_standardize_trig_power(getmsspoly(p),getCos(p),getSin(p));
    end
  elseif isa(p,'msspoly')
    if deg(p) == 0
      p = double(p);
    else
      p = clean(p,tol);
      p = mss_standardize_trig_power(p,getCos(p),getSin(p));
    end
  else
    return;
  end
  if any(size(p) ~= [m,n])
    p = reshape(p,m,n);
  end
  
  
