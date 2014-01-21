function phat = lumpedToOriginalParams(obj,p,p_orig,np,lp,lp_est,nlp)
%%   Step 3: Geometric program to back out original parameters.
%
% Note: Now this is actually solved as a QP
%
% min | log(p) - log(p_orig) |_2^2 
%  s.t. forall i, lp_i = lp_est_i
%                 pmin <= p <= pmax
%
% decision variables y_i, p_i = e^y_i, log(p_i) = y_i
%   allows me to write this as 
% min | y - log(p_orig) |_2^2 
%  s.t.   lp_i(p) = lp_est_i, 
%  s.t.   log(pmin) <= y <= log(pmax)
%  where lp_i is a monomial in p (with coeff = 1)
%  and taking the log of this constraint gives the equivalent constraint
%         A y = log(lp_est_i) where
%  A(i,j) is the power of the monomial in parameter j

A = sparse(nlp,np);
for i=1:nlp  % todo:  there must be a better way to do this with spotless
  [a,b,M] = decomp(lp(i));
  assert(isscalar(M));
  assert(M==1);
  for j=1:length(a)
    ind = find(match(a(j),p));
    A(i,ind) = b(j);
  end
end

irrelevant_params = find(~any(A));
if ~isempty(irrelevant_params)
  for i=1:length(irrelevant_params)
    warning(['Parameter ',getCoordinateName(getParamFrame(obj),irrelevant_params(i)),' does not impact the dynamics in any way.  Consider removing it from the parameter list']);
  end
%  A(:,irrelevant_params) = [];
%  p_map = find(any(A));
%else
%  p_map = 1:np;
end

[pmin,pmax] = getParamLimits(obj);
assert(all(pmin>=0));  % otherwise I'll have to subtract it out from the coordinates
%  not hard, just not implmented yet.
qpopt = optimset('Display','iter');
% [log_p_est,~,exitflag] = quadprog(eye(np),-log(p_orig),[],[],A,log(lp_est),log(pmin),log(pmax),log(p_orig),qpopt);

R_AT = orth(full(A)');
N_A = null(full(A)');
log_p_est = log(p_orig) + R_AT*((A*R_AT)\(log(lp_est) - A*log(p_orig)));

% log_p_est = full(A\log(lp_est));
exitflag = 1;
if (exitflag~=1)
  warning('quadprog exitflag = %d',exitflag);
end

phat = Point(getParamFrame(obj),exp(log_p_est));
end
