function r = paramEstStatic(r,options)
if nargin < 2
  options = struct();
end
if nargin < 1
  r = AcrobotPlant();
end
if ~isfield(options,'noise_factor'), options.noise_factor = 0.1; end;

% r = setParamFrame(r,CoordinateFrame('AcrobotParams',3,'p',...
%   {'m2','m1lc1','m2lc2'}));
r = setParamFrame(r,CoordinateFrame('AcrobotParams',2,'p',...
  {'m2','m1lc1'}));
nq = r.getNumDOF();
nu = r.getNumInputs
q=msspoly('q',nq);
s=msspoly('s',nq);
c=msspoly('c',nq);
qt=TrigPoly(q,s,c);
u=msspoly('u',nu);
p=r.getParamFrame.poly;
pr = setParams(r,p);
[H,C,B] = manipulatorDynamics(pr,qt,zeros(nq,1));
[chi,W,rho,lp,beta] = identifiableParameters(getmsspoly(C),p);
q_data = [];
u_data = [];
for i = 1:10
  q_data = [q_data;pi*(2*repmat(rand(nq,1),1,1e4) - 1)];
  u_data = [u_data;findStaticTorques(r,q_data(end-1:end,:))];
end
sigma_u = options.noise_factor*rms(u_data,2);
sigma_q = options.noise_factor*rms(q_data,2);
q_noise = bsxfun(@times,sigma_q,randn(size(q_data)));
u_noise = bsxfun(@times,sigma_u,randn(size(u_data)));
q_data = q_data + q_noise;
u_data = u_data + u_noise;
q_data = reshape(mean(q_data,2),2,[]);
u_data = reshape(mean(u_data,2),2,[]);
s_data = sin(q_data);
c_data = cos(q_data);
nchi = length(chi);
W_col = reshape(W',[],1);
W_raw = full(msubs(W_col,[q;s;c;u],[q_data;s_data;c_data;u_data])');
W_data = reshape(W_raw',nchi,[])';
rho_data = full(msubs(rho,[q;s;c;u],[q_data;s_data;c_data;u_data]));
Y_data = reshape(u_data-rho_data,[],1);
chi_est = W_data\Y_data
p_orig = double(getParams(r));
np = length(p_orig);
nlp = length(lp);
nchi = length(chi);
lp_orig = double(subs(lp,p,p_orig));
chi_orig = double(subs(chi,p,p_orig));
p_guess = p_orig + 2e-1*(2*rand(np,1)-1).*p_orig;
% p_guess = double(phat);
lp_guess = double(subs(lp,p,p_guess));
chi_guess = double(subs(chi,p,p_guess));
R_betaT = orth(full(beta)');
N_beta = null(full(beta)');
lp_est = lp_guess + R_betaT*((beta*R_betaT)\(chi_est - beta*lp_guess));
chi_error = 100*abs((double(chi_est) - chi_orig)./chi_orig);
chi_guess_error = 100*abs((double(chi_guess) - chi_orig)./chi_orig);
lp_error = 100*abs((double(lp_est) - lp_orig)./lp_orig);
lp_guess_error = 100*abs((double(lp_guess) - lp_orig)./lp_orig);
phat = r.lumpedToOriginalParams(p,p_guess,np,lp,lp_est,nlp);
p_error = 100*abs((double(phat) - p_orig)./p_orig);
p_guess_error = 100*abs((double(p_guess) - p_orig)./p_orig);coords = getCoordinateNames(getParamFrame(r));
fprintf('\nIdentifiable parameter estimation results:\n\n');
fprintf('  Param  \tActual  \tGuess   \tEstimated\tError (guess)\tError (estimate)\n');
fprintf('  -----  \t------  \t--------\t---------\t-------------\t----------------\n');
for i=1:nchi
  fprintf('%7d  \t%8.2f\t%8.2f\t%8.2f\t%8.2f%%\t%8.2f%%\n',i,chi_orig(i),chi_guess(i),chi_est(i),chi_guess_error(i),chi_error(i));
end
  
fprintf('\nMonomial estimation results:\n\n');
fprintf('  Param  \tActual  \tGuess   \tEstimated\tError (guess)\tError (estimate)\n');
fprintf('  -----  \t------  \t--------\t---------\t-------------\t----------------\n');
for i=1:nlp
  fprintf('%7d  \t%8.2f\t%8.2f\t%8.2f\t%8.2f%%\t%8.2f%%\n',i,lp_orig(i),lp_guess(i),lp_est(i),lp_guess_error(i),lp_error(i));
end
fprintf('\nParameter estimation results:\n\n');
fprintf('  Param  \tActual  \tGuess   \tEstimated\tError (guess)\tError (estimate)\n');
fprintf('  -----  \t------  \t--------\t---------\t-------------\t----------------\n');
for i=1:length(coords)
  fprintf('%7s  \t%8.2f\t%8.2f\t%8.2f\t%8.2f%%\t%8.2f%%\n',coords{i},p_orig(i),p_guess(i),phat(i),p_guess_error(i),p_error(i));
end

r = setParams(r,double(phat));

