function [phat,p_guess,estimated_delay] = parameterEstimation(obj,data,options)
%%
% Parameter estimation algorithm for manipulators
%
% Attempts to minimize the objective
%
% $$ \min_p  | H(q,p)qddot - C(q,qd,p) - B(q,qd,p)*u |_2^2 $$
%
% by extracting an affine representation using lumped-parameters and then
% running least-squares.
%
% Restrictions:
%   H,C,and B must be trig/poly in q,qd and polynomial in p.
%   All params p must be lower bounded (general constraints are not
%   implemented yet, but would be easy if they are affine)
%   so far, I require full-state feedback
%
% Algorithm:
%
% # Extract lumped-parameters
%    Use TrigPoly + spotless to parse H,C,B and extract unique monomial
%    coefficients in p.
% # Least-squares estimation of the lumped-parameters (more many
%      candidate unit delays)
%      Insert the data and do linear regression
% # Geometric program to back out original parameters.
%
% @param data an instance of iddata (from the system identification
% toolbox; see 'help iddata' for more info) containing the data to
% be used for estimation.
%

if (getOutputFrame(obj)~=getStateFrame(obj))
  error('Only full-state feedback is implemented so far');
end

if ~isfield(options,'Fcutoff'), options.Fcutoff = 5; end;
if ~isfield(options,'Fdecimate'), options.Fdecimate = 10; end;
if ~isfield(options,'filter_order'), options.filter_order = 8; end;
if ~isfield(options,'window'), options.window = 41; end;
if ~isfield(options,'method'), options.method = 'dynamics'; end;
if ~isfield(options,'filter'), options.filter = 'off'; end;
if ~isfield(options,'timesteps_per_interval'), options.timesteps_per_interval = 10; end;

nq = obj.num_q;
nu = obj.num_u;
p_orig = double(getParams(obj));  % probably only for testing


q=msspoly('q',nq);
s=msspoly('s',nq);
c=msspoly('c',nq);
qt=TrigPoly(q,s,c);
qd=msspoly('qd',nq);
qdd=msspoly('qdd',nq);
u=msspoly('u',nu);
p=obj.getParamFrame.poly;
pobj = setParams(obj,p);

%% Process Data
if (nargin>1)
  % populate A and b matrices from iddata
  % todo: make the processing of q,qd,qdd more robust
  Ts = get(data,'Ts');
  t_data = get(data,'SamplingInstants')';
  x_data = get(data,'OutputData')';
  q_data = x_data(1:nq,:);
  qd_data = x_data(nq+(1:nq),:);
%   qdd_data = x_data(2*nq+(1:nq),:);
  u_data = get(data,'InputData')';
  
  switch options.filter
    case 'lowpass'
      h=fdesign.lowpass('N,F3dB',options.filter_order,options.Fcutoff*2*Ts);
      d1 = design(h,'butter');
      u_data = filtfilt(d1.sosMatrix,d1.ScaleValues,u_data')';
      q_data = filtfilt(d1.sosMatrix,d1.ScaleValues,q_data')';
      qd_data = filtfilt(d1.sosMatrix,d1.ScaleValues,qd_data')';
      qdd_data = gradient(qd_data,Ts);
      qdd_data = filtfilt(d1.sosMatrix,d1.ScaleValues,qdd_data')';
%       d1 = design(h,'maxflat');
%       u_data = filtfilt(d1.Numerator,1,u_data')';
%       q_data = filtfilt(d1.Numerator,1,q_data')';
%       qd_data = filtfilt(d1.Numerator,1,qd_data')';
%       qdd_data = gradient(qd_data,Ts);
%       qdd_data = filtfilt(d1.Numerator,1,qdd_data')';
      
      n_discard = 20;
      t_data = t_data(:,(n_discard+1):(end-n_discard));
      q_data = q_data(:,(n_discard+1):(end-n_discard));
      qd_data = qd_data(:,(n_discard+1):(end-n_discard));
      qdd_data = qdd_data(:,(n_discard+1):(end-n_discard));
      u_data = u_data(:,(n_discard+1):(end-n_discard));
    case 'decimate'
      r = options.timesteps_per_interval;
      t_data = t_data(1:r:end)';
      q_data = decimateMultiCol(q_data',r,options.filter_order)';
      qd_data = decimateMultiCol(qd_data',r,options.filter_order)';
      qdd_data = gradient(qd_data,Ts*r);
      u_data = decimateMultiCol(u_data',r,options.filter_order)';
    case 'sgolay'
      u_data = sgolayfilt(u_data,options.filter_order,options.window,[],2);
      q_data = sgolayfilt(q_data,options.filter_order,options.window,[],2);
      qd_data = sgolayfilt(qd_data,options.filter_order,options.window,[],2);
      qdd_data = gradient(qd_data,Ts);
      qdd_data = sgolayfilt(qdd_data,options.filter_order,options.window,[],2);
    case 'off'
      qdd_data = gradient(qd_data,Ts);
  end
  
  %DEBUG
  colorOrder = get(gca, 'ColorOrder');
  figure(7); plot(t_data,qdd_data(1,:), 'Color', colorOrder(mod(length(get(gca, 'Children')), size(colorOrder, 1))+1, :));
  hold on;
  %END_DEBUG
  
else  % temporary... just for debugging
  n = 1000;
  t_data = .01*(0:n-1);
  q_data = randn(nq,n);
  qd_data = randn(nq,n);
  u_data = randn(nu,n);
end

% just for debugging
% for i=1:length(t_data)
%   [H,C,B] = manipulatorDynamics(obj,q_data(:,i),qd_data(:,i));
%   qdd_data_dyn(:,i) = (H\(B*u_data(:,i) - C));
%   %qdd_data_dyn_noisy(:,i) = (H\(B*u_data(:,i) - C)) + .01*randn(nq,1);
%   %qdd_data(:,i) = (H\(B*u_data(:,i) - C)) + .01*randn(nq,1);
% end

%%   Step 1: Extract lumped-parameters
switch options.method
  case 'dynamics'
    [H,C,B] = manipulatorDynamics(pobj,qt,qd);
    err = H*qdd + C - B*u;
    [chi,W,rho,lp,beta] = identifiableParameters(err,p);
    W = getmsspoly(W);
    rho = getmsspoly(rho);
    
    %%% Energy method
    % As described on pp. 306 - 308 of [1]. Given $r$ sets of data for $q(t)$,
    % $\dot{q(t)}$, and $u(t)$, solve
    %
    % $$ Y(\Gamma,\dot{q}) = W(q,\dot{q}) \chi + \rho $$
    %
    % where
    %
    % $$\chi = (Fv, K_b)'$$
    %
    % $$y_i = \int_{t_{a(i)}}^{t_{b(i)}} u'B'\dot{q}dt$$
    %
    % and where $W \in \bf{R}^{r \times b}$ is a block matrix with blocks defined by
    %
    % $$W_{i2} = \Delta h(i) = h[(q,\dot{q})_{b(i)}] - h[(q,\dot{q})_{a(i)}]$$
    %
    % $$W_{i1} = \Delta f_c(i) = \int_{t_{a(i)}}^{t_{b(i)}} (\dot{q} \circ \dot{q})' dt $$
  case {'energy','power'}
    E = pobj.getTotalEnergy(qt,qd);
    [~,~,B] = manipulatorDynamics(pobj,qt,qd);
    [Kb,h,rho,lp,beta] = identifiableParameters(E,p);
    Fv = getFv(pobj);
    chi = [Fv; Kb];
    beta = blkdiag(eye(length(Fv)),beta);
    lp = [Fv;lp];
    
    h = getmsspoly(h);
    rho = getmsspoly(rho);
    
end

np = length(p_orig);
nlp = length(lp);
nchi = length(chi);
lp_orig = double(subs(lp,p,p_orig));
chi_orig = double(subs(chi,p,p_orig));
% p_guess = p_orig + 2e-1*p_orig.*(2*(rand(size(p_orig))-0.5));
p_guess = p_orig + 2e-1*p_orig;
lp_guess = double(subs(lp,p,p_guess));
chi_guess = double(subs(chi,p,p_guess));
lumped_params = msspoly('lp',nlp);
% now err=M*lp+Mb and lperr=M*lumped_params+Mb;

%%   Step 2: Least-squares estimation
nt_total = length(t_data);
switch options.method
  %%% Dynamics method
  case 'dynamics'
    %%%
    % Compute _W_
    s_data = sin(q_data);
    c_data = cos(q_data);
    W_col = reshape(W',[],1);
    W_raw = full(msubs(W_col,[q;s;c;qd;qdd;u],[q_data;s_data;c_data;qd_data;qdd_data;u_data])');
    
    %%%
    % Compute $\rho$
    rho_raw = full(msubs(rho,[q;s;c;qd;qdd;u],[q_data;s_data;c_data;qd_data;qdd_data;u_data])');
    
    %%%
    % Apply parallel filtering
    %%%
    nt = floor((1/options.Fdecimate)/Ts);
    W_data = reshape(decimateMultiCol(W_raw,nt,8)',nchi,[])';
    % Compute number of rows in _W_ and _Y_
    r = size(W_data,1);
    
    %%%
    % Compute _Y_
    Y_data = zeros(size(W_data,1),1);
    
    rho_data = reshape(decimateMultiCol(rho_raw,nt,8)',r,1);
    
    %%%
    % Solve OLS
    chi_est = W_data\(Y_data-rho_data);
    
%     [chi_est,W_data,rho_data] = obj.lumpedParameterEstimation(t_data,q_data,qd_data,qdd_data,u_data,W,rho,nchi);
    % lp_est
    
    %%% Energy Method
  case 'energy'
    %%%
    % Split data into intervals of length nt
    nt_total = length(t_data);
    nt = options.timesteps_per_interval;
    r = floor(nt_total/nt)
    nt_total = nt*r+1;
    t_data = t_data(:,1:nt_total);
    q_data = q_data(:,1:nt_total);
    qd_data = qd_data(:,1:nt_total);
    qdd_data = qdd_data(:,1:nt_total);
    u_data = u_data(:,1:nt_total);
    t_lf = t_data(1:nt:end)';
%     q_lf = decimateMultiCol(q_data',nt,12)';
%     qd_lf = decimateMultiCol(qd_data',nt,12)';
%     u_lf = decimateMultiCol(u_data',nt,12)';
    q_lf = downsample(q_data',nt)';
    qd_lf = downsample(qd_data',nt)';
    u_lf = downsample(u_data',nt)';
    a = 1:nt:nt_total;
    b = nt:nt:nt_total;
    s_data = sin(q_data);
    c_data = cos(q_data);
    s_lf = sin(q_lf);
    c_lf = cos(q_lf);
    
    %%%
    % Compute
    %
    % $$Y_i = \int_{t_{a(i)}}^{t_{b(i)}} u'B'\dot{q}dt$$
    %
    Gamma_data = (u_data'*B')';
    input_power = dot(Gamma_data,qd_data,1);
%     input_power_reshaped = reshape(input_power,nt,r)';  % [r x nt]
    input_power_reshaped = intervalReshape(input_power,nt,r);
    Y_data = trapz(input_power_reshaped,1)'*Ts;             % [r x 1]
    
    %%%
    % Compute
    %
    % $$W_{i2} = \Delta f_c(i) = \int_{t_{a(i)}}^{t_{b(i)}} (\dot{q} \circ \dot{q})' dt $$
    qd_reshaped = intervalReshape(qd_data,nt,r);           % [(nt+1) x r*nq]
    delta_f_c_reshaped = trapz(qd_reshaped.^2,1)*Ts;     %  [1  x r*nq]
    delta_f_c = reshape(delta_f_c_reshaped,r,nq);     % [r x nq]
    
    %%%
    % Compute
    %
    % $$W_{i1} = \Delta h(i) = h[(q,\dot{q})_{b(i)}] - h[(q,\dot{q})_{a(i)}]$$
    % The naive way below give huge errors
%     h_a = msubs(h',[q;s;c;qd;qdd;u],[q_data(:,a);s_data(:,a);c_data(:,a);...
%                                      qd_data(:,a);qdd_data(:,a);u_data(:,a)])';
%     h_b = msubs(h',[q;s;c;qd;qdd;u],[q_data(:,b);s_data(:,b);c_data(:,b);...
%                                      qd_data(:,b);qdd_data(:,b);u_data(:,b)])';
     
    h_data = msubs(h',[q;s;c;qd],[q_lf;s_lf;c_lf;qd_lf])';
    W_data = [delta_f_c,diff(h_data,1,1)];
%     W_data = [diff(h_data,1,1)];
    
    %%%
    % Compute $\rho$
    if deg(rho) ~= 0 || double(rho) ~= 0
%       drho_data = msubs(rho,[q;s;c;qd;qdd;u],[q_data;s_data;c_data;qd_data;qdd_data;u_data]);
%       drho_data_reshaped = intervalReshape(drho_data,nt,r);
%       rho_data = trapz(drho_data_reshaped,1)'*Ts;
      rho_data = diff(msubs(rho,[q;s;c;qd],[q_lf;s_lf;c_lf;qd_lf])',1,1);
    else
      rho_data = zeros(r,1);
    end
    
    %%%
    % Solve OLS
    chi_est = W_data\(Y_data-rho_data);
    
    %%% Power Method
  case 'power'
    s_data = sin(q_data);
    c_data = cos(q_data);
    
    %%%
    % Compute $\dot{q} \circ \dot{q}$
    qd_sq_data = qd_data.^2;
    
    %%%
    % Compute _h_
    h_data = msubs(h',[q;s;c;qd],[q_data;s_data;c_data;qd_data]);
    dh_data = gradient(h_data,Ts);    
    
    W_raw = [qd_sq_data',dh_data'];
%     W_raw = [dh_data'];
    
    %%%
    % Compute
    %
    % $$Y_i = u'B'\dot{q}$$
    %
    Gamma_data = (u_data'*B')';
    Y_raw = dot(Gamma_data,qd_data,1)';
    
    nt_total = length(t_data);
    nt = floor((1/options.Fdecimate)/Ts);
    t_lf = t_data(1:nt:end);
    W_data = decimateMultiCol(W_raw,nt,8);
    Y_data = decimateMultiCol(Y_raw,nt,8);
    r = length(Y_data)
    
    %%%
    % Compute $\rho$
    if deg(rho) ~= 0 || double(rho) ~= 0
      rho_raw = gradient(msubs(rho,[q;s;c;qd;qdd;u],[q_data;s_data;c_data;qd_data;qdd_data;u_data]),Ts);
      rho_data = decimateMultiCol(full(rho_raw'),nt,8);
    else
      rho_data = zeros(r,1);
    end
    
    %%%
    % Solve OLS
    chi_est = W_data\(Y_data-rho_data);
end
err_orig = W_data*chi_orig + rho_data - Y_data;

err_est = W_data*chi_est + rho_data - Y_data;
sqerr_orig = sum(err_orig'*err_orig);
sqerr_est = sum(err_est'*err_est);

% Compute estimate of monomial values from estimate of identifiable
% parameters. Really we want to find p such that
%   beta*lp(p) = chiHat

R_betaT = orth(full(beta)');
N_beta = null(full(beta)');
lp_est = lp_guess + R_betaT*((beta*R_betaT)\(chi_est - beta*lp_guess));

chi_error = 100*abs((double(chi_est) - chi_orig)./chi_orig);
chi_guess_error = 100*abs((double(chi_guess) - chi_orig)./chi_orig);
sigma_rho_square = err_est'*err_est/(r-nchi);
C_chi_est = sigma_rho_square*inv(W_data'*W_data);
sigma_chi_est = sqrt(diag(C_chi_est));
rsd_chi_est = sigma_chi_est./abs(chi_est);
rsd_factor = rsd_chi_est./min(rsd_chi_est);

lp_error = 100*abs((double(lp_est) - lp_orig)./lp_orig);
lp_guess_error = 100*abs((double(lp_guess) - lp_orig)./lp_orig);
% sigma = 0.1*p_orig;
% p_guess = p_orig+sigma.*randn(size(p_orig));
phat = obj.lumpedToOriginalParams(p,p_guess,np,lp,lp_est,nlp);
p_error = 100*abs((double(phat) - p_orig)./p_orig);
p_guess_error = 100*abs((double(p_guess) - p_orig)./p_orig);
% print out results  (todo: make this an option?)
coords = getCoordinateNames(getParamFrame(obj));
fprintf('\nIdentifiable parameter estimation results:\n\n');
fprintf('  Param  \tActual  \tGuess   \tEstimated\tError (guess)\tError (estimate)\tRSD/min(RSD)\n');
fprintf('  -----  \t------  \t--------\t---------\t-------------\t----------------\t------------\n');
for i=1:nchi
  fprintf('%7d  \t%8.2f\t%8.2f\t%8.2f\t%8.2f%%\t%8.2f%%\t%8.2f\n',i,chi_orig(i),chi_guess(i),chi_est(i),chi_guess_error(i),chi_error(i),rsd_factor(i));
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

end

function reshaped = intervalReshape(mat,nt,r)
rows = size(mat,1);
tmp = reshape(mat(:,1:end-1),nt*rows,r);        % [nt*rows x r]
tmp2 = [tmp;[tmp(1:rows,2:r),mat(:,end)]];    % [(nt+1)*rows x r]
tmp3 = reshape(tmp2,rows,(nt+1)*r);             % [rows x (nt+1)*r]
reshaped = reshape(tmp3',nt+1,r*rows);          % [nt+1 x r*rows]
end
%% References
% [1] W. Khalil and E. Dombre, Modeling, identification and control of robots. Butterworth-Heinemann, 2004.
