%% Create manipulator
S = warning('OFF','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('OFF','Drake:RigidBodyManipulator:UnsupportedJointLimits');
warning('OFF','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('OFF','Drake:RigidBody:SimplifiedCollisionGeometry');
% rbm = PlanarRigidBodyManipulator('urdf/simple_quarter_atlas.urdf',struct('floating',true));
% rbm_w_params = PlanarRigidBodyManipulator('urdf/simple_quarter_atlas_param.urdf',struct('floating',true));
rbm = PlanarRigidBodyManipulator('urdf/planar_quarter_atlas_param.urdf',struct('floating',true));
rbm_w_params = PlanarRigidBodyManipulator('urdf/planar_quarter_atlas_param.urdf',struct('floating',true));
foot = length(rbm.body);
foot_frame = RigidBodyFrame(foot,zeros(3,1),zeros(3,1),'foot_origin');
rbm = addFrame(rbm,foot_frame);
rbm = rbm.compile();
tsrbm = TimeSteppingRigidBodyManipulator(rbm,0.001);
tsrbm = addSensor(tsrbm,FullStateFeedbackSensor());
tsrbm = addSensor(tsrbm,ContactForceTorqueSensor(tsrbm,foot_frame));
tsrbm = compile(tsrbm);
nq = rbm.getNumDOF();
nu = rbm.getNumInputs();
nx = tsrbm.getOutputFrame().dim; % NOT 2*nq
nf = tsrbm.getOutputFrame().getFrameByNum(2).dim;
warning(S);

%% Construct msspoly variables
q = msspoly('q',nq);
s = msspoly('s',nq);
c = msspoly('c',nq);
q_trig = TrigPoly(q,s,c);
ft = msspoly('ft',nf);
p = rbm.getParamFrame().poly;

%% Set parameters to msspoly values
rbm_w_params = rbm_w_params.setParams(p);
rbm_w_params = rbm_w_params.compile();

%% Construct matrix of gravitational terms
G_msspoly = G(rbm_w_params.featherstone,q_trig);

%% Compute the identifiable parameters
rows_of_interest = 2:3;
[chi, W, rho,lp,beta] = identifiableParameters(G_msspoly(rows_of_interest),p);

%% Compute actual parameter values
nlp = length(lp);
nchi = length(chi);
np = length(p);

p_orig = double(rbm.getParams());
lp_orig = full(msubs(lp,p,double(p_orig)));
chi_orig = full(msubs(chi,p,double(p_orig)));

p_guess = p_orig + 5e-1*(2*rand(np,1)-1).*(p_orig+1);
lp_guess = double(subs(lp,p,p_guess));
chi_guess = double(subs(chi,p,p_guess));
%% Populate regression matrices
idx_validation = rand(1,size(q_data,2)) > 0.05;
idx_training = ~idx_validation;
q_data_validation = q_data(:,idx_validation);
q_data_training = q_data(:,idx_training);
ft_data_validation = ft_data(:,idx_validation);
ft_data_training = ft_data(:,idx_training);
n = size(q_data_training,2);
n_validation = sum(idx_validation);
s_data = sin(q_data_training);
c_data = cos(q_data_training);
W_col = reshape(W',[],1);
W_raw = full(msubs(getmsspoly(W_col),[q;s;c],[q_data_training;s_data;c_data])');
W_data = reshape(W_raw',nchi,[])';
rho_col = reshape(rho',[],1);
rho_data = full(msubs(getmsspoly(rho_col),[q;s;c],[q_data_training;s_data;c_data]));
Y_data = zeros(size(W,1),n);
for i = 1:n
  kinsol = doKinematics(rbm,q_data_training(:,i));
  [pos_foot,J_foot] = forwardKin(rbm,kinsol,rbm.findFrameId(foot_frame.name),[0;0;0],1);
  J_foot([2,4,6],:) = [];J_foot = J_foot(:,rows_of_interest);
  Y_data(:,i) = J_foot'*ft_data_training(:,i);
end

%% Estimate identifiable parameters
%idx_good = (Y_data - W_data*chi_orig).^2 < 200;
%idx_good = 1:n;
idx_good = ft_data_training(2,:) > 0;
Y_data = reshape(Y_data(:,idx_good),[],1);
rho_data = reshape(rho_data(:,idx_good),[],1);
chi_est = W_data(reshape(repmat(idx_good,size(W,1),1),[],1),:)\(Y_data-rho_data);

%% Estimate monomials
R_betaT = orth(full(beta)');
N_beta = null(full(beta)');
lp_est = lp_guess + R_betaT*((beta*R_betaT)\(chi_est - beta*lp_guess));

%% Display error
chi_error = double(chi_est) - chi_orig;
chi_guess_error = double(chi_guess) - chi_orig;
lp_error = double(lp_est) - lp_orig;
lp_guess_error = double(lp_guess) - lp_orig;
phat = rbm.lumpedToOriginalParams(p,p_guess,np,lp,lp_est,nlp);
p_error = double(phat) - p_orig;
p_guess_error = double(p_guess) - p_orig;
coords = getCoordinateNames(getParamFrame(rbm));
%%
fprintf('\nIdentifiable parameter estimation results:\n\n');
fprintf('  Param  \tActual  \tGuess   \tEstimated\tError (guess)\tError (estimate)\n');
fprintf('  -----  \t------  \t--------\t---------\t-------------\t----------------\n');
for i=1:nchi
  fprintf('%7d  \t%8.2f\t%8.2f\t%8.2f\t%8.2f\t%8.2f\n',i,chi_orig(i),chi_guess(i),chi_est(i),chi_guess_error(i),chi_error(i));
end
  
fprintf('\nMonomial estimation results:\n\n');
fprintf('  Param  \tActual  \tGuess   \tEstimated\tError (guess)\tError (estimate)\n');
fprintf('  -----  \t------  \t--------\t---------\t-------------\t----------------\n');
for i=1:nlp
  fprintf('%7d  \t%8.2f\t%8.2f\t%8.2f\t%8.2f\t%8.2f\n',i,lp_orig(i),lp_guess(i),lp_est(i),lp_guess_error(i),lp_error(i));
end
fprintf('\nParameter estimation results:\n\n');
fprintf('  Param  \tActual  \tGuess   \tEstimated\tError (guess)\tError (estimate)\n');
fprintf('  -----  \t------  \t--------\t---------\t-------------\t----------------\n');
for i=1:length(coords)
  fprintf('%7s  \t%8.2f\t%8.2f\t%8.2f\t%8.2f\t%8.2f\n',coords{i},p_orig(i),p_guess(i),phat(i),p_guess_error(i),p_error(i));
end

%% Validation
rbm_validation = rbm.setParams(phat);
rbm_guess = rbm.setParams(p_guess);
com_guess = zeros(2,n_validation);
com_actual = zeros(2,n_validation);
com_validation = zeros(2,n_validation);
for i = 1:n_validation
  kinsol = doKinematics(rbm,q_data_validation(:,i));
  com_actual(:,i) = rbm.getCOM(kinsol);
  kinsol = doKinematics(rbm_guess,q_data_validation(:,i));
  com_guess(:,i) = rbm_guess.getCOM(kinsol);
  kinsol = doKinematics(rbm_validation,q_data_validation(:,i));
  com_validation(:,i) = rbm_validation.getCOM(kinsol);
end

com_error_guess = sqrt(sum((com_actual - com_guess).^2,1));
com_error_validation = sqrt(sum((com_actual - com_validation).^2,1));

%%
fprintf('\nValidation results: Errors in COM location\n\n');
fprintf('                    Mean  \tStd. Dev \tMax\n');
fprintf('                    ----- \t-------- \t---\n');
fprintf('Initial Guess     | %8.2e\t%8.2e\t%8.2e\n',mean(com_error_guess),std(com_error_guess),max(com_error_guess));
fprintf('Estimated Params. | %8.2e\t%8.2e\t%8.2e\n',mean(com_error_validation),std(com_error_validation),max(com_error_validation));

