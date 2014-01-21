function [x_data,u_data] = paramEstSyntheticData(options,x_data,u_data)
figure(7);cla;
oldpath = addpath(fullfile(pwd,'..'));


if nargin < 1, options = struct(); end;
if ~isfield(options,'t_final'), options.t_final = 100; end;
if ~isfield(options,'normalized_input_mag'), options.normalized_input_mag = 0.1; end;
if ~isfield(options,'noise_factor'), options.noise_factor = 0.1; end;

rng(1)


r = AcrobotPlant;
r_sim = RigidBodyManipulator('Acrobot.urdf');
v = AcrobotVisualizer(r);

true_parameters = getParams(r);
nq = r.getNumDOF();
%% test on swingup up data
%[utraj_swingup,xtraj_swingup] = swingUpTrajectory(r);

%% test on simulated data
Ts = 0.001; 
t_data = 0:Ts:options.t_final;
freq_u = 10;
% utraj = PPTrajectory(spline(downsample(t_data,10),(r.umin+r.umax)/2+options.normalized_input_mag*(r.umax-r.umin)*randn(r.getNumInputs(),length(downsample(t_data,10)))));
% utraj = PPTrajectory(foh(t_data,zeros(size(t_data))));
if nargin < 2
%   utraj = PPTrajectory(foh(t_data,(r.umin+r.umax)/2+options.normalized_input_mag*(r.umax-r.umin)/2*sin(freq_u*t_data)));
% utraj = PPTrajectory(foh(t_data,bsxfun(@plus,(r.umin+r.umax)/2,bsxfun(@times,options.normalized_input_mag*(r.umax-r.umin)/2,chirp(t_data,0,t_data(end),freq_u)))));
utraj = PPTrajectory(foh(t_data,bsxfun(@plus,(r.umin+r.umax)/2,bsxfun(@times,options.normalized_input_mag*(r.umax-r.umin)/2,[chirp(t_data,0,t_data(end),freq_u);fliplr(chirp(t_data,0,t_data(end),freq_u))]))));
utraj = setOutputFrame(utraj,r.getInputFrame());
sys = cascade(utraj,r);
xtraj = simulate(sys,utraj.tspan,.5*randn(4,1));


% Try parameter estimation without any noise
breaks=getBreaks(utraj); T0 = breaks(1); Tf = breaks(end);

u_data = eval(utraj,T0:Ts:Tf);
x_data = eval(xtraj,T0:Ts:Tf);
% just for debugging
% qdd_data = zeros(nq,size(x_data,2));
% for i=1:length(t_data)
%   [H,C,B] = manipulatorDynamics(r,x_data(1:nq,i),x_data(nq+(1:nq),i));
% %   qdd_data_dyn(:,i) = (H\(B*u_data(:,i) - C));
%   %qdd_data_dyn_noisy(:,i) = (H\(B*u_data(:,i) - C)) + .01*randn(nq,1);
%   qdd_data(:,i) = H\(B*u_data(:,i) - C);
% end
% x_data = [x_data;qdd_data];

colorOrder = get(gca, 'ColorOrder');
% figure(7); plot(t_data+Ts,qdd_data(1,:), 'Color', colorOrder(mod(length(get(gca, 'Children')), size(colorOrder, 1))+1, :));
hold on;
nt = numel(T0:Ts:Tf);
% data = iddata(x_data',u_data',Ts,'InputName',r.getInputFrame.coordinates,'OutputName',[r.getOutputFrame.coordinates;strcat(r.getOutputFrame.coordinates(end/2+1:end),'dot')]);
data = iddata(x_data',u_data',Ts,'InputName',r.getInputFrame.coordinates,'OutputName',r.getOutputFrame.coordinates);
end

% r = paramEstStatic(r,options);
% r = setParamFrame(r,CoordinateFrame('AcrobotParams',2,'p',...
%   {'Ic1','Ic2' }));
% r = setParamFrame(r,CoordinateFrame('AcrobotParams',4,'p',...
%   { 'm1','m2','lc1','lc2'}));

% data = iddata(x_data',u_data',Ts,'InputName',r.getInputFrame.coordinates,'OutputName',[r.getOutputFrame.coordinates;strcat(r.getOutputFrame.coordinates(end/2+1:end),'dot')]);
data = iddata(x_data',u_data',Ts,'InputName',r.getInputFrame.coordinates,'OutputName',r.getOutputFrame.coordinates);
estimated_parameters = parameterEstimation(r,data,options);

% Try parameter estimation with noise

sigma_u = options.noise_factor*rms(u_data,2);
sigma_x = options.noise_factor*rms(x_data,2);
x_noise = bsxfun(@times,sigma_x,randn(size(x_data)));
u_noise = bsxfun(@times,sigma_u,randn(size(u_data)));
colorOrder = get(gca, 'ColorOrder');
% figure(7); plot(t_data+Ts,x_data(2*nq+1,:)+x_noise(2*nq+1,:), 'Color', colorOrder(mod(length(get(gca, 'Children')), size(colorOrder, 1))+1, :));
% noisy_data = iddata(x_data'+x_noise',u_data'+u_noise',Ts,'InputName',r.getInputFrame.coordinates,'OutputName',[r.getOutputFrame.coordinates;strcat(r.getOutputFrame.coordinates(end/2+1:end),'dot')]);
noisy_data = iddata(x_data'+x_noise',u_data'+u_noise',Ts,'InputName',r.getInputFrame.coordinates,'OutputName',r.getOutputFrame.coordinates);

[estimated_parameters,guessed_parameters] = parameterEstimation(r,noisy_data,options);

% Try parameter estimation with noise and known delay
r_est = setParams(r,estimated_parameters);
r_guess = setParams(r,guessed_parameters);

path(oldpath);
