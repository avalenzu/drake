function [lp_est,M_data,Mb_data] = lumpedParameterEstimation(obj,t_data,q_data,qd_data,qdd_data,u_data,M,Mb,nlp,lambda)
if ~exist('lambda','var'), lambda = 0; end;
ndata = length(t_data);
nq = obj.num_q;
nu = obj.num_u;

q=msspoly('q',nq);
s=msspoly('s',nq);
c=msspoly('c',nq);
qd=msspoly('qd',nq);
qdd=msspoly('qdd',nq);
u=msspoly('u',nu);

s_data = sin(q_data);
c_data = cos(q_data);

M_data = reshape(msubs(M(:),[q;s;c;qd;qdd;u],[q_data;s_data;c_data;qd_data;qdd_data;u_data])',nq*ndata,nlp);
Mb_data = reshape(msubs(Mb,[q;s;c;qd;qdd;u],[q_data;s_data;c_data;qd_data;qdd_data;u_data])',nq*ndata,1);

[~,S,V] = svds(M_data);
lp_est_svd = -V*1/(S.^2+lambda*eye(size(S)))*V'*M_data'*Mb_data;
if lambda == 0
  lp_est = -M_data\Mb_data;
else
  lp_est = (M_data'*M_data+lambda*eye(nlp))\(-M_data'*Mb_data);
end
% fprintf('  mldivide\tsvd      \n');
% fprintf('  --------\t---------\n');
% for i=1:length(lp_est)
%   fprintf('  %8.2f\t%8.2f\n',full(lp_est(i)),lp_est_svd(i));
% end
end
