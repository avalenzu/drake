function u = findStaticTorques(r,q)
[nq,n] = size(q);
nu = r.getNumInputs();
u = zeros(nu,n);
for i = 1:n
  [~,C,B] = manipulatorDynamics(r,q(:,i),zeros(nq,1));
  u(:,i) = B\C;
end
end