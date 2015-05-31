function testQuatSmoothDistance
import drakeFunction.*
import drakeFunction.frame.*
import drakeFunction.geometry.*

quat_A = randn(4,1);
quat_A = quat_A/norm(quat_A);
quat_diff_fcn = QuatSmoothDistance();
quat_B = randn(4,1);
quat_B = quat_B./norm(quat_B);
[d,dd] = quat_diff_fcn.eval([quat_A; quat_B]);
[f,df] = geval(@(quat_A, quat_B) eval(quat_diff_fcn,[quat_A; quat_B]),quat_A, quat_B,struct('grad_method',{{'user','taylorvar'}},'tol',1e-6));

d = quat_diff_fcn.eval([quat_A; quat_A]);
valuecheck(d,0,1e-5);

d = quat_diff_fcn.eval([quat_A; -quat_A]);
valuecheck(d,0,1e-5);
end
