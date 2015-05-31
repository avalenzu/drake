function testQuatProductWithFixedQuat
import drakeFunction.*
import drakeFunction.frames.*
import drakeFunction.geometry.*

quat_fixed = normalizeVec(randn(4,1));

test_fun_A = QuatProductWithFixedQuat(quat_fixed, true);
test_fun_B = QuatProductWithFixedQuat(quat_fixed, false);

quat = normalizeVec(randn(4,1));

[f,df] = geval(@(quat) eval(test_fun_A,quat), ...
               quat, struct('grad_method',{{'user','taylorvar'}},'tol',1e-6));

[f,df] = geval(@(quat) eval(test_fun_B,quat), ...
               quat, struct('grad_method',{{'user','taylorvar'}},'tol',1e-6));

valuecheck(test_fun_A(quat), quatProduct(quat_fixed, quat), 1e-6);
valuecheck(test_fun_B(quat), quatProduct(quat, quat_fixed), 1e-6);
