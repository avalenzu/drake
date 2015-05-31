function testQuatRotationAboutFixedAxis
import drakeFunction.*
import drakeFunction.frames.*
import drakeFunction.geometry.*

p = normalizeVec(rand(3,1));
quat_rotation_about_fixed_axis_fun = QuatRotationAboutFixedAxis(p);
theta = randn;
[f,df] = geval(@(theta) eval(quat_rotation_about_fixed_axis_fun,theta), ...
               theta, struct('grad_method',{{'user','taylorvar'}},'tol',1e-6));

valuecheck(f,axis2quat([p; theta]),1e-5);
end

