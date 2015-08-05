function [q3,dq3] = quatProduct(q1,q2)
% q3 = quatMultiply(q1,q2) Quaternion product of q1 and q
%
% @param q1
% @param q2
% @retval q3 - q1 q2
% @retval dq3 - Jacobian of q3 wrt [q1;q2]

% these are slow and will if they are not met then an error will occur
% immediately below
% sizecheck(q1,[4,1]);  
% sizecheck(q2,[4,1]);

w1 = q1(1);
w2 = q2(1);
v1 = q1(2:4);
v2 = q2(2:4);
crossv1v2 = [v1(2)*v2(3)-v1(3)*v2(2);v1(3)*v2(1)-v1(1)*v2(3);v1(1)*v2(2)-v1(2)*v2(1)];
q3 = [w1*w2 - v1(1)*v2(1)-v1(2)*v2(2)-v1(3)*v2(3); crossv1v2 + w1*v2 + w2*v1];

if nargout > 1
  inds = [1, 2, 3, 4; ...
          2, 1, 4, 3; ...
          3, 4, 1, 2; ...
          4, 3, 2, 1];
  signs = [ 1, -1, -1, -1, 1, -1, -1, -1; ...
            1,  1,  1, -1, 1,  1, -1,  1; ...
            1, -1,  1,  1, 1,  1,  1, -1; ...
            1,  1, -1,  1, 1, -1,  1,  1];
           
  dq3 = signs.*[q2(inds), q1(inds)];
end

end
