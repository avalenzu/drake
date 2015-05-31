classdef QuatProductWithFixedQuat < drakeFunction.Linear
  % Computes the product of a quaternion with a fixed quaternion
  %
  %   q_out = q_fixed*q_in
  methods
    function obj = QuatProductWithFixedQuat(quat_fixed,fixed_first)
      if nargin < 2, fixed_first = true; end
      quat_fixed = normalizeVec(quat_fixed);
      if fixed_first
        negative_idx = [0, 1, 1, 1; ...
                        0, 0, 1, 0; ...
                        0, 0, 0, 1; ...
                        0, 1, 0, 0];
      else
        negative_idx = [0, 1, 1, 1; ...
                        0, 0, 0, 1; ...
                        0, 1, 0, 0; ...
                        0, 0, 1, 0];
      end
      negative_idx = logical(negative_idx);

      A = quat_fixed([1, 2, 3, 4; ...
                      2, 1, 4, 3; ...
                      3, 4, 1, 2; ...
                      4, 3, 2, 1]);
      A(negative_idx) = -A(negative_idx);

      frame = drakeFunction.frames.Quaternion();
      obj = obj@drakeFunction.Linear(frame, frame, A);
    end
  end
end
