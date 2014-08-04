classdef Projection < expression.Linear
  methods
    function obj = Projection(input_dim, output_dim)
      import expression.*
      import expression.frames.*
      assert(input_dim >= output_dim, ...
        'The input_dim (%d) is less than the output_dim (%d)', ...
        input_dim,output_dim)
      A = eye(output_dim,input_dim);
      obj = obj@expression.Linear(R(input_dim),R(output_dim),A);
    end
  end
end
