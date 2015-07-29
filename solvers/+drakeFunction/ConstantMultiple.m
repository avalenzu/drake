classdef ConstantMultiple < drakeFunction.Linear
  methods
    function obj = ConstantMultiple(varargin)
      % obj = ConstantMultiple(dim_input, value) returns a
      %   drakeFunction.Linear that multiplies its input elementwise by
      %   the given value. 
      % 
      % @param dim_input        -- Length of the input vector
      % @param value          -- Numeric scalar or vector. The output of the
      %                          returned function is value.*x
      %
      % @retval obj           -- drakeFunction.Linear object
      dim_input = varargin{1};
      if isscalar(value)
        A = value*eye(dim_input);
      else
        A = diag(value);
      end
      obj = obj@drakeFunction.Linear(dim_input,dim_input,A);
    end
  end
end
