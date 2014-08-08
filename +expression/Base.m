classdef Base
  properties (SetAccess = private)
    input_frame;
    output_frame;
  end
  methods
    function obj = Base(input_frame,output_frame)
      obj.input_frame = input_frame;
      obj.output_frame = output_frame;
    end
    function [f,df] = eval(obj,x)
      sizecheck(x,[obj.input_frame.dim,1]);
      [f,df] = obj.fastEval(x);
    end

    function expr3 = compose(expr1, expr2)
      import expression.*
      expr3 = Composed(expr1,expr2);
    end
    
    function expr3 = plus(expr1, expr2)
      import expression.*
      expr_cat = concatenate(expr1,expr2);
      expr_sum = Sum(expr1.output_frame);
      expr3 = compose(expr_sum,expr_cat);
    end

    function expr2 = uminus(expr1)
      import expression.*
      expr2 = -1*expr1;
    end

    function expr3 = times(expr1,expr2)
      import expression.*
      if isnumeric(expr2)
        expr2 = Constant(expr1.output_frame,expr2);
      elseif isnumeric(expr1)
        expr3 = expr2*expr1;
        return;
      end
      expr_cat = concatenate(expr1,expr2);
      expr_prod = ElementwiseProduct(expr1.output_frame);
      expr3 = compose(expr_prod,expr_cat);
    end

    function expr3 = mtimes(expr1,expr2)
      if isnumeric(expr1) && isscalar(expr1)
        expr3 = expr1.*expr2;
      elseif isnumeric(expr2) && isscalar(expr2)
        expr3 = expr1.*expr2;
      else
        error('Drake:expression:Base:NotImplemented', ...
          'Matrix products of expressions are not yet imiplemented');
      end
    end

    function expr3 = power(expr1,expr2)
      import expression.*
      assert(isnumeric(expr2));
      assert(isscalar(expr2));
      expr3 = ConstantPower(expr1,expr2);
    end

    function expr3 = vertcat(expr1, expr2)
      import expression.*
      expr3 = VertCat(expr1,expr2);
    end

    function expr = concatenate(varargin)
      % expr3 = concatenate(expr1, expr2, ..., exprN, same_input) returns an expression
      % whos output frame is the concatenation of the output frames of expr1
      % and expr2.
      import expression.*
      if islogical(varargin{end}) 
        same_input = varargin{end};
        varargin(end) = [];
      else
        same_input = false;
      end
      n_expr = numel(varargin);
      switch n_expr
        case 1
          expr = varargin{1};
        case 2
          obj = varargin{1};
          other = varargin{2};
          expr = VertCat(obj,other,same_input);
        otherwise
          split = floor(n_expr/2);
          expr = concatenate(concatenate(varargin{1:split},same_input), ...
                             concatenate(varargin{split+1:end},same_input));
      end
    end

    function expr = duplicate(expr,n)
      tmp = repmat({expr},1,n);
      expr = concatenate(tmp{:});
    end

    function input_frame = getInputFrame(obj)
      input_frame = obj.input_frame;
    end

    function output_frame = getOutputFrame(obj)
      output_frame = obj.output_frame;
    end

    function obj = setInputFrame(obj,input_frame)
      obj.input_frame = input_frame;
    end

    function obj = setOutputFrame(obj,output_frame)
      obj.output_frame = output_frame;
    end
  end
  methods (Abstract)
    [f,df] = fastEval(obj,x);
  end
end
