function testGradients( fcn, geval_options ) %NOTEST
%   Helper function for testing drakeFunction gradients
  if nargin < 2, geval_options = struct(); end
  if ~isfield(geval_options, 'grad_method')
    geval_options.grad_method = {'user', 'numerical', 'taylorvar'};
  end
  x = rand(fcn.dim_input, 1);

  [f,df] = geval(@(x) eval(fcn,x), x, geval_options);
end

