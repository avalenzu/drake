function stormerVerletResidualsTest()
  dim_r = 3;
  N = 10;
  fcn = drakeFunction.integration.StormerVerletResiduals(dim_r, N);
  options.grad_method = {'user', 'numerical'};
  options.tol = 1e-7;
  drakeFunction.test.testGradients(fcn, options)
end

