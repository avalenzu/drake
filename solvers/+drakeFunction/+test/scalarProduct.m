function scalarProduct()
  dim = 7;
  fcn = drakeFunction.ScalarProduct(dim);
  drakeFunction.test.testGradients(fcn);
end
