#include "mex.h"
#include <iostream>
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

/*
 * mex-interface for clearing stored data in collision models
 *
 * MATLAB signature:
 *
 *    clearStoredContactsmex( mex_model_ptr );
 */

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {

  if (nrhs < 1) {
    mexErrMsgIdAndTxt("Drake:collisionDetectmex:NotEnoughInputs","Usage clearStoredContacts(model_ptr)");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  // See whether we're using multiple contacts or not
  bool clear_margin_model = true;
  if (nrhs >= 2) {
    if (!mxIsLogicalScalar(prhs[1])) {
      mexErrMsgIdAndTxt("Drake:clearStoredContacts:InvalidInput", 
                        "The second argument must be a logical scalar.");
    }
    clear_margin_model = mxIsLogicalScalarTrue(prhs[1]);
  }

  model->clearStoredContacts(clear_margin_model);
}
