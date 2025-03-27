/** --------------------------------------------------------------------------
 ** "THE BEER-WARE LICENSE" (Revision 42):
 ** <maggio.martina@gmail.com> wrote this file. As long as you retain this 
 ** notice you can do whatever you want with this stuff. If we meet some day, 
 ** and you think this stuff is worth it, you can buy me a beer in return.
 ** --------------------------------------------------------------------------
 **/

// Tyler Paine 16 July 2021 - Added method to set the weights with an initial
//                            estimate. 


#ifndef __RLS_H
#define __RLS_H

#include <armadillo>
using namespace arma;

namespace identification {

  // Interface for the identificator
  class identificator {

    protected:
      unsigned int _n_inputs;   // number of input values
      unsigned int _n_order;    // order per input value (how many past values are relevant)
      vec          _weights;    // identified weights 

    public:
      virtual ~identificator(){}
      virtual void step(vec inputs, double output) = 0;
      vec get_weights() { return _weights; }
      virtual bool set_weights(vec init_weights) = 0; 

  };

  /**** Recursive Least Square (rls) estimator algorithm
      * details about the algorithm can be found at
      * http://en.wikipedia.org/wiki/Recursive_least_squares_filter
      * the variable names are kept as similar as possible. */
  class rls : public identificator {

    private:
      double       _lambda;     // forgetting factor (algorithm parameter)
      unsigned int _n_rlsorder; // total recursive least square order
      mat          _P;          // matrix dimension _n_rlsorder x _n_rlsorder

    public:
      rls(unsigned int n_inputs, unsigned int n_order, double lambda = 0.5)
        : _lambda(lambda) {
          _n_inputs = n_inputs;
          _n_order = n_order;
          _n_rlsorder = _n_inputs * _n_order;
        // initialization of P matrix and weights
        _P = eye<mat>(_n_rlsorder+_n_inputs, _n_rlsorder+_n_inputs);
        _P = inv(0.0001*eye<mat>(_n_rlsorder+_n_inputs, _n_rlsorder+_n_inputs)) * _P;
        _weights = zeros<vec>(_n_rlsorder+_n_inputs);
      }

      // inputs: contains input data, size is [ _n_rlsorder * _n_inputs ]
      // data are in the form
      // in1(k), in1(k-1) ... in1(k-_n_order), in2(k), ... in2(k-_n_order) ...
      void step(vec inputs, double output) {
        double alpha = output - det(trans(_weights)*inputs);
        mat g = _P * inputs * 1/(_lambda + det(trans(inputs)*_P*inputs));
        _P = (1/_lambda) * _P - g*trans(inputs)*(1/_lambda)*_P;
        _weights = _weights + alpha*g;
      }

      // Set weights - perhaps an initial estimate
      //               returns true if successful
      bool set_weights(vec init_weights) {
	if ( init_weights.size() != _weights.size() ) {
	  return false;
	}
	_weights = init_weights;
	return true;
      }

  };

  /**** Normalized Least Mean Squares (lms) estimator algorithm
      * details about the algorithm can be found at
      * http://en.wikipedia.org/wiki/Least_mean_squares_filter */
  class nlms : public identificator {

    private:
      double       _mu;         // step size (algorithm parameter)
      unsigned int _n_lmsorder; // total least mean squares order

    public:
      nlms(unsigned int n_inputs, unsigned int n_order, double mu = 0.5)
        : _mu(mu) {
        _n_inputs = n_inputs;
        _n_order = n_order;
        _n_lmsorder = _n_inputs * _n_order;
        // initialization of weights
        _weights = zeros<vec>(_n_lmsorder+_n_inputs);
      }

      // inputs: contains input data, size is [ _n_rlsorder * _n_inputs ]
      // data are in the form
      // in1(k), in1(k-1) ... in1(k-_n_order), in2(k), ... in2(k-_n_order) ...
      void step(vec inputs, double output) {
        double e   = output - det(trans(_weights)*inputs);
        _weights   = _weights + (_mu*e*inputs)/(det(trans(inputs)*inputs));
      }

  };

}

#endif
