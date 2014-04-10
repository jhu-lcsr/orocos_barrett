/***********************************************************************

  Copyright 2011 Carnegie Mellon University
Author: Mike Vande Weghe <vandeweg@cmu.edu>

This file is part of owd.

owd is free software; you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation; either version 3 of the License, or (at your
option) any later version.

owd is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ***********************************************************************/
#ifndef BUTTERWORTH_H
#define BUTTERWORTH_H


#include <deque>
#include <oro_barrett_hw/butterworth_solver.h>
#include <stdio.h>
#include <stdlib.h>

/// Creates a butterworth filter with the specified order and 
/// cutoff frequency.  Defaults to lowpass, but can also be created
/// with types BUTTER_HIGHPASS, BUTTER_BANDPASS, or BUTTER_BANDSTOP.
/// In the case of the pass or stop filters, the higher frequency is
/// passed in as cutoff2.

template<class value_t> class Butterworth {
public:
  Butterworth(unsigned int _order,
              double cutoff,
              int filter_type=BUTTER_LOWPASS,
              double cutoff2 = 0) 
    : order(_order) 
  {
    A=new double[order+1];
    B=new double[order+1];

    int err = butterworth_solver(filter_type, order, 500, 
                                 cutoff, cutoff2,
                                 A, B);
    if (err) {
      throw err;
    }
  }

  ~Butterworth() {
    delete[] A;
    delete[] B;
  }

  value_t eval(value_t current) {
    if (X.size() != order) {
      // initialize with all current values
      X.resize(order,current);
      Y.resize(order,current);
    } 

    value_t output = B[0] * current;

    for (unsigned int i=0; i<order; ++i) {
      output += B[i+1]*X[i] - A[i+1]*Y[i];
    }

    X.pop_back();
    Y.pop_back();
    X.push_front(current);
    Y.push_front(output);
    return output;
  }

  inline void reset() {
    X.resize(0);
    Y.resize(0);
  }
protected:
  unsigned int order;
  double *A;
  double *B;
  std::deque<value_t> X;
  std::deque<value_t> Y;


};

#endif // BUTTERWORTH_H
