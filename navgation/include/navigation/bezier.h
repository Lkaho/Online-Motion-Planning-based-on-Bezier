/*
This header file is used to provide some basic mathematic support for the
Bernstein-basis trajectory generation optimization problem. Includes: 1-:
Mapping matrix maps the coefficients of the Bernstein basis (ie. control points)
to Monomial basis. The mapping matrix range from order 3 to order 10 2-: Modulus
list of the Bernstein basis to a given order. That is, pre-compute the
constant-modulus (the 'n choose k' combinatorial) of the basis vector. To save
computation cost of frequently call this value.

The class should be initialized to a instance before the trajectory generator
called. Several initializer are provided, and the instance is initialized
according to the given order of the control points.
*/

#ifndef _BAZIER_H_
#define _BAZIER_H_

#include <Eigen/Dense>
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>

using namespace std;
using namespace Eigen;

class Bernstein {
 private:
  vector<MatrixXd> MQMList, MList;
  vector<MatrixXd> FMList;
  vector<VectorXd> CList, CvList, CaList, CjList;

  int _order_min,
      _order_max;     // The order of the polynomial in each segment, also the
                      // number of control points used in each segment
  double _min_order;  // The order to which we minimize.   1 -- velocity, 2 --
                      // acceleration, 3 -- jerk, 4 -- snap

 public:
  Bernstein() {}  // Empty constructor

  ~Bernstein() {}

  int setParam(int poly_order_min, int poly_order_max, double min_order);

  MatrixXd CholeskyDecomp(MatrixXd Q);  // return square root F of Q; Q = F' * F

  vector<MatrixXd> getM() {
    return MList;
  }
  vector<MatrixXd> getMQM() {
    return MQMList;
  }
  vector<MatrixXd> getFM() {
    return FMList;
  }
  vector<VectorXd> getC() {
    return CList;
  }
  vector<VectorXd> getC_v() {
    return CvList;
  }
  vector<VectorXd> getC_a() {
    return CaList;
  }
  vector<VectorXd> getC_j() {
    return CjList;
  }
};

#endif