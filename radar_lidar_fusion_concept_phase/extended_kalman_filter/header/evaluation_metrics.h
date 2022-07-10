#ifndef EVALUATION_METRICS_H_
#define EVALUATION_METRICS_H_

#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <iomanip>
#include <vector>
#include "data.h"
#include <assert.h>
#include "../eigen_lib/Dense"

using std::vector;
using Eigen::VectorXd;

void computeMeanSquareError(const vector<VectorXd> &ground_truth_list, const vector<VectorXd> &state_estimation_list);


#endif