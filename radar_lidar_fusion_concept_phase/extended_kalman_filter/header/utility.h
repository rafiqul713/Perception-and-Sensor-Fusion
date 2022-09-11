#ifndef UTILITY_H_
#define UTILITY_H

#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <iomanip>
#include <vector>
#include <ctime>
#include "data.h"
#include "../eigen_lib/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

VectorXd convert_cartesian_to_polar(const VectorXd& v);
VectorXd convert_polar_to_cartesian(const VectorXd& v);
MatrixXd calculate_jacobian(const VectorXd &v);
void is_valid_file(std::ifstream& in_file, std::string& in_name, std::ofstream& out_file, std::string& out_name);
clock_t getCPUTime();

#endif
