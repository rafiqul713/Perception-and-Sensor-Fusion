#include "../header/evaluation_metrics.h"

void computeMeanSquareError(const vector<VectorXd> &ground_truth_list, const vector<VectorXd> &state_estimation_list)
{
	assert(ground_truth_list.size() == state_estimation_list.size());

	int total_item = state_estimation_list.size();
	std::cout << "Total number of element " << total_item << std::endl;
	VectorXd accumulated_error(4);
	VectorXd mean_square_error(4);
	accumulated_error << 0.0, 0.0, 0.0, 0.0;
	for (auto item = 0; item < total_item; ++item) {
		VectorXd error = (ground_truth_list[item] - state_estimation_list[item]);
		error = error.array() * error.array();
		accumulated_error += error;
	}
	
	mean_square_error = (accumulated_error / static_cast<double>(total_item));
	std::cout << "Mean Square Error " << std::endl;
	std::cout << "-----------------------------------------" << std::endl;
	std::cout << "x: " << mean_square_error(0) << std::endl;
	std::cout << "y: " << mean_square_error(1) << std::endl;
	std::cout << "vx: " << mean_square_error(2) << std::endl;
	std::cout << "vy: " << mean_square_error(3) << std::endl;
	std::cout << "-----------------------------------------" << std::endl;

}