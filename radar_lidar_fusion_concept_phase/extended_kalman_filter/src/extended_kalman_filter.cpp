#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "../eigen_lib/Dense"
#include "../header/data.h"
#include "../header/utility.h"
#include "../header/fusion_with_ekf.h"

using namespace std;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

int main(int argc, char* argv[]) {

	string input_file_from_cmd = argv[1];
	string output_file_from_cmd = argv[2];
	ifstream in_file(input_file_from_cmd.c_str(), ifstream::in);
	ofstream out_file(output_file_from_cmd.c_str(), ofstream::out);

	is_valid_file(in_file, input_file_from_cmd, out_file, output_file_from_cmd);

	vector<Data> radar_lidar_raw_data;
	vector<Data> all_truth_data;
	string sensor_type;
	string line;


	clock_t start = getCPUTime();

	// Read data from the file and store it in the vector
	while (getline(in_file, line)) {

		istringstream read_line(line);
		Data sensor_data;
		Data truth_data;
		long long timestamp;

		read_line >> sensor_type;

		// read radar data
		if (sensor_type.compare("R") == 0) {
			double radar_meas_rho, radar_meas_phi, radar_meas_drho;

			read_line >> radar_meas_rho;
			read_line >> radar_meas_phi;
			read_line >> radar_meas_drho;
			read_line >> timestamp;

			VectorXd radar_vec(3);
			radar_vec << radar_meas_rho, radar_meas_phi, radar_meas_drho;

			sensor_data.set(timestamp, SourceOfData::RADAR, radar_vec);
		}
		// read lidar data
		else if (sensor_type.compare("L") == 0) {
			double lidar_meas_x, lidar_meas_y;

			read_line >> lidar_meas_x;
			read_line >> lidar_meas_y;
			read_line >> timestamp;

			VectorXd lidar_vec(2);
			lidar_vec << lidar_meas_x, lidar_meas_y;

			sensor_data.set(timestamp, SourceOfData::LIDAR, lidar_vec);

		}

		else
		{
			// MISRA rules
		}

		double gt_x, gt_y, gt_vx, gt_vy;

		read_line >> gt_x;
		read_line >> gt_y;
		read_line >> gt_vx;
		read_line >> gt_vy;

		VectorXd truth_vec(4);
		truth_vec << gt_x, gt_y, gt_vx, gt_vy;
		truth_data.set(timestamp, SourceOfData::STATE, truth_vec);

		radar_lidar_raw_data.push_back(sensor_data);
		all_truth_data.push_back(truth_data);
	}


	ExtendedKalmanFilter extended_kalman_filter;
	vector<VectorXd> state_estimation;
	vector<VectorXd> ground_truths;

	for (int data_indx = 0; data_indx < radar_lidar_raw_data.size(); ++data_indx) {

		extended_kalman_filter.applyEKF(radar_lidar_raw_data[data_indx]); // apply extended kalman filter

		VectorXd prediction = extended_kalman_filter.getStateFromPrediction();
		VectorXd measurement = radar_lidar_raw_data[data_indx].getStateFromMeasurement();

		VectorXd truth = all_truth_data[data_indx].getRawMeasurementData();

		out_file << prediction(0) << "\t";
		out_file << prediction(1) << "\t";
		out_file << prediction(2) << "\t";
		out_file << prediction(3) << "\t";

		out_file << measurement(0) << "\t";
		out_file << measurement(1) << "\t";

		out_file << truth(0) << "\t";
		out_file << truth(1) << "\t";
		out_file << truth(2) << "\t";
		out_file << truth(3) << "\n";

		state_estimation.push_back(prediction);
		ground_truths.push_back(truth);
	}

	// close files
	in_file.close();
	out_file.close();

	clock_t end = getCPUTime();
	std::cout<<"Total time taken (including I/O operation) "<< (1.0*(end - start) / CLOCKS_PER_SEC)<<" seconds";

	return 0;
}
