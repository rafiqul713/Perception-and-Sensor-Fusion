#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "../eigen_lib/Dense"
#include "../header/data.h"
#include "../header/utility.h"
#include "../header/fusion_with_ekf.h"
#include "../header/evaluation_metrics.h"

using namespace std;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

int main(int argc, char* argv[]) {

	string input_file_from_cmd = argv[1];
	string output_file_from_cmd = argv[2];
	string activated_sensor_type = argv[3];
	bool is_activate_radar = true;
	bool is_activate_lidar = true;

	sensor_activation(activated_sensor_type, is_activate_radar, is_activate_lidar);


	ifstream in_file(input_file_from_cmd.c_str(), ifstream::in);
	ofstream out_file(output_file_from_cmd.c_str(), ofstream::out);

	is_valid_file(in_file, input_file_from_cmd, out_file, output_file_from_cmd);

	vector<Data> radar_lidar_raw_data;
	vector<Data> all_ground_truth_datas;
	string sensor_type;
	string line;

	// Read data from the file and store it in the vector
	while (getline(in_file, line)) {

		istringstream read_line(line);
		Data sensor_data;
		Data truth_data;
		long long timestamp;

		read_line >> sensor_type;

		// read radar data
		if ((sensor_type.compare("R") == 0) && is_activate_radar) {
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
		else if (sensor_type.compare("L") == 0 && is_activate_lidar) {
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
			continue;
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
		all_ground_truth_datas.push_back(truth_data);
	}


	ExtendedKalmanFilter extended_kalman_filter;
	vector<VectorXd> state_estimation_list;
	vector<VectorXd> ground_truth_list;

	for (int data_indx = 0; data_indx < radar_lidar_raw_data.size(); ++data_indx) {

		extended_kalman_filter.applyEKF(radar_lidar_raw_data[data_indx]); // apply extended kalman filter

		VectorXd predicted_state = extended_kalman_filter.getStateFromPrediction();
		VectorXd measurement = radar_lidar_raw_data[data_indx].getStateFromMeasurement();

		VectorXd ground_truth_data_element = all_ground_truth_datas[data_indx].getRawMeasurementData();

		out_file << predicted_state(0) << "\t";
		out_file << predicted_state(1) << "\t";
		out_file << predicted_state(2) << "\t";
		out_file << predicted_state(3) << "\t";

		out_file << measurement(0) << "\t";
		out_file << measurement(1) << "\t";

		out_file << ground_truth_data_element(0) << "\t";
		out_file << ground_truth_data_element(1) << "\t";
		out_file << ground_truth_data_element(2) << "\t";
		out_file << ground_truth_data_element(3) << "\n";

		state_estimation_list.push_back(predicted_state);
		ground_truth_list.push_back(ground_truth_data_element);
	}

	// close files
	in_file.close();
	out_file.close();

	computeMeanSquareError(ground_truth_list, state_estimation_list);

	return 0;
}
