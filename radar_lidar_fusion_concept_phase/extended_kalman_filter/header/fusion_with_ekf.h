#ifndef FUSION_WITH_EKF_H_
#define FUSION_WITH_EKF_H_

#include "../eigen_lib/Dense"
#include "../header/kalman_filter.h"
#include "../header/data.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;


/*
	https://thekalmanfilter.com/kalman-filter-explained-simply/#:~:text=What%20is%20the%20Kalman%20Filter,unobservable%20variable%20with%20greater%20accuracy.
	https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/

*/
class ExtendedKalmanFilter {

private:
	int N_;
	int lidar_N_;
	int radar_N_;
	double ax_;
	double ay_;
	bool initialized_;
	long long timestamp_;
	MatrixXd P_; // state covariance matrix
	MatrixXd F_; // state transition matrix
	MatrixXd Q_; // noise covariance matrix deal with uncertainity
	MatrixXd radar_R_; // measurement covariance matrix
	MatrixXd lidar_R_; // measurement covariance matrix
	MatrixXd lidar_H_; // state to measurement transition matrix
	KalmanFilter kalman_filter_;

public:
	ExtendedKalmanFilter();
	void updateNoiseCovarianceQ(const double dt);
	void processWithEKF(const Data& data);
	void initializeFirstMeasurement(const Data& data);
	void applyEKF(const Data& data);
	VectorXd getStateFromPrediction() const;
};

#endif
