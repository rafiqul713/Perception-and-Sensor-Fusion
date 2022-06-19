#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "../eigen_lib/Dense"
#include "data.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
private:
	int N_;
	VectorXd X_; // state
	MatrixXd P_; // process covariance matrix
	MatrixXd F_; // state transition matrix
	MatrixXd Q_; // noise covariance matrix

public:
	KalmanFilter();
	void initializeFirstMeasurement(const int nin, const VectorXd& xin, const MatrixXd& Pin, const MatrixXd& Fin, const MatrixXd& Qin);
	void setCovarianceQ(const MatrixXd& q_matrix);
	void updateStateTransitionF(const double delta_t);
	VectorXd getStateFromPrediction() const;
	void predictNextState();
	void update(const VectorXd& z, const MatrixXd& H, const VectorXd& Hx, const MatrixXd& R);
};


#endif
