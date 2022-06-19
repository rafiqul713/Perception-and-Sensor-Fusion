#include "../header/fusion_with_ekf.h"

ExtendedKalmanFilter::ExtendedKalmanFilter() {

	this->N_ = 4;
	this->lidar_N_ = 2;
	this->radar_N_ = 3;
	this->ax_ = 9.0;
	this->ay_ = 9.0;

	this->initialized_ = false;

	this->lidar_R_ = MatrixXd(this->lidar_N_, this->lidar_N_);
	this->radar_R_ = MatrixXd(this->radar_N_, this->radar_N_);
	this->lidar_H_ = MatrixXd(this->lidar_N_, this->N_);

	this->P_ = MatrixXd(this->N_, this->N_);
	this->F_ = MatrixXd::Identity(this->N_, this->N_);
	this->Q_ = MatrixXd::Zero(this->N_, this->N_);

	this->lidar_R_ << 0.0225, 0.0,
		0.0, 0.0225;

	this->radar_R_ << 0.09, 0.0, 0.0,
		0.0, 0.0009, 0,
		0.0, 0.0, 0.09;

	this->lidar_H_ << 1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0;

	this->P_ << 1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1000.0, 0.0,
		0.0, 0.0, 0.0, 1000.0;
}

void ExtendedKalmanFilter::updateNoiseCovarianceQ(const double delta_t) {

	const double delta_t2 = delta_t * delta_t;
	const double delta_t3 = delta_t * delta_t2;
	const double delta_t4 = delta_t * delta_t3;

	const double r11 = delta_t4 * this->ax_ / 4;
	const double r13 = delta_t3 * this->ax_ / 2;
	const double r22 = delta_t4 * this->ay_ / 4;
	const double r24 = delta_t3 * this->ay_ / 2;
	const double r31 = delta_t3 * this->ax_ / 2;
	const double r33 = delta_t2 * this->ax_;
	const double r42 = delta_t3 * this->ay_ / 2;
	const double r44 = delta_t2 * this->ay_;

	this->Q_ << r11, 0.0, r13, 0.0,
		0.0, r22, 0.0, r24,
		r31, 0.0, r33, 0.0,
		0.0, r42, 0.0, r44;

	this->kalman_filter_.setCovarianceQ(Q_);

}

void ExtendedKalmanFilter::initializeFirstMeasurement(const Data& data) {

	this->timestamp_ = data.get_timestamp();
	VectorXd x = data.getStateFromMeasurement();
	this->kalman_filter_.initializeFirstMeasurement(this->N_, x, this->P_, this->F_, this->Q_);
	this->initialized_ = true;
}

void ExtendedKalmanFilter::processWithEKF(const Data& data) {

	// prediction
	double current_timestamp = data.get_timestamp();
	double previous_timestamp = this->timestamp_;
	double timestamp_diff = (current_timestamp - previous_timestamp) / 1.e6;
	this->timestamp_ = current_timestamp;
	this->updateNoiseCovarianceQ(timestamp_diff);
	this->kalman_filter_.updateStateTransitionF(timestamp_diff);
	this->kalman_filter_.predictNextState();

	// update
	const VectorXd z = data.getRawMeasurementData();
	const VectorXd x = this->kalman_filter_.getStateFromPrediction();

	VectorXd Hx;
	MatrixXd R; // measurement covariance matrix
	MatrixXd H; // state to transition

	// non linear 
	if (data.get_type() == SourceOfData::RADAR) {
		VectorXd s = data.getStateFromMeasurement();
		H = calculate_jacobian(s); // Calculate the Jacobian matrix about the current predicted state
		Hx = convert_cartesian_to_polar(x);
		R = this->radar_R_;

	}
	// linear
	else if (data.get_type() == SourceOfData::LIDAR) {
		H = this->lidar_H_;
		Hx = this->lidar_H_ * x;
		R = this->lidar_R_;
	}

	else
	{
		// MISRA 
	}

	this->kalman_filter_.update(z, H, Hx, R);
}

void ExtendedKalmanFilter::applyEKF(const Data& data) {
	if (this->initialized_)
	{
		this->processWithEKF(data);
	}
	else { // measurement at the first time
		this->initializeFirstMeasurement(data);
	}
}

VectorXd ExtendedKalmanFilter::getStateFromPrediction() const {
	return this->kalman_filter_.getStateFromPrediction();
}
