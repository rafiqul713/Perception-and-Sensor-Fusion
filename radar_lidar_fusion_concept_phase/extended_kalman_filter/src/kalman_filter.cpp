#include "../header/kalman_filter.h"

KalmanFilter::KalmanFilter() {

}

// initialize first measurement
void KalmanFilter::initializeFirstMeasurement(
	const int N_init, const VectorXd& X_init, const MatrixXd& P_init, const MatrixXd& F_init, const MatrixXd& Q_init) {
	this->N_ = N_init;
	this->X_ = X_init;
	this->P_ = P_init;
	this->F_ = F_init;
	this->Q_ = Q_init;
}

void KalmanFilter::setCovarianceQ(const MatrixXd& Qin) {
	this->Q_ = Qin;
}

void KalmanFilter::updateStateTransitionF(const double delta_t) {
	this->F_(0, 2) = delta_t;
	this->F_(1, 3) = delta_t;
}

VectorXd KalmanFilter::getStateFromPrediction() const {
	return this->X_;
}

void KalmanFilter::predictNextState() {
	// predict the state using the state transition matrix
	this->X_ = this->F_ * this->X_;
	// update the covariance matrix using state transition matrix and the process noise
	this->P_ = this->F_ * this->P_ * this->F_.transpose() + this->Q_;
}

void KalmanFilter::update(const VectorXd& z, const MatrixXd& H, const VectorXd& Hx, const MatrixXd& R) {

	const MatrixXd H_transpose = H.transpose();
	const MatrixXd PH_transpose = this->P_ * H_transpose;
	const MatrixXd S = H * PH_transpose + R;
	const MatrixXd K = PH_transpose * S.inverse();

	VectorXd y = z - Hx;

	// normalize angle for radar measurement by adjusting theta if it is outside of [-PI, PI] 
	if (y.size() == 3)
	{
		if (y(1) > M_PI)
		{
			y(1) = y(1) - 2 * M_PI;
		}
		else if (y(1) < -M_PI)
		{
			y(1) = y(1) + 2 * M_PI;
		}
		else
		{
			// MISRA
		}
	}


	// update state
	this->X_ = this->X_ + (K * y);

	MatrixXd I = MatrixXd::Identity(this->N_, this->N_); // identity matrix
	// update convariance matrix
	this->P_ = (I - K * H) * this->P_;
}
