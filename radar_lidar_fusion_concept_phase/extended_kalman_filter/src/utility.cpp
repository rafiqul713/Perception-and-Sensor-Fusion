#include "../header/utility.h"

using namespace std;

VectorXd convert_cartesian_to_polar(const VectorXd& v) {

	const double THRESH = 0.0001;
	VectorXd polar_vec(3);

	const double px = v(0);
	const double py = v(1);
	const double vx = v(2);
	const double vy = v(3);

	const double rho = sqrt(px * px + py * py);
	const double phi = atan2(py, px); //accounts for atan2(0, 0)
	const double drho = (rho > THRESH) ? (px * vx + py * vy) / rho : 0.0;

	polar_vec << rho, phi, drho;
	return polar_vec;
}

VectorXd convert_polar_to_cartesian(const VectorXd& v) {

	VectorXd cartesian_vec(4);

	const double rho = v(0);
	const double phi = v(1);
	const double drho = v(2);

	const double px = rho * cos(phi);
	const double py = rho * sin(phi);
	const double vx = drho * cos(phi);
	const double vy = drho * sin(phi);

	cartesian_vec << px, py, vx, vy;
	return cartesian_vec;
}

MatrixXd calculate_jacobian(const VectorXd &v) {

	const double THRESH = 0.0001;
	MatrixXd H = MatrixXd::Zero(3, 4);

	const double px = v(0);
	const double py = v(1);
	const double vx = v(2);
	const double vy = v(3);

	const double d_squared = px * px + py * py;
	const double d = sqrt(d_squared);
	const double d_cubed = d_squared * d;

	if (d >= THRESH) {

		const double r11 = px / d;
		const double r12 = py / d;
		const double r21 = -py / d_squared;
		const double r22 = px / d_squared;
		const double r31 = py * (vx * py - vy * px) / d_cubed;
		const double r32 = px * (vy * px - vx * py) / d_cubed;

		H << r11, r12, 0.0, 0.0,
			r21, r22, 0.0, 0.0,
			r31, r31, r11, r12;
	}

	return H;
}

void is_valid_file(std::ifstream& in_file, std::string& in_name, std::ofstream& out_file, std::string& out_name) {

	if (!in_file.is_open()) {
		cerr << "Cannot open input file: " << in_name << endl;
		exit(EXIT_FAILURE);
	}

	if (!out_file.is_open()) {
		cerr << "Cannot open output file: " << out_name << endl;
		exit(EXIT_FAILURE);
	}
}
