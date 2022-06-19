#include "../header/data.h"

using namespace std;

Data::Data() {
	this->initialized = false;
}

Data::Data(const long long timestamp, const SourceOfData data_type, const VectorXd raw) {
	this->set(timestamp, data_type, raw);
}

void Data::set(const long timestamp, const SourceOfData data_type, const VectorXd raw) {
	this->timestamp = timestamp;
	this->data_type = data_type;
	this->raw = raw;
	this->initialized = true;
}

VectorXd Data::getRawMeasurementData() const {
	return this->raw;
}

VectorXd Data::getStateFromMeasurement() const {

	VectorXd state(4);

	if (this->data_type == SourceOfData::LIDAR) {

		double x = this->raw(0);
		double y = this->raw(1);
		state << x, y, 0.0, 0.0;

	}
	else if (this->data_type == SourceOfData::RADAR) {

		state = convert_polar_to_cartesian(this->raw);

	}
	else if (this->data_type == SourceOfData::STATE) {

		state = this->raw;
	}

	return state;
}

long long Data::get_timestamp() const {
	return this->timestamp;
}

SourceOfData Data::get_type() const {
	return this->data_type;
}

void Data::print() const {

	if (this->initialized) {

		cout << "Timestamp: " << this->timestamp << endl;
		cout << "Sensor ID: " << static_cast<int>(this->data_type) << " (LIDAR = 0 | RADAR = 1 | STATE = 2)" << endl;
		cout << "Raw Data: " << endl;
		cout << this->raw << endl;

	}
	else {

		cout << "DataPoint is not initialized." << endl;
	}
}
