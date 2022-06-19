#ifndef DATAPOINT_H_
#define DATAPOINT_H_

#include "../eigen_lib/Dense"
#include "../header/utility.h"
#include <stdlib.h>
#include <iostream>


using Eigen::MatrixXd;
using Eigen::VectorXd;

enum SourceOfData {
	LIDAR, RADAR, STATE
};

struct Data {
private:
	bool initialized;
	long long timestamp;
	SourceOfData data_type;
	VectorXd raw;

public:
	Data();
	Data(const long long timestamp, const SourceOfData data_type, const VectorXd raw);
	void set(long timestamp, const SourceOfData data_type, const VectorXd raw);
	VectorXd getRawMeasurementData() const;
	VectorXd getStateFromMeasurement() const;
	SourceOfData get_type() const;
	long long get_timestamp() const;
	void print() const;
};

#endif
