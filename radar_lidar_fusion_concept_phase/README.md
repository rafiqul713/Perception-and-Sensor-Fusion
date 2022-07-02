### Data
#### This project considers two sensors: RADAR and LiDAR. 
- RADAR sensor provides the measurement in polar coordinates (non-linear, that is why extended Kalman filter). 
- LiDAR sensor provides measurement in cartesian coordinates (linear). 

Both measurements (LiDAR or RADAR) may be noisy in real-life scenarios. 

##### RADAR measurement 
The RADAR measurement contains:
- Range (rho): Distance of an object from a particular point
- Bearning (phi): Determine the direction of the target. The angle between true north and the target.  
- Radial velocity (rho dot): rate of change of distance (range) between observer and the target. More detail: https://en.wikipedia.org/wiki/Radial_velocity  

#### Goal: To predict the position and velocity of a system in cartesian coordinates (position_x, position_y, velocity_x, velocity_y).
For simplicity, consider the constant velocity. 

##### Input format (RADAR): 
###### R, 7.5087, 0.0127032,	-1.89765,	1477010443914714,	7.45,	0.100001,	-1.8165,	-0.908239
	
###### R (RADAR), measurment_range, measurement_bearing, measurement_radial_velocity, timestamp (when taken the measurement),  ground_truth_position_measurement_in_x_direction, ground_truth_position_measurement_in_y_direction, ground_truth_velocity_in_x_direction, ground_truth_velocity_in_y_direction
![Alt text](media_file\visualization\radar_meas.png?raw=true "RADAR measurement")


##### Input format (LiDAR):
###### L	7.3426,	0.0345312,	1477010443969766,	7.35,	0.0500011,	-1.81648,	-0.90823
###### L (LiDAR), measurment_position_in_x_direction, measurement_position_in_y_direction, timestamp (when taken the measurement),  ground_truth_position_measurement_in_x_direction, ground_truth_position_measurement_in_y_direction, ground_truth_velocity_in_x_direction, ground_truth_velocity_in_y_direction
![Alt text](media_file\visualization\lidar_meas.png?raw=true "LiDAR measurement")
##### Output format: 
###### 6.96164	-0.0655127	-2.13393	-0.628335	6.74585	-0.143185	6.95	-0.15	-1.81898	-1.81898
###### predicted_position_in_x_direction, predicted_position_in_y_direction,
predicted_velocity_in_x_direction, predicted_velocity_in_y_direction,  
measured_position_in_x_direction(cartesian coordinate), measured_position_in_y_direction (cartesian coordinate), ground_truth_position_measurement_in_x_direction, ground_truth_position_measurement_in_y_direction, ground_truth_velocity_in_x_direction, ground_truth_velocity_in_y_direction
