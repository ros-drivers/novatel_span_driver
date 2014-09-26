# message 812
#Contains the RAWIMU data corrected for gravity, 
#the earths rotation and estimated sensor errors.
#gyro values are instantaneous and in radians: multiply by datarate 

int32 week
float64 gpssec

float64 pitch_rate
float64 roll_rate
float64 yaw_rate

float64 x_accel
float64 y_accel
float64 z_accel