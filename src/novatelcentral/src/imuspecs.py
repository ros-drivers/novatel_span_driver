from math import pow
from math import radians as RAD

def get_imu_specs(imutype)
# returns a dict

  return {
  'IMU_ADIS16488'  : { 'gyro':RAD(720.0/pow(2.0,31.0)), 'accel': 200.0/pow(2.0,31.0) 'rate':200 }
  'IMU_HG1700_AG58': { 'gyro':pow(2.0,-33.0), 'accel':0.3048 / 134217728.0 }
  'IMU_HG1700_AG62': { 'gyro':16, 'accel':0.3048 / 67108864 }
  'IMU_HG1900_CA29': { 'gyro':16, 'accel':17 }
  }.get(imutype, None)