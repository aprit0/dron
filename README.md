# dron

## Messages
/barometer/altitude -Float32, low frequency barometer calculation
/joy - Joy, input rc controls
/laser/range - Range, 0 to 1m
/multiwii/imu - IMU, linear and angular
/multiwii/mag - MagneticField, x,y,z
/multiwii/pressure - Float32, 101325.0 - float(alt_data['estalt'])
/multiwii/rc - Joy, current state of fc rc controls
/optical_flow - Vector3, raw x,y pixels from pwm3901
/ultrasonic/range - Range, 0 to 6m
* `/gps/fix` (NavSatFix: latitude, longitude, altitude) - optionally includeed

