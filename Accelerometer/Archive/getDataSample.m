function s = getDataSample(serial)

% Allocate the output
s = zeros(1, 9);
formatspec  = ['a/g/m:\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\n']; 

% Make sure to read a valid data sample with all 9 fields
data = fscanf(serial, formatspec);
while( size(data) ~= 9 )
	data = fscanf(serial, formatspec);
end

% Read in and adjust the accelerometer readings
accel = (4*9.81) * data(1:3) / (2^16);
accel_bias = [-0.0349 -0.0213 -0.0349]; 
accel_calib = accel - accel_bias';

% Read in and adjust the gyroscope data
gyro = (2*250) * data(4:6) / (2^16);  
gyro_bias = [1.1543 -0.5184 0.3289]; 
gyro_calib = gyro - gyro_bias';

% Read in and adjust the magnetometer readings
magn = (2*1200) * data(7:9) / (2^16);
magn_bias = [-16.9542 -5.4400 16.9730];
magn_calib = magn - magn_bias';

s(1:3) = accel_calib; 
s(4:6) = gyro_calib; 
s(7:9) = magn_calib;
