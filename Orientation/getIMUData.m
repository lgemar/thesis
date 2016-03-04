function [atool, gtool] = getIMUData(IMU)

formatString=['a/g/m:\t','%d\t','%d\t','%d\t','%d\t','%d\t','%d\n'];

data = fscanf(IMU, formatString); % Acquire a data frame. 
while( size(data) ~= 6 )
    data = fscanf(IMU, formatString);
end

% Return accelerometer data as a row vector
AccSensitivity = (2^16) / (4 * 9.81); % bits / g 
atool = data(1:3)' / AccSensitivity;

% Return gyroscope data as a row vector
GyroSensitivity = 2^16 / (2 * 250); %bits / (deg / s)
gtool = data(4:6)' / GyroSensitivity; 

