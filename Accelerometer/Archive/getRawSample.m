function s = getRawSample(serial, formatspec)

% Make sure to read a valid data sample with all 9 fields
data = fscanf(serial, formatspec);
while( size(data) ~= 9 )
	data = fscanf(serial, formatspec);
end

% Read in and adjust the accelerometer readings
ax = data(1); ay = data(2); az = data(3);

% Read in and adjust the magnetometer readings
mx = data(7); 
my = data(8); 
mz = data(9);  

% Read in and adjust the gyroscope data
gx = data(4); 
gy = data(5); 
gz = data(6); 


% Set the fields and values of the output structure
field1 = 'ax';  value1 = ax;
field2 = 'ay';  value2 = ay;
field3 = 'az';  value3 = az;
field4 = 'gx';  value4 = gx;
field5 = 'gy';  value5 = gy;
field6 = 'gz';  value6 = gz;
field7 = 'mx';  value7 = mx;
field8 = 'my';  value8 = my;
field9 = 'mz';  value9 = mz;

s = struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,field6,value6,field7,value7,field8,value8,field9,value9);
