function data = getRawData(IMU)

formatString=['a/g/m:%d\t','%d\t','%d\t','%d\t','%d\t','%d\t','%d\t', '%d\t', '%d\n'];

data = fscanf(IMU, formatString); % Acquire a data frame. 
while( size(data) ~= 9 )
    data = fscanf(IMU, formatString);
end