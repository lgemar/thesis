acqSize = 2000;
com_port = 'COM5'; 

baudRate = 250000;
comPort = com_port; 
serialPort = serial(comPort, 'BaudRate', baudRate);
formatString=['a/g/m:\t','%d\t','%d\t','%d\t','%d\t','%d\t','%d\n'];
fopen(serialPort); 

h1 = figure(1); 

% Accelerometer Data
subplot(3, 1, 1); ax = line(nan, nan, 'Color', 'r'); ay = line(nan, nan, 'Color', 'g'); az = line(nan, nan, 'Color', 'b');
legend('X-axis', 'Y-axis', 'Z-axis')
title('Accelerometer output'); xlabel('s'); ylabel('m/s^2'); 

% Gyroscope Data
subplot(3, 1, 2); gx = line(nan, nan, 'Color', 'r'); gy = line(nan, nan, 'Color', 'g');
legend('XZ-axis', 'YZ-axis')
title('Gyroscope output'); xlabel('s'); ylabel('degress/s');

% Orientation Data
subplot(3, 1, 3); yaw = line(nan, nan, 'Color', 'r'); pitch = line(nan, nan, 'Color', 'g');
legend('Yaw', 'Pitch')
title('Orientation output'); xlabel('s'); ylabel('degrees');

% Initialize storage variables
t0 = tic;
t = []; 
AccRx = []; AccRy = []; AccRz = []; 
GyroRateAxz = []; GyroRateAyz = []; 

while( ishandle(h1) )
    
    t = [t toc(t0)]; 
    
    data = fscanf(serialPort, formatString);
    while( size(data) ~= 6 )
        data = fscanf(serialPort, formatString);
    end


    Rx = data(1) / AccSensitivity; % bits / (bits / g)
    Ry = data(2) / AccSensitivity; 
    Rz = data(3) / AccSensitivity; 

    R = sqrt(Rx^2 + Ry^2 + Rz^2); 
    Axr = acosd(Rx/R); 
    Ayr = acosd(Ry/R); 
    Azr = acosd(Rz/R); 


    RateAxz = data(4) / GyroSensitivity; 
    RateAyz = data(5) / GyroSensitivity; 
    
    % UPDATE VARIBLES
    
    AccRx = [AccRx Axr]; 
    AccRy = [AccRy Ayr]; 
    AccRz = [AccRz Azr]; 
    GyroRateAxz = [GyroRateAxz RateAxz]; 
    GyroRateAyz = [GyroRateAyz RateAyz]; 

    % DISPLAY THE RESULTS
    
    if( ~ishandle(h1) )
        break;
    end
    
    % Set the accelerometer data in real time
    set(ax, 'XData', t, 'YData', AccRx);
    set(ay, 'XData', t, 'YData', AccRy);
    set(az, 'XData', t, 'YData', AccRz);

    % Set the gyroscope data in real time
    set(gx, 'XData', t, 'YData', GyroRateAxz);
    set(gy, 'XData', t, 'YData', GyroRateAyz);
    
    drawnow limitrate; 
    
end

%%
fclose(serialPort); 
delete(serialPort);
clear serialPort;