%% Close, clear, and set up for acquisition
clear all;
acqSize = 2000;
com_port = 'COM6'; 

%% Open the serial port
A = Accelerometer(com_port); 
A = A.calibrate();

%% Set up the filters and parameters for acquisition
i=1;
dt=0;
GyroRate=zeros(3,acqSize);
Acc=zeros(3,acqSize);
t = zeros(1, acqSize);

%% Do the acquisition
h1 = figure(1); 

% Accelerometer Data
subplot(2, 1, 1); ax = line(nan, nan, 'Color', 'r'); ay = line(nan, nan, 'Color', 'g'); az = line(nan, nan, 'Color', 'b');
legend('X-axis', 'Y-axis', 'Z-axis')
title('Accelerometer calibrated output'); xlabel('s'); ylabel('m/s^2'); 

% Gyroscope Data
subplot(2, 1, 2); gx = line(nan, nan, 'Color', 'r'); gy = line(nan, nan, 'Color', 'g'); gz = line(nan, nan, 'Color', 'b');
legend('X-axis', 'Y-axis', 'Z-axis')
title('Gyroscope calibrated output'); xlabel('s'); ylabel('degress/s'); 

disp('Starting acquisition...')
t0 = tic;
while(i<=acqSize)
    if(i>1)
        t(i)=toc(t0);
    end

    % Get the data
    D = A.getDataSample();

	% Get the data from the sampler
    Acc(1:3,i) = D(1:3)';
    GyroRate(1:3,i) = D(4:6)';
    
    i=i+1;
end
disp('Finishing acquisition...')
disp(['Sample rate: ', num2str(length(t)/max(t))])
disp('Cleaning up...')
    
%% Clean up the serial port from the workspace
A.close(); 
A.delete(); 
clear A

i = i-1;
% Set the accelerometer data in real time
set(ax, 'XData', t(1:i), 'YData', Acc(1,1:i));
set(ay, 'XData', t(1:i), 'YData', Acc(2,1:i));
set(az, 'XData', t(1:i), 'YData', Acc(3,1:i));

% Set the gyroscope data in real time
set(gx, 'XData', t(1:i), 'YData', GyroRate(1,1:i));
set(gy, 'XData', t(1:i), 'YData', GyroRate(2,1:i));
set(gz, 'XData', t(1:i), 'YData', GyroRate(3,1:i));
