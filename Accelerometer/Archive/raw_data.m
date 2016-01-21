com_port = 'COM5'; 
acqSize=500;

%% Open the serial port
disp('Open the serial port...')
baud_rate = 38400; 
s = serial(com_port, 'BaudRate', baud_rate);
fopen(s); 
formatspec  = ['a/g/m:\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\n']; 

%% Set up the filters and parameters for acquisition
t=[0];
i=1;
dt=0;
GyroRate=zeros(3,acqSize);
Acc=zeros(3,acqSize);
Magn=zeros(3,acqSize);

%% Do the acquisition
h1 = figure(1); 

% Accelerometer Data
subplot(3, 1, 1); ax = line(nan, nan, 'Color', 'r'); ay = line(nan, nan, 'Color', 'g'); az = line(nan, nan, 'Color', 'b');
legend('X-axis', 'Y-axis', 'Z-axis')
title('Accelerometer raw output'); xlabel('s'); ylabel('bits'); 

% Gyroscope Data
subplot(3, 1, 2); gx = line(nan, nan, 'Color', 'r'); gy = line(nan, nan, 'Color', 'g'); gz = line(nan, nan, 'Color', 'b');
legend('X-axis', 'Y-axis', 'Z-axis')
title('Gyroscope raw output'); xlabel('s'); ylabel('bits'); 

% Magnetometer Data
subplot(3, 1, 3); mx = line(nan, nan, 'Color', 'r'); my = line(nan, nan, 'Color', 'g'); mz = line(nan, nan, 'Color', 'b');
legend('X-axis', 'Y-axis', 'Z-axis')
title('Magnetometer raw output'); xlabel('s'); ylabel('bits'); 

disp('Complete acquisition...')
while(i<=acqSize)
    if(i>1)
        dt = toc(t0);
        t=[t t(length(t))+dt];
    end

		D = getRawSample(s, formatspec);
        t0 = tic;

        %------
        % pause(0.005)
        %------
    
	% Get the data from the sampler
    Acc(1,i)=D.ax; Acc(2,i)=D.ay; Acc(3,i)=D.az;
    Magn(1,i)=D.mx; Magn(2,i)=D.my; Magn(3,i)=D.mz;
    GyroRate(1,i)=D.gx; GyroRate(2,i)=D.gy; GyroRate(3,i)=D.gz;

	% Safety piece
	if(~ishandle(h1))
		break;
    end

    i=i+1;
end

i = i - 1;
% Set the accelerometer data in real time
set(ax, 'XData', t, 'YData', Acc(1,1:i));
set(ay, 'XData', t, 'YData', Acc(2,1:i));
set(az, 'XData', t, 'YData', Acc(3,1:i));

% Set the gyroscope data in real time
set(gx, 'XData', t, 'YData', GyroRate(1,1:i));
set(gy, 'XData', t, 'YData', GyroRate(2,1:i));
set(gz, 'XData', t, 'YData', GyroRate(3,1:i));

% Set the magnetometer data in real time
set(mx, 'XData', t, 'YData', Magn(1,1:i));
set(my, 'XData', t, 'YData', Magn(2,1:i));
set(mz, 'XData', t, 'YData', Magn(3,1:i));
drawnow limitrate

disp('Cleaning up...')
    
% Clean up the serial port from the workspace
fclose(s); 
delete(s)
clear s

disp(['Sampling rate: ' num2str(length(t)/max(t))])

%% Calibration
acc_res = 4/(2^16);
gyrorate_res = 2*250/(2^16); 
magn_res = 2*1200/(2^13);

% Bias calibration
acc_bias = mean(Acc, 2) * acc_res + [0; 0; -1]
gyrorate_bias = mean(GyroRate, 2) * gyrorate_res
magn_bias = mean(Magn, 2) * magn_res

% Standard deviation calibration
acc_std = std(Acc, 0, 2) * acc_res
gyrorate_std = std(GyroRate, 0, 2) * gyrorate_res
magn_std = std(Magn, 0, 2) * magn_res

%% Drift calculation 
AccCal = (Acc * acc_res - repmat(acc_bias + [0; 0; 1], 1, size(Acc, 2))) * 9.81; 
GyroCal = GyroRate * gyrorate_res - repmat(gyrorate_bias, 1, size(GyroRate, 2)); 
MagnCal = Magn * magn_res - repmat(magn_bias, 1, size(Magn, 2)); 

% Drifting velocity data
figure; 
subplot(3, 1, 1)
plot(t, AccCal(1,:), t, AccCal(2,:), t, AccCal(3,:))
title('Acceleration'); xlabel('s'); ylabel('m/s^2')
legend('X-axis', 'Y-axis', 'Z-axis')

subplot(3, 1, 2)
vx = cumtrapz(t, AccCal(1,:)); vy = cumtrapz(t, AccCal(2,:)); vz = cumtrapz(t, AccCal(3,:)); 
plot(t, vx, t, vy, t, vz)
title('Velocity'); xlabel('s'); ylabel('m/s')
legend('X-axis', 'Y-axis', 'Z-axis')

subplot(3, 1, 3)
px = cumtrapz(t, vx); py = cumtrapz(t,vy); pz = cumtrapz(t,vz); 
plot(t, px, t, py, t, pz)
title('Position'); xlabel('s'); ylabel('m')
legend('X-axis', 'Y-axis', 'Z-axis')

%% Drift calculation after filtering
% Filter parameters
k = 3; % order of the filter
f = 101; % frame length
% sgolayfilt(AccCal(1,:), k, f)

% figure; 
% subplot(3, 1, 1)
% plot(t, sgolayfilt(AccCal(1,:), k, f), t, sgolayfilt(AccCal(2,:), k, f), t, sgolayfilt(AccCal(3,:), k, f))
% title('Acceleration'); xlabel('s'); ylabel('m/s^2')
% legend('X-axis', 'Y-axis', 'Z-axis')
% 
% subplot(3, 1, 2)
% vx = cumtrapz(t, sgolayfilt(AccCal(1,:), k, f)); vy = cumtrapz(t, sgolayfilt(AccCal(2,:), k, f)); vz = cumtrapz(t, sgolayfilt(AccCal(3,:), k, f)); 
% plot(t, vx, t, vy, t, vz)
% title('Velocity'); xlabel('s'); ylabel('m/s');
% legend('X-axis', 'Y-axis', 'Z-axis')
% 
% subplot(3, 1, 3)
% px = cumtrapz(t,vx); py = cumtrapz(t,vy); pz = cumtrapz(t,vz); 
% plot(t, px, t, py, t, pz)
% title('Position'); xlabel('s'); ylabel('m');
% legend('X-axis', 'Y-axis', 'Z-axis')


