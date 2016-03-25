CurrentFolder = pwd; 
DataFolder = (['C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-17\', 'Data\']); 
AllTestNames = {'TsTest', 'ClockAlign', 'ClockAlign2', 'horzslow', 'horzmed', ...
    'horzfast', 'depthslow', 'depthmed', 'depthfast', 'depthveryfast', 'clockalign3', ...
    'failure1'}; 
ViconTestNames = {'TsTest', 'ClockAlign', 'ClockAlign2', 'AllTests', 'ClockAlign3', 'FailureTests'}; 

% Compute the t0 for unity, the sensor and the vicon
UnityInit = csvread([DataFolder, 'ClockAlign.csv']); UnityInit  = UnityInit (2:end, :); 
ViconInit = csvread([DataFolder, 'ClockAlign_vicon.csv']); ViconInit = ViconInit(2:end, :); 

tA0 = min(UnityInit(:,1));
tS0 = min(UnityInit(:,6));
tV0 = min(ViconInit(:,1));

UnityTestName = AllTestNames{3}; 
ViconTestName = ViconTestNames{3}; 

UnityFile = [UnityTestName, '.csv']; 
UnityData = csvread([DataFolder, UnityFile]); UnityData = UnityData(2:end, :); 

ViconFile = [ViconTestName, '_vicon.csv']; 
ViconDataMat = dlmread([DataFolder, ViconFile], ','); ViconDataMat = ViconDataMat(2:end, :); 

tV = ViconDataMat(:,1); 
tA = UnityData(:,1); 
tS = UnityData(:,6);

temp = mean( tS - tA )
temp2 = (tS(1) - tV(1)) - tV0;

%% 

% Compute the sample rates for the Vicon, App, and Sensor sensors in two
% different ways
temp = mean(diff(tV),1); disp(['Vicon sample rate: ', num2str(1/temp), ' Hz']); 
temp = mean(diff(tA),1); disp(['App sample rate: ', num2str(1/temp), ' Hz']); 
temp = mean(diff(tS),1); disp(['Sensor sample rate: ', num2str(1/temp), ' Hz']); 

Tv = (max(tV) - min(tV)) / length(tV); disp(['Vicon sample rate: ', num2str(1/Tv), ' Hz']); 
Ta = (max(tA) - min(tA)) / length(tA); disp(['App sample rate: ', num2str(1/Ta), ' Hz']); 
Ts = (max(tS) - min(tS)) / length(tS); disp(['Sensor sample rate: ', num2str(1/Ts), ' Hz']); 

Fv = 1/Tv; 
Fa = 1/Ta; 
Fs = 1/Ts; 

% Upsample the app and sensor data streams


% Read the interesting data vectors
aX = -(UnityData(:,11)+9.81); 
yD = -UnityData(:,3); 
zW = ViconDataMat(:,4); 

% Resample the data
rV = linspace(min(tV), max(tV), ceil( length(tV) * round(Fa) / round(Fv) ))'; 
rzW = resample(zW,round(Fa),round(Fv));

clf

figure(1)

subplot(1,3,1)
plot(tV,zW,rV,rzW, 'o')
legend('original','resampled')
ylim([0.4 0.7])
title('Downsampled VICON data')
xlabel('time (s)')
ylabel('Position (m)')

% subplot(1,3,2)
% plot(tSensor,aX,rSensor,raX, 'o')
% legend('original','resampled')
% title('Upsampled sensor data')
% xlabel('time (s)')
% 
% subplot(1,3,3)
% plot(tApp,yD,rApp,ryD, 'o')
% legend('original','resampled')
% title('Upsampled app data')
% xlabel('time (s)')

%% Autocorrelation with downsampled Vicon data
% Acceleration in camera and vicon coordinate frames
ayD = diff( diff(yD) ./ diff(tA) ) ./ diff(tA(2:end)); 
azW = diff( diff(rzW) ./ diff(rV) ) ./ diff(rV(2:end));

ayD = conv(ayD, ones(10,1)/10, 'same'); 
azW = conv(azW, ones(10,1)/10, 'same'); 

figure(2)
subplot(1,3,1)
plot(tS,aX)
title('Sensor Acceleration Estimate')
ylim([-10 10])
subplot(1,3,2)
plot(tA(3:end), ayD)
title('Application Acceleration Estimate')
subplot(1,3,3)
plot(rV(3:end), azW)
title('Vicon Acceleration Estimate')

% Compute the cross correlation of the downsampled vicon reference with sensor and app
[acorav,lag] = xcorr(ayD,azW);
[~,I] = max(abs(acorav));
lav = lag(I) 
dtva = (rV(-lav) - tA(1))

[acorsv,lag] = xcorr(aX,azW);
[~,I] = max(abs(acorsv));
las = lag(I)
dtvs = (rV(-las) - tS(1)) 

dtsa = dtva - dtvs

figure(3)
subplot(1,2,1)
plot(acorav)
title('Position estimate and VICON autocorrelation')
subplot(1,2,2)
plot(acorsv)
title('Accelerometer and VICON data autocorrelation')

disp(['Latency is ', num2str(1000 * -(dtva - dtvs)), ' ms'])

% The dtsv is the true offset between the world clock and the sensor
% clock
dtsv_est = mean( rV(-las:(-las+length(tS)-1)) - tS )
dtav_est = mean( rV(-las:(-las+length(tA)-1)) - tA )
dtsa_est = dtsv_est - dtav_est % offset between the sensor clock and app clock, as measured by both referenced to vicon
dtsa = mean( tS - tA ) % offset between sensor clock and app clock, as measured directly

% Create latency plot  

ayD = conv(ayD, ones(10,1)/10, 'same'); 

figure; 
plot(tS(2:end-1), aX(2:end-1), tS(2:end-1), azW((-las+2):((-las+2)+(length(tS)-2)-1)))
legend('aX', 'azW')

figure;
plot(tS(2:end-1), aX(2:end-1), tS(2:end-1), ayD / 1000)
legend('VICON', 'Tool Tracker')
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')
title('Reference and estimated acceleration')



