CurrentFolder = pwd; 
DataFolder = (['C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-17\', 'Data\']); 
AllTestNames = {'TsTest', 'ClockAlign', 'ClockAlign2', 'horzslow', 'horzmed', ...
    'horzfast', 'depthslow', 'depthmed', 'depthfast', 'depthveryfast', 'clockalign3', ...
    'failure1'}; 
ViconTestNames = {'TsTest', 'ClockAlign', 'ClockAlign2', 'AllTests', 'ClockAlign3', 'FailureTests'}; 

UnityTestName = AllTestNames{11}; 
ViconTestName = ViconTestNames{5}; 

UnityFile = [UnityTestName, '.csv']; 
UnityData = csvread([DataFolder, UnityFile]); UnityData = UnityData(2:end, :); 

ViconFile = [ViconTestName, '_vicon.csv']; 
ViconDataMat = dlmread([DataFolder, ViconFile], ','); ViconDataMat = ViconDataMat(2:end, :); 

tVicon = ViconDataMat(:,1); 
tApp = UnityData(:,1); 
tSensor = UnityData(:,6); 

% Compute the sample rates for the Vicon, App, and Sensor sensors in two
% different ways
temp = mean(diff(tVicon),1); disp(['Vicon sample rate: ', num2str(1/temp), ' Hz']); 
temp = mean(diff(tApp),1); disp(['App sample rate: ', num2str(1/temp), ' Hz']); 
temp = mean(diff(tSensor),1); disp(['Sensor sample rate: ', num2str(1/temp), ' Hz']); 

Tv = (max(tVicon) - min(tVicon)) / length(tVicon); disp(['Vicon sample rate: ', num2str(1/Tv), ' Hz']); 
Ta = (max(tApp) - min(tApp)) / length(tApp); disp(['App sample rate: ', num2str(1/Ta), ' Hz']); 
Ts = (max(tSensor) - min(tSensor)) / length(tSensor); disp(['Sensor sample rate: ', num2str(1/Ts), ' Hz']); 

Fv = 1/Tv; 
Fa = 1/Ta; 
Fs = 1/Ts; 

% Upsample the app and sensor data streams


% Read the interesting data vectors
aX = -(UnityData(:,11)+9.81); 
yD = -UnityData(:,3); 
zW = ViconDataMat(:,4); 

% Resample the data
rVicon = linspace(min(tVicon), max(tVicon), ceil( length(tVicon) * round(Fa) / round(Fv) ))'; 
rzW = resample(zW,round(Fa),round(Fv));

clf

figure(1)

subplot(1,3,1)
plot(tVicon,zW,rVicon,rzW, 'o')
legend('original','resampled')
ylim([0.4 0.7])
title('Downsampled VICON data')
xlabel('time (s)')
ylabel('Position (m)')

subplot(1,3,2)
plot(tSensor,aX,rSensor,raX, 'o')
legend('original','resampled')
title('Upsampled sensor data')
xlabel('time (s)')

subplot(1,3,3)
plot(tApp,yD,rApp,ryD, 'o')
legend('original','resampled')
title('Upsampled app data')
xlabel('time (s)')

%% Autocorrelation with downsampled Vicon data
% Acceleration in camera and vicon coordinate frames
ayD = diff( diff(yD) ./ diff(tApp) ) ./ diff(tApp(2:end)); 
azW = diff( diff(rzW) ./ diff(rVicon) ) ./ diff(rVicon(2:end));

ayD = conv(ayD, ones(10,1)/10, 'same'); 
azW = conv(azW, ones(10,1)/10, 'same'); 

figure(2)
subplot(1,3,1)
plot(tSensor,aX)
title('Sensor Acceleration Estimate')
ylim([-10 10])
subplot(1,3,2)
plot(tApp(3:end), ayD)
title('Application Acceleration Estimate')
subplot(1,3,3)
plot(rVicon(3:end), azW)
title('Vicon Acceleration Estimate')

% Compute the cross correlation of the downsampled vicon reference with sensor and app
[acorDW,lag] = xcorr(ayD,azW);
[~,I] = max(abs(acorDW));
lagDiffDW = lag(I) 
timeDiffDW = lagDiffDW/Fa

[acorSW,lag] = xcorr(aX,azW);
[~,I] = max(abs(acorSW));
lagDiffSW = lag(I)
timeDiffSW = lagDiffSW/Fa 

figure(3)
subplot(1,2,1)
plot(acorDW)
title('Image and ref autocorrelation')
subplot(1,2,2)
plot(acorSW)
title('Sensor and ref autocorrelation')

disp(['Latency is ', num2str(1000 * (timeDiffDW - timeDiffSW)), ' ms'])

% The lagDiffSW is the true offset between the sensor clock and world clock
offsetSW = mean( rVicon(-lagDiffSW:(-lagDiffSW+length(tSensor)-1)) - tSensor )
offsetDW = mean( rVicon(-lagDiffSW:(-lagDiffSW+length(tApp)-1)) - tApp )

offsetSD1 = offsetDW - offsetSW % offset between the sensor clock and app clock, as measured by both referenced to vicon
offsetSD2 = mean( tSensor - tApp ) % offset between sensor clock and app clock, as measured directly

% Create latency plot  

ayD = conv(ayD, ones(10,1)/10, 'same'); 

figure; 
subplot(1,2,1)
plot(tSensor(2:end-1), aX(2:end-1), tSensor(2:end-1), azW((-lagDiffSW+2):((-lagDiffSW+2)+(length(tSensor)-2)-1)))
legend('aX', 'azW')
subplot(1,2,2)
plot(tSensor(2:end-1), aX(2:end-1), tSensor(2:end-1), ayD / 1000)
legend('aX', 'ayD')



