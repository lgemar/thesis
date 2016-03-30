clf
close all
CurrentFolder = pwd; 
DataFolder = (['C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\', 'Data\']); 
AllTestNames = {'test1', 'clckalign1', 'horz1', 'v2horz1'}; 
ViconTestNames = {'test1', 'clkalign1', 'alltests', 'alltests2'}; 


UnityTestName = AllTestNames{3};
ViconTestName = ViconTestNames{3}; 
UnityFile = [UnityTestName, '.csv']; 
UnityData = csvread([DataFolder, UnityFile]); UnityData = UnityData(2:end, :); 

t = UnityData(:,1); 
tSensor = UnityData(:,6); 

%% 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Raw Position data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pu = UnityData(:, 2:4); 
figure(1); 
plot(t, pu)
legend('x', 'y', 's')
title('Position in undistorted image')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Raw Sensor data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
a = UnityData(:,11:13); % accelerometer observations
g = UnityData(:,14:16); % gyro measurements
m = UnityData(:,17:19); % magnetometer observations
figure(2); 

subplot(2,2,1)
plot(t,a)
title('Accelerometer')
xlabel('time (s)')
ylabel('m/s^2')
legend('X', 'Y', 'Z')

subplot(2,2,2)
plot(t,g)
title('Gyroscope')
xlabel('time (s)')
s = sprintf('%c/s', char(176)); xlabel('time (s)'); 
ylabel(s)
legend('X', 'Y', 'Z')

subplot(2,2,3)
plot(t,m)
title('Magnetometer')
xlabel('time (s)')
ylabel('\mu T')
legend('X', 'Y', 'Z')


%% 
 
ViconFile = [ViconTestName, '_vicon.csv']; 
ViconDataMat = dlmread([DataFolder, ViconFile], ','); ViconDataMat = ViconDataMat(2:end, :); 
tVicon = ViconDataMat(:,1); 

tV = tVicon; 
pW = ViconDataMat(:,2:4);

figure; 

subplot(1,2,1)
plot(t, pu)
subplot(1,2,2)
plot(tV, pW)

qC = UnityData(:,7:10); 
qW = ViconDataMat(:,5:8); 

figure; 
subplot(1,2,1)
plot(t, qC)
subplot(1,2,2)
plot(tVicon, qW)

clf
window_size = 10; 
figure; 
subplot(1,3,1)
plot(tsmovavg(diff(t),'s',window_size,1))
subplot(1,3,2)
plot(tsmovavg(diff(tSensor),'s',window_size,1))
subplot(1,3,3)
plot(tsmovavg(diff(tVicon),'s',window_size,1))
title('Time between samples')
ylim([-0.01 0.04])

