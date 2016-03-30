close all
DataFolder = (['C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\', 'Data\']); 

AllTestNames = {'test1', 'clckalign1', 'horz1'}; 
ViconTestNames = {'test1', 'clkalign1', 'alltests'}; 
ViconTestName = ViconTestNames{3}; 
UnityTestName = AllTestNames{3}; 

%% Clock alignment
DataFolderPr = (['C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\', 'Data\']); 
UnityData = csvread([DataFolderPr, 'clckalign1.csv']); UnityData  = UnityData (2:end, :); 
ViconData = csvread([DataFolderPr, 'clkalign1_vicon.csv']); ViconData = ViconData(2:end, :); 

tV = ViconData(:,1); 
tA = UnityData(:,1); 

% Compute the sample rates for the Vicon, App, and Sensor sensors in two
% different ways
temp = mean(diff(tV),1); disp(['Vicon sample rate: ', num2str(1/temp), ' Hz']); 
temp = mean(diff(tA),1); disp(['App sample rate: ', num2str(1/temp), ' Hz']); 

Tv = (max(tV) - min(tV)) / length(tV); disp(['Vicon sample rate: ', num2str(1/Tv), ' Hz']); 
Ta = (max(tA) - min(tA)) / length(tA); disp(['App sample rate: ', num2str(1/Ta), ' Hz']); 

Fv = 1/Tv; 
Fa = 1/Ta;  

% Read the interesting data vectors
yD = -UnityData(:,3); 
zW = ViconData(:,4); 

% Resample the data
rV = linspace(min(tV), max(tV), ceil( length(tV) * round(Fa) / round(Fv) ))'; 
rzW = resample(zW,round(Fa),round(Fv));

% Smooth the data
yD = conv(yD, ones(10,1)/10, 'same'); 
rzW = conv(rzW, ones(10,1)/10, 'same'); 

% Compute the cross correlation of the downsampled vicon reference with sensor and app
[acorav,lag] = xcorr(yD - mean(yD),rzW - mean(rzW));
[~,I] = max(abs(acorav));
lav = lag(I); 
dtva = (rV(-lav) - tA(1))

% Align the time vectors relative to the clock in the application
rVpr = rV - (dtva + min(tA)); % Transform vicon time to app time
tApr = tA - min(tA); % Transform app time to 0

Plot = 0; 

if(Plot == 1)

    figure; 
    plot(tV,zW,rV,rzW, 'o')
    legend('original','resampled')
    ylim([0.4 0.7])
    title('Downsampled VICON data')
    xlabel('time (s)')
    ylabel('Position (m)')

    % Plot the two data vectors next to each other
    figure; 
    subplot(1,2,1)
    plot(yD)
    subplot(1,2,2)
    plot(rzW)

    figure; 
    plot(acorav)
    title('Vertical position measurement and VICON autocorrelation')

end

figure; 

subplot(1,2,1)
plot(tApr, yD)
xlim([min(tApr) max(tApr)])
title('Application')

subplot(2,2,2)
plot(rVpr, rzW)
xlim([min(tApr) max(tApr)])
title('Vicon')

%% Read in the UNITY data
UnityFile = [UnityTestName, '.csv']; 
UnityData = csvread([DataFolder, UnityFile]); UnityData = UnityData(2:end, :); 

t = UnityData(:,1); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Raw Position data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pu = UnityData(:, 2:4); 
figure; 
plot(t, pu)
legend('x', 'y', 's')
title('Position in undistorted image')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Raw Sensor data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
a = UnityData(:,11:13); % accelerometer observations
g = UnityData(:,14:16); % gyro measurements
m = UnityData(:,17:19); % magnetometer observations
figure; 

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

subplot(2,3,5)
plot(t,m)
title('Magnetometer')
xlabel('time (s)')
ylabel('\mu T')
legend('X', 'Y', 'Z')

%% Check the VICON data
close all; 
clf; 

ViconFile = [ViconTestName, '_vicon.csv']; 
ViconDataMat = dlmread([DataFolder, ViconFile], ','); ViconDataMat = ViconDataMat(2:end, :); 
tVicon = ViconDataMat(:,1); 

tV = tVicon - (dtva + min(t)); 
pW = ViconDataMat(:,2:4);

figure; 

subplot(1,2,1)
plot(tApr, pu)
xlim([min(tApr) max(tApr)])
title('Application')

subplot(1,2,2)
plot(tV, pW)
xlim([min(tApr) max(tApr)])
title('Vicon')

qC = UnityData(:,7:10); 
qW = ViconDataMat(:,5:8); 

figure; 
subplot(1,2,1)
plot(t, qC)
subplot(1,2,2)
plot(tV, qW)

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
