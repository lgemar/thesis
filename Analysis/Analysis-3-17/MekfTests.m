%% Load the test data
DataFolder = (['C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-17\', 'Data\']); 
AllTestNames = {'yawslow', 'yawmed', 'yawfast', 'rollslow', 'rollmed', 'rollfast'}; 
ViconTestNames = {'AllTests'}; 

UnityTestName = AllTestNames{1}; 
ViconTestName = ViconTestNames{1}; 

UnityFile = [UnityTestName, '.csv']; 
UnityData = csvread([DataFolder, UnityFile]); UnityData = UnityData(2:end, :); 

ViconFile = [ViconTestName, '_vicon.csv']; 
ViconDataMat = dlmread([DataFolder, ViconFile], ','); ViconDataMat = ViconDataMat(2:end, :); 

% Align the time vectors relative to the clock on the sensor
tSensor = UnityData(:,6); 
tVicon = ViconDataMat(:,1) - (min(tSensor) + 1458243948.5022); 
tApp = UnityData(:,1) - (min(tSensor) + 189.9815); 
tSensor = tSensor - min(tSensor); 

% Plot the position and orientation as a sanity check
pU = UnityData(:,2:3); 
pV = ViconDataMat(:,2:4); 

figure(1)
subplot(2,2,1)
plot(pU) 
title('Distorted Position')
subplot(2,2,2)
plot(pV)
title('Reference Position')
subplot(2,2,3)
