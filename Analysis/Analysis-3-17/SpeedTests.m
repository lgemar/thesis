CurrentFolder = pwd; 
DataFolder = (['C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-17\', 'Data\']); 
AllTestNames = {'failure1'}; 
ViconTestNames = {'TsTest', 'ClockAlign', 'ClockAlign2', 'AllTests', 'ClockAlign3', 'FailureTests'}; 

UnityTestName = AllTestNames{1}; 
ViconTestName = ViconTestNames{6}; 

UnityFile = [UnityTestName, '.csv']; 
UnityData = csvread([DataFolder, UnityFile]); UnityData = UnityData(2:end, :); 

ViconFile = [ViconTestName, '_vicon.csv']; 
ViconDataMat = dlmread([DataFolder, ViconFile], ','); ViconDataMat = ViconDataMat(2:end, :); 

% Align the time vectors relative to the clock on the sensor
tSensor = UnityData(:,6); 
tVicon = ViconDataMat(:,1) - (min(tSensor) +  1.458243951045658e+09); 
tApp = UnityData(:,1) - (min(tSensor) + 1859.9815); 
tSensor = tSensor - min(tSensor); 

pV = ViconDataMat(:,2:4); 
pU = UnityData(:,2:3); 

figure(1)
subplot(1,2,1)
plot(tVicon(2:end), abs(diff(pV) ./ repmat(diff(tVicon),1,3)) )
xlim([0 max(tSensor)])
title('Tool Velocity')
subplot(1,2,2)
plot(tSensor, pU)
xlim([0 max(tSensor)])