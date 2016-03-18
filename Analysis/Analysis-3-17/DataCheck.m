CurrentFolder = pwd; 
DataFolder = (['C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-17\', 'Data\']); 
AllTestNames = {'TsTest', 'ClockAlign', 'ClockAlign2', 'horzslow', 'horzmed', 'horzfast', 'depthslow', 'depthmed', 'depthfast', 'depthveryfast', 'clockalign3', ...
    'failure1'}; 

ViconTestNames = {'TsTest', 'ClockAlign', 'ClockAlign2', 'AllTests', 'ClockAlign3', 'FailureTests'}; 

UnityTestName = AllTestNames{11}; 
ViconTestName = ViconTestNames{5}; 

UnityFile = [UnityTestName, '.csv']; 
UnityData = csvread([DataFolder, UnityFile]); UnityData = UnityData(2:end, :); 

ViconFile = [ViconTestName, '_vicon.csv']; 
ViconDataMat = dlmread([DataFolder, ViconFile], ','); ViconDataMat = ViconDataMat(2:end, :); 
tVicon = ViconDataMat(:,1); 

tUnity = UnityData(:,1); 
tSensor = UnityData(:,6); 

clf
window_size = 10; 
figure(1)
subplot(1,3,1)
plot(tsmovavg(diff(tUnity),'s',window_size,1))
subplot(1,3,2)
plot(tsmovavg(diff(tSensor),'s',window_size,1))
subplot(1,3,3)
plot(tsmovavg(diff(tVicon),'s',window_size,1))
title('Time between samples')
ylim([-0.01 0.04])

pD = UnityData(:, 2:3); 
pW = ViconDataMat(:,2:4); 


figure(2)
subplot(1,2,1)
plot(tUnity, pD)
subplot(1,2,2)
plot(pW)

qC = UnityData(:,7:10); 
qW = ViconDataMat(:,5:8); 

figure(3)
subplot(1,2,1)
plot(tUnity, qC)
subplot(1,2,2)
plot(tVicon, qW)
