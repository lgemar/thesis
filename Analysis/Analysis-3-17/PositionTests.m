CurrentFolder = pwd; 
DataFolder = (['C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-17\', 'Data\']); 
AllTestNames = {'TsTest', 'ClockAlign', 'ClockAlign2', 'horzslow', 'horzmed', ...
    'horzfast', 'depthslow', 'depthmed', 'depthfast', 'depthveryfast', 'clockalign3', ...
    'failure1'}; 
ViconTestNames = {'TsTest', 'ClockAlign', 'ClockAlign2', 'AllTests', 'ClockAlign3', 'FailureTests'}; 

UnityTestName = AllTestNames{3}; 
ViconTestName = ViconTestNames{3}; 

UnityFile = [UnityTestName, '.csv']; 
UnityData = csvread([DataFolder, UnityFile]); UnityData = UnityData(2:end, :); 

ViconFile = [ViconTestName, '_vicon.csv']; 
ViconDataMat = dlmread([DataFolder, ViconFile], ','); ViconDataMat = ViconDataMat(2:end, :); 

% Align the time vectors relative to the clock on the sensor
tSensor = UnityData(:,6); 
tVicon = ViconDataMat(:,1) - (min(tSensor) + 1458243948.5022); 
tApp = UnityData(:,1) - (min(tSensor) + 189.9815); 
tSensor = tSensor - min(tSensor); 

zW = ViconDataMat(:,4); 
aX = -(UnityData(:,11)+9.81); 

figure(1)
subplot(1,2,1)
plot(tVicon( tVicon > 0 & tVicon < max(tSensor) ), zW( tVicon > 0 & tVicon < max(tSensor) ))
subplot(1,2,2)
plot( aX )
