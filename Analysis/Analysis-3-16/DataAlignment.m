CurrentFolder = pwd; 
DataFolder = ([pwd, '\Data']); 
AllTestNames = {'clckalign', 'clckalign2', 'slowhorz1', 'medhorz1', 'fasthorz1', ...
    'fasthorz2', 'slowdepth', 'meddepth', 'fastdepth', 'slowyaw', 'medyaw', ... 
    'fastyaw', 'slowroll', 'medroll', ...
    'fastroll', 'horzdist', 'vertdist', 'depthdist'}; 

TestName = AllTestNames{5}; 
UnityFile = [TestName, '_unity.csv']; 
ViconFile = [TestName, '_vicon.csv']; 

UnityData = csvread([DataFolder, '\', UnityFile]); UnityData = UnityData(2:end, :); 
ViconData = csvread([DataFolder, '\', ViconFile]); ViconData = ViconData(2:end, :); 

tUnity = UnityData(:,1); 
tSensor = UnityData(:,6); 
    
window_size = 10; 
figure(1)
plot([ tsmovavg(diff(tUnity),'s',window_size,1), tsmovavg(diff(tSensor),'s',window_size,1)])
ylim([0.01 0.04])
title('Time between samples')
legend('Unity', 'Sensor')

pD = UnityData(:, 2:5); 


