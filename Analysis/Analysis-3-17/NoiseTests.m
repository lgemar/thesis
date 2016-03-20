%% Load the test data
DataFolder = (['C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-17\', 'Data\Sensor\']); 
AllTestNames = {'no_manuever1'}; 

TestName = AllTestNames{1}; 

SensorFile = [TestName, '.csv']; 
SensorData = csvread([DataFolder, SensorFile]); SensorData = SensorData(3:end, :); 
tSensor = SensorData(:,1)/1000; % it's in milliseconds 

N = size(SensorData,1); % Number of samples
dt = mean(diff(tSensor)); % Simulation time step
t = tSensor - min(tSensor); 

% Simulation variables
qref = SensorData(:,2:5); % quaternion reference array
qestT = zeros(N,4); % Triad estimates
qestK = zeros(N,4); % Kalman filter estimates
z = [SensorData(:,6:8)./repmat(VectorNorm(SensorData(:,6:8)),1,3)...
     SensorData(:,12:14)./repmat(VectorNorm(SensorData(:,12:14)),1,3)]; % sensor measurements
% z = [SensorData(:,6:8),...
%      SensorData(:,12:14)]; % sensor measurements
omega = SensorData(:,9:11); % gyro measurements
% omega = omega - repmat(mean(omega,1),N,1); % mean-subtracted gyro measurements

% Reference vectors
g = [0; 0; 9.81]; % world coordinates
% b = [cos(-pi/2-0.05)*1; sin(-pi/2-0.05)*1; 0]; % world coordinates
b = z(1,4:6)'; 

figure(1)
subplot(2,2,1)
plot( t, rad2deg( quat2eul( qref ) ) )
title('Quaternion Ref')
legend('yaw', 'pitch', 'roll')
subplot(2,2,2)
plot( t, omega )
title('Omega Input')
legend('X', 'Y', 'Z')
subplot(2,2,3)
plot( t,z(:,1:3) )
title('Accelerometer')
ylim([-10 10])
legend('X', 'Y', 'Z')
subplot(2,2,4)
plot( t, z(:,4:6) )
title('B Field')
legend('X', 'Y', 'Z')

Pgyro = cov(omega)
Paccel = cov(z(:,1:3))
Pmagn = cov(z(:,4:6))
