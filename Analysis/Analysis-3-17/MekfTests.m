%% Load the test data
DataFolder = (['C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-17\', 'Data\']); 
AllTestNames = {'yawslow', 'yawmed', 'yawfast', 'rollslow', 'rollmed', 'rollfast'}; 
ViconTestNames = {'AllTests'}; 

UnityTestName = AllTestNames{4}; 
ViconTestName = ViconTestNames{1}; 

UnityFile = [UnityTestName, '.csv']; 
UnityData = csvread([DataFolder, UnityFile]); UnityData = UnityData(2:end, :); 

ViconFile = [ViconTestName, '_vicon.csv']; 
ViconDataMat = dlmread([DataFolder, ViconFile], ','); ViconDataMat = ViconDataMat(2:end, :); 

% Align the time vectors relative to the clock on the sensor
tSensor = UnityData(:,6); 
tVicon = ViconDataMat(:,1) - (min(tSensor) + 1458243948.5022); 
tApp = UnityData(:,1); 
tApp = tApp - min(tApp); 
tSensor = tSensor - min(tSensor); 

% Plot the position and orientation as a sanity check
pU = UnityData(:,2:3); 
pV = ViconDataMat(:,2:4); 

qS = UnityData(:,7:10); 
qV = ViconDataMat(:,4:7); 

figure(1)
subplot(2,2,1)
plot(tSensor, pU) 
title('Distorted Position')
subplot(2,2,2)
plot(tVicon( tVicon > 0 & tVicon < max(tSensor) ), pV(tVicon > 0 & tVicon < max(tSensor), :))
title('Reference Position')
subplot(2,2,3)
plot( tSensor, rad2deg( quat2eul( qS ) ) )
subplot(2,2,4)
plot(tVicon( tVicon > 0 & tVicon < max(tSensor) ), rad2deg( quat2eul(qV(tVicon > 0 & tVicon < max(tSensor), :) ) ) )

%% Initialize orientation variables

N = length(tSensor); % Number of samples
dt = mean(diff(tSensor)); % Simulation time step
t = tSensor; 

% Reference vectors
g = [0; 0; -1]; % world coordinates
b = [cos(0)*1; sin(0)*1; 0]; % world coordinates

% Simulation variables
qref = qV; % quaternion reference array
qestT = zeros(N,4); % Triad estimates
qestK = zeros(N,4); % Kalman filter estimates
z = [UnityData(:,11:13), UnityData(:,17:19)]; % sensor measurements
omega = UnityData(:,14:16); 

%% Run the TRIAD algorithm over the data set

for i = 1:size(z,1)
    
    mg = z(i,1:3)'; % measurement of g vector in body
    mb = z(i,4:6)'; % measurement of b vector in body
    
    qe =  triadFun(g,mg,b,mb,eye(3,3)); % column-major quaternion 
    qestT(i,:) = qflip( qe ); % row-major quaternion estimate
    
end

figure(2)
subplot(1,2,1)
plot( rad2deg( quat2eul( qref ) ) )
legend('yaw', 'pitch', 'roll')
% plot( qref )
% legend('w','x','y','z')
title('Quaternion Ref')

subplot(1,2,2)
plot( rad2deg( quat2eul( qestT ) ) )
legend('yaw', 'pitch', 'roll')
% plot( qestT )
% legend('w','x','y','z')
title('Quaternion Est')

%% Run the MEKF over the data set

% Compute initial state vector using TRIAD algorithm
mg = z(1,1:3)'; % measurement of g vector in body
mb = z(1,4:6)'; % measurement of b vector in body
qe = triadFun(g,mg,b,mb,eye(3,3)); % column-major quaternion 

% Initalize the covariance matrix, process noise, and measurement noise
Sp = [0.05; 0.05; 0.05]; Pe = diag(Sp); 
Sq = [3e-5 3e-5 3e-5]'; Q = diag(Sq); 
Sr = (0.05 * ones(6,1)); R = diag(Sr); 

for k = 2:N

    % Covariance prediction (Pp)
    Pp = Pe + Q ;

    % State prediction (Qp)
    DeltaTheta = omega(k,:)' * dt; 
    Alpha = cos( norm(DeltaTheta)/2 ); 
    Beta = sin( norm(DeltaTheta)/2 ) / norm( DeltaTheta ); 
    
    qp = Alpha * qe + Beta * XiMat( qe ) * DeltaTheta; 
    qp = qp / norm( qp ); 

    % Sensitivity matrix
    r1 = g; r2 = b; 
    pB1 = AMat( qp ) * r1; pB2 = AMat( qp ) * r2;
    H = [ 2*CrossMat( pB1 ); 2*CrossMat( pB2 )]; 

    % Kalman gain
    K = Pp * H' / (H * Pp * H' + R);

    % Observation, Prediction error
    Zm = z(k,:)'; 
    Ze = [ AMat( qp ) * r1 ; AMat( qp ) * r2]; 
    Nu = (Zm - Ze); 
    dq = [K * Nu; 1]; 

    % State update and 
    qe = qp + XiMat( qp ) * dq(1:3); 
    qe = qe / norm(qe); 
    
    % Covariance update
    Pe = (eye(3,3) - K * H) * Pp;
    
    qestK(k,:) = qflip( qe ); 
        
end

figure(3)
subplot(1,3,1)
plot( rad2deg( quat2eul( qS ) ) )
legend('yaw', 'pitch', 'roll')
% plot( theta, qref )
% legend('w','x','y','z')
title('Quaternion Ref')

subplot(1,3,2)
plot( rad2deg( quat2eul( qestK ) ) )
legend('yaw', 'pitch', 'roll')
% plot( theta, qestT )
% legend('w','x','y','z')
title('Quaternion Est')

subplot(1,3,3)
plot( rad2deg( quat2eul( qestK - qS ) ) )
title('Error')


