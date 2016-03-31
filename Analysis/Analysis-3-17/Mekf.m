% set(groot,'defaultAxesColorOrder',[1 0 0;0 1 0;0 0 1],...
%       'defaultAxesLineStyleOrder','-|--|:')

%% Test Data
% Specify the test name and file location
AllTestNames = {'yaw5-1', 'pitch5-1', 'roll5-1', 'roll1', 'pitch1', 'yaw1', 'no_manuever1', 'rollslow1', 'yawslow1', 'all2', 'yawstep2'}; 
TestName = AllTestNames{1}; % specify the trial test name
SensorFile = [TestName, '.csv']; % find the sensor data file

DataFolder = (['C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-17\', 'Data\Sensor\']); 
SensorData = csvread([DataFolder, SensorFile]); SensorData = SensorData(2:end, :); % read the sensor data

% Number of data samples
N = size(SensorData,1);
 
% Clock and timing variables
tSensor = SensorData(:,1); tSensor = tSensor/1000; % it's in milliseconds - convert to seconds
t = tSensor - min(tSensor); % align the time vec with the beginning of the trial
dt = mean(diff(tSensor)); % Simulation time step

% Observations
a = SensorData(:,6:8); % accelerometer observations
g = SensorData(:,9:11); % gyro measurements
m = SensorData(:,12:14); % magnetometer observations
z = [a, m]; % sensor observations
qr = SensorData(:,2:5); % quaternion reference array
tr = fliplr(rad2deg( quat2eul( qr ) )); 
    
% Simulation variable allocation
qet = zeros(N,4); % Triad estimates
qek = zeros(N,4); % Kalman filter estimates
dqt = zeros(N,4); % TRIAD quaternion error
dqk = zeros(N,4); % Kalman quaternion error

% Reference vectors
th = deg2rad(165); % location of magnetic North, typical: th = deg2rad(160); 
gr = [0; 0; 9.81]; % gravity reference vector
br = [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1] * m(1,:)'; % b-field reference vector

% Build strings of important notation
tstr = 'Orientation Reference, $\vec{\theta}$'; 
testr = 'Orientation Estimate, $\hat{\vec{\theta}}$';
dtstr = 'Esimation Error, $\delta\vec{\theta}$'; 
dtxstr = '$\delta\vec{\theta}_x$'; 
dtystr = '$\delta\vec{\theta}_y$'; 
dtzstr = '$\delta\vec{\theta}_z$'; 

% Display the raw data
figure(1)

subplot(2,2,1)
plot(t,tr)
title('Orientation Reference')
xlabel('time (s)')
s = sprintf('%c', char(176)); xlabel('time (s)'); 
ylabel(s)
legend('roll', 'pitch', 'yaw')

subplot(2,2,2)
plot(t,a)
title('Accelerometer')
ylim([-10 10])
xlabel('time (s)')
ylabel('m/s^2')
legend('X', 'Y', 'Z')

subplot(2,2,3)
plot(t,g)
title('Gyroscope')
xlabel('time (s)')
s = sprintf('%c/s', char(176)); xlabel('time (s)'); 
ylabel(s)
legend('X', 'Y', 'Z')

subplot(2,2,4)
plot(t,m)
title('Magnetometer')
xlabel('time (s)')
ylabel('\mu T')
legend('X', 'Y', 'Z')



%% TRIAD algorithm

% Compute the quaternion estimates
for i = 1:size(qr,1)
    qe =  triadFun(gr,a(i,:)',br,m(i,:)',eye(3,3)); % column-major quaternion 
    qet(i,:) = qflip( qe ); % row-major quaternion estimate, row vectors are written as (w,x,y,z) quaternions
end
tet = fliplr(rad2deg( quat2eul(qet) )); 

% Compute the quaternion error of estimates
for i = 1:N
    dqt(i,:) = qflip( qmultiply( qflip( qr(i,:) ), qinv( qflip( qet(i,:) ) ) ) ); 
end
dtt = rad2deg(2 * dqt(:,2:4)); 

% Compute bias and variance of the error
mudtt = mean(dtt,1); % bias - mean of the error
Pdtt = cov(dtt,1); % compute covariance of error, rows are observations
biasdtt = norm(mudtt);  
vardtt = trace(Pdtt);  % target variance is 5^2 + 5^2 + 5^2, for a 5degree change in orientation is unnoticable

% Mean-squared magnitude of the unbiased error, three possible calculations
msme1 = trace(Pdtt);
[Q, D] = eig(Pdtt); msme2 = trace(D); 
msme3 = mean( sum( (dtt - repmat(mudtt,N,1)).^2, 2) ); 

% Trace comparison
figure(2)

subplot(1,2,1)
plot( t, tr )
title(tstr, 'Interpreter', 'Latex')
ylim([-181 181])
legend('Roll', 'Pitch', 'Yaw')

subplot(1,2,2)
plot( t, tet )
title(testr, 'Interpreter', 'Latex')
ylim([-181 181])
legend('Roll', 'Pitch', 'Yaw')

% Error for x,y,z
figure; 
plot( t, dtt)
legend('\delta\theta_x', '\delta\theta_y', '\delta\theta_z')
title(['TRIAD ', dtstr], 'Interpreter', 'Latex') 
xlabel('Time (s)', 'Interpreter', 'Latex')
ylabel('degrees ($^{\circ})$', 'Interpreter', 'Latex')
dttrange = [-max(dtt(:))-10 max(dtt(:))+10];
ylim(dttrange)

biasstr = ['$$ | E[\delta\vec{\theta}] | = ', num2str(biasdtt), ' $$'];
text(0.05 * max(t),dttrange(2)-10,biasstr,'Interpreter','latex')
varstr = ['$$ \textrm{tr}(\textrm{Var}(\delta\vec{\theta})) = ', num2str(vardtt), ' $$'];                                                
text(0.05 * max(t),dttrange(2)-20,varstr,'Interpreter','latex')

figure; 

xyzstr = {'x', 'y', 'z'}; 
rpystr = {'Roll', 'Pitch', 'Yaw'};
colorstr = {'r', 'g', 'b'}

for j = 1:3
    subplot(2,2,j+1)
    plot(t, dtt(:,j), colorstr{j})
    title(['$ E[\delta\theta_', xyzstr{j}, '] = ', num2str(mudtt(j), '%.1f'), ...
        '^{\circ} $ and ', '$ \textrm{Var}(\delta\theta_', xyzstr{j}, ')) = ('....
        , num2str(sqrt(Pdtt(j,j)), '%.1f'), '^{\circ})^2 $'], 'Interpreter', 'Latex')
    ylim([-181 181])
    xlim([min(t) max(t)])
end

% Trace and error comparison maneuver by maneuver
for j = 1:3
    figure(4+j)
    
    subplot(1,2,1)
    plot(t, tr(:,j), t, tet(:,j))
    title(['Reference Trajectory, \theta_', xyzstr{j}, ...
        'Versus Estimate, \hat{\theta_', xyzstr{j}, '}'])
    legend('Reference', 'Estimate')
    
%     subplot(1,2,2)
%     title(['Reference Trajectory, \delta\vec{\theta}', ...
%     'Versus Estimate, \delta\hat{\vec{\theta}}'])
end



%% Run the MEKF over the data set

% Compute initial state vector using TRIAD algorithm
mg = z(1,1:3)'; % measurement of g vector in body
mb = z(1,4:6)'; % measurement of b vector in body
qe = triadFun(gr,mg,br,mb,eye(3,3)); % column-major quaternion 

% Initalize the covariance matrix, process noise, and measurement noise
Sp = [deg2rad(10)^2 deg2rad(10)^2 deg2rad(15)^2]; Pe = diag(Sp); 
% These two work really well: 
% Sq = 20*(1/3)*[2.8e-6 2.8e-6 12*2.8e-6]'; Q = diag(Sq); 
% Sr = 0.001*(1/3)*[1.1e-6*ones(1,3) 17*[1,1,1]]'; R = diag(Sr); 

% These two may work even better: 
Sq = 20*(1/3)*[2.8e-6 2.8e-6 12*2.8e-6]'; Q = diag(Sq); 
Sr = [(1)^2 * 1.1e-6*ones(1,3) (0.0008)^2*17*[1,1,1]]'; R = diag(Sr); 

% % These two may work even better: 
% Sq = [2.8e-6 2.8e-6 12*2.8e-6]'; Q = diag(Sq); 
% Sr = [(1) * 1.1e-6*ones(1,3) (Sra(idx))*17*[1,1,1]]'; R = diag(Sr); 

% Gyro gain and gyro gain reference
pgain = zeros(N-1,3); 
pgainref = zeros(N-1,3); 
inngain = zeros(N-1,3);

for k = 2:N

    % Covariance prediction (Pp)
    Pp = Pe + Q;

    % State prediction (Qp)
    % DeltaTheta = deg2rad(g(k,:))'; 
    DeltaTheta = g(k,:)' * dt; % turns out the output of this sensor is rad / s
    Alpha = cos( norm(DeltaTheta)/2 ); 
    if( norm(DeltaTheta) > 0 )
        Beta = sin( norm(DeltaTheta)/2 ) / norm( DeltaTheta ); 
    else
        Beta = 0.5; 
    end
    
    % OR:  qp = Alpha * qe + 0.7 * XiMat( qe ) * DeltaTheta;
    qp = [XiMat( qe ) qe] * [(0.5 * DeltaTheta); Alpha]; 
    qp = qp / norm( qp ); 
    
    % Store gyro input, estimation and reference
    dqp = qmultiply( qp, qinv( qe ) ); 
    pgain(k,:) = 2*dqp(1:3)'; 
    
    dqpref = qmultiply( qflip( qr(k,:) ), qinv( qflip( qr(k-1,:) ) ) ); 
    pgainref(k,:) = 2*dqpref(1:3)'; 

    % Sensitivity matrix
    r1 = gr; r2 = br; 
    pB1 = AMat( qp ) * r1; pB2 = AMat( qp ) * r2;
    H = [ 2*CrossMat( pB1 ); 2*CrossMat( pB2 )]; 

    % Kalman gain
    K = Pp * H' / (H * Pp * H' + R);

    % Observation, Prediction error
    Zm = z(k,:)'; 
    Zm(1:3) = Zm(1:3) / norm(Zm(1:3)); 
    Zm(4:6) = Zm(4:6) / norm(Zm(4:6)); 
    Ze = [ AMat( qp ) * r1 ; AMat( qp ) * r2]; 
    Ze(1:3) = Ze(1:3) / norm(Ze(1:3)); 
    Ze(4:6) = Ze(4:6) / norm(Ze(4:6)); 
    Nu = (Zm - Ze); 
    dq = [K * Nu; 1]; 
    
    % Store innovation gain
    inngain(k,:) = 2*dq(1:3)'; 

    % State update and 
    qe = qp + XiMat( qp ) * dq(1:3); 
    qe = qe / norm(qe); 
    
    % Covariance update
    Pe = (eye(3,3) - K * H) * Pp;
    
    qek(k,:) = qflip( qe ); 
        
end
tek = fliplr(rad2deg( quat2eul(qek) )); 

% Compute the quaternion error of estimates
for i = 1:N
    dqk(i,:) = qflip( qmultiply( qflip( qr(i,:) ), qinv( qflip( qek(i,:) ) ) ) ); 
end
dtk = rad2deg(2 * dqk(:,2:4)); 

% Compute bias and variance of the error
mudtt = mean(dtk,1); % bias - mean of the error
Pdtt = cov(dtk,1); % compute covariance of error, rows are observations
biasdtt = norm(mudtt);  
vardtt = trace(Pdtt);  % target variance is 5^2 + 5^2 + 5^2, for a 5degree change in orientation is unnoticable

figure(3)
subplot(2,3,1)
plot( t, rad2deg( quat2eul( qr ) ) )
legend('yaw', 'pitch', 'roll')
% plot( theta, qr )
% legend('w','x','y','z')
title('Reference')
ylim([-181 181])

subplot(2,3,2)
plot( t, rad2deg( quat2eul( qek ) ) )
legend('yaw', 'pitch', 'roll')
% plot( theta, qet )
% legend('w','x','y','z')
title('Estimate')
ylim([-181 181])

subplot(2,3,3)

plot( t, rad2deg( quat2eul( dqk ) ) )
legend('yaw', 'pitch', 'roll')
% plot( theta, qet )
% legend('w','x','y','z')
title('Error')
ylim([-40 40])

subplot(2,3,4)
plot(t, pgain)
title('Prediction gain: $\delta \hat{\vec{\theta}}{^{(-)}}_k$', 'Interpreter', 'Latex')
ylabel('Rad')

subplot(2,3,5)
plot(t, pgainref)
title('True prediction gain: $\delta \vec{\theta}{^{(-)}}_k$', 'Interpreter', 'Latex')
ylabel('Rad')

subplot(2,3,6)
plot(t, inngain)
title('Innovation gain: $\delta \vec{\theta}{^{(+)}}_k$', 'Interpreter', 'Latex')
ylabel('Rad')
plot( t, rad2deg( quat2eul( qek ) ) )

% Error for x,y,z
figure; 
plot( t, dtk)
legend('\delta\theta_x', '\delta\theta_y', '\delta\theta_z')
title(['Extended Kalman Filter ', dtstr], 'Interpreter', 'Latex') 
dttrange = [-max(dtt(:))-10 max(dtt(:))+10];
ylim(dttrange)

biasstr = ['$$ | E[\delta\vec{\theta}] | = ', num2str(biasdtt), ' $$'];
text(0.05 * max(t),dttrange(2)-10,biasstr,'Interpreter','latex')
varstr = ['$$ \textrm{tr}(\textrm{Var}(\delta\vec{\theta})) = ', num2str(vardtt), ' $$'];                                                
text(0.05 * max(t),dttrange(2)-20,varstr,'Interpreter','latex')


% plot(t, tek(:,3))
% 
% title('Measurement Noise Tuning', 'Interpreter', 'Latex')
% ylim([-181 181])
% xlim([4.5 6])
% xlabel('Sensor time (s)', 'Interpreter', 'Latex')
% ylabel('degrees $(^{\circ})$', 'Interpreter', 'Latex')
% h = legend('$R^{\prime} = \frac{1}{2} R$', '$R^{\prime} = R$', '$R^{\prime} = 2 R$')
% set(h,'Interpreter','latex')