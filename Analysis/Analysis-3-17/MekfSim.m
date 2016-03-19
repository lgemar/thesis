%% Create a data set of quaternions

N = 100; % Number of samples
dt = 0.02; % Simulation time step
t = dt * (1:1:N); 
theta = linspace(0, 4*pi, N); % Sweep of angles (in the world frame)
theta = sin(theta); 
n = [1; 5; 15]; % Axis of rotation (in the world frame)
n = n / norm(n);

% Reference vectors
g = [0; 0; 1]; % world coordinates
b = [1; 0; 0]; % world coordinates

% Simulation variables
qref = zeros(N,4); % quaternion reference array
qestT = zeros(N,4); % Triad estimates
qestK = zeros(N,4); % Kalman filter estimates
z = zeros(N,6); % sensor measurements
omega = zeros(N,3); 

for i = 1:length(theta)
    
    % Process noise
    muProc = [0 0 0]'; 
    sigmaProc = diag([3e-5 3e-5 3e-5]'); 
    r = mvnrnd(muProc,sigmaProc)'; 

    % Measurement noise
    muMeas = [0 0 0]'; 
    sigmaMeas = diag([0.09 0.09 0.09]'); 
    w = mvnrnd(muMeas,sigmaMeas)';
    
    % Find the reference quaternion
    qr = [ sin(theta(i) / 2) * n; cos(theta(i) / 2) ]; % world to body quaternion, where n is the normal of rotation in the world frame
    % qr = qr + [r; 1]; qr = qr / norm(qr); % add noise and renormalize
    qref(i,:) = qflip( qr ); % body to world referenced vector
    
    % Generate the sensor measurements
    R = AMat( qr ); % Attitude matrix of the body-to-world quaternion - a rotation from world to body
    z(i,1:3) = ((R * g) + w)'; % gravity vector in body coordinates
    z(i,4:6) = ((R * b) + w)'; % b-field vector in body coordinates
    
    % Generate the gyro input measurement
    if( i > 1 ) 
        qdiff = qmultiply( qr, qinv( qflip( qref(i-1,:) ) ) );
        qdiff(1:3) = qdiff(1:3); 
        omega(i,:) = (qdiff(1:3) / dt + r)'; 
    end
end

figure(1)
subplot(2,2,1)
plot( rad2deg(theta), rad2deg( quat2eul( qref ) ) )
title('Quaternion Ref')
legend('yaw', 'pitch', 'roll')
subplot(2,2,2)
plot( t, omega )
title('Omega Input')
legend('X', 'Y', 'Z')
subplot(2,2,3)
plot( rad2deg(theta),z(:,1:3) )
title('Accelerometer')
ylim([-1.1 1.1])
legend('X', 'Y', 'Z')
subplot(2,2,4)
plot( rad2deg(theta),z(:,4:6) )
title('B Field')
ylim([-1.1 1.1])
legend('X', 'Y', 'Z')


%% Run the TRIAD algorithm over the data set

for i = 1:size(qref,1)
    
    mg = z(i,1:3)'; % measurement of g vector in body
    mb = z(i,4:6)'; % measurement of b vector in body
    
    qe =  triadFun(g,mg,b,mb,eye(3,3)); % column-major quaternion 
    qestT(i,:) = qflip( qe ); % row-major quaternion estimate
    
end

figure(2)
subplot(1,2,1)
plot( rad2deg(theta), rad2deg( quat2eul( qref ) ) )
legend('yaw', 'pitch', 'roll')
% plot( theta, qref )
% legend('w','x','y','z')
title('Quaternion Ref')

subplot(1,2,2)
plot( rad2deg(theta), rad2deg( quat2eul( qestT ) ) )
legend('yaw', 'pitch', 'roll')
% plot( theta, qestT )
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
subplot(1,2,1)
plot( rad2deg(theta), rad2deg( quat2eul( qref ) ) )
legend('yaw', 'pitch', 'roll')
% plot( theta, qref )
% legend('w','x','y','z')
title('Quaternion Ref')

subplot(1,2,2)
plot( rad2deg(theta), rad2deg( quat2eul( qestK ) ) )
legend('yaw', 'pitch', 'roll')
% plot( theta, qestT )
% legend('w','x','y','z')
title('Quaternion Est')




