%% Read in data samples into a matrix
current_folder = pwd; 
data_folder = ([pwd, '\Data']); 
file_name = 'pitch1.csv'; 
M = csvread([data_folder, '\', file_name]); 
M = M(2:end, :); 
N = size(M, 1); % Number of samples

%% True of tests collected on about 3/5
AccelData = M(:, 9:11)'; 
GyroData = M(:, 12:14)';
MagnData = M(:, 15:17)'; 
QuatRef = M(:, 21:24); % (w, x, y, z)
T = [0 1 0; -1 0 0; 0 0 1]; % sensor alignment matrix from body to sensor
% Time vector
t = M(:, 1); t = t - min(t); 

%% True of yaw,pitch,roll tests collected on 3/15
QuatRef = M(:, 2:5); % (w, x, y, z)
AccelData = M(:, 6:8)'; 
GyroData = M(:, 9:11)'; 
MagnData = M(:, 12:14)'; 
T = [1 0 0; 0 1 0; 0 0 1]; % sensor alignment matrix from body to sensor
% Time vector
t = M(:, 1) / 1000; t = t - min(t); 

%%

% Output allocation

NTrials = 50; 
ad = linspace(0,2*pi,NTrials); 
    
gw = [0; 0; 9.81];
bw = [cos(1.7)*19.5; sin(1.7)*(-5.2); -48.2]; 
%bw = [cos(ad(i))*19.5; sin(ad(i))*-5.2; -48.2]; 

QuatEst = zeros(size(QuatRef)); 

% Initalize Kalman Filter Variables
dT = t(2) - t(1); 
Sr = 100; Sq = 1; % trust measurements, trust model
Q = Sq * dT * eye(3,3); 
R = Sr * dT * eye(6,6); 

% Intialization of covariance estimate (arbitrary?): Pe
Pe = dT * eye(3,3); 

% Compute initial state vector using TRIAD algorithm
v1 = gw; v2 = bw; 
v1 = v1 / norm(v1); 
v2 = v2 / norm(v2); 
r1 = v1; 
r2 = cross(v1, v2) / norm(cross(v1, v2)); 
r3 = cross(r1, r2); 
m1 = AccelData(:, 1); % w1 = T * w1; 
m2 = MagnData(:, 1); % w2 = T * w2; 
m1 = m1 / norm(m1);
m2 = m2 / norm(m2);
s1 = m1; 
s2 = cross(m1, m2) / norm(cross(m1, m2)); 
s3 = cross(s1, s2); 
M_r = cat(2, r1, r2, r3); 
M_s = cat(2, s1, s2, s3);
A = (T' * M_s) * M_r';
q = rotm2quat(A'); 

% Initial State Estimate (Qe)
Qe = [q(2); q(3); q(4); q(1)]; 
QuatEst(1, :) = [Qe(4), Qe(1), Qe(2), Qe(3)]; 

% Check the observation model accuracy
pS1 = AccelData(:,1); 
pS2 = MagnData(:,1); 
Z = [ pS1; pS2 ]; 
Ze = [ T * AMat( Qe ) * gw ; T * AMat( Qe ) * bw]; 

% State matrices
Rsd = []; 
QpRsd = []; 
QeRsd = []; 
ThetaIn = []; 

for k = 2:N
    % Time step update
    dT = t(k) - t(k-1); 
    
    if( dT > 0 )

        % Covariance prediction (Pp)
        Pp = Pe + 0.25 * Q ;

        % State prediction (Qp)
        DeltaTheta = GyroData(:,k) * dT; 
        Alpha = cos( norm(DeltaTheta)/2 ); Beta = sin( norm(DeltaTheta)/2 ) / norm( DeltaTheta ); 
        Qp = Alpha * Qe + Beta * XiMat( Qe ) * ([DeltaTheta(1:2); -DeltaTheta(3)]); 
        Qp = Qp / norm( Qp ); 
        
        % Save state from the prediction step
        ThetaIn = [ThetaIn DeltaTheta]; 
        QpRsd = [QpRsd Beta*XiMat( Qe )*DeltaTheta];

        % Sensitivity matrix
        v1 = gw; v2 = bw; v1 = v1 / norm(v1); v2 = v2 / norm(v2); 
        pB1 = AMat( Qp ) * v1; pB2 = AMat( Qp ) * v2;
        H = [2*T*CrossMat(pB1); 2*T*CrossMat(pB2)]; 

        % Kalman gain
        K = Pp * H' / (H * Pp * H' + R);

        % Observation, Prediction error
        pS1 = AccelData(:,k) / norm(AccelData(:,k)); 
        pS2 = MagnData(:,k) / norm(MagnData(:,k)); 
        Z = [ pS1; pS2 ]; 
        Ze = [ T * AMat( Qp ) * v1 ; T * AMat( Qp ) * v2]; 
        Nu = (Z - Ze); 
        dq = [K * Nu; 1]; 

        % State update and prediction update
        Qe = Qp + XiMat( Qp ) * dq(1:3); Qe = Qe / norm(Qe); 
        Pe = (eye(3,3) - K * H) * Pp;
        
        % Save state from the update step
        Rsd = [Rsd Nu]; % residuals
        QeRsd = [QeRsd Beta*XiMat( Qp )*dq(1:3)];
        QuatEst(k, :) = [Qe(4), Qe(1), Qe(2), Qe(3)]; 
    end
end

f = figure(1);

subplot(1,3,1)
DegEst = rad2deg( quat2eul( cat(2, QuatEst(:,1), QuatEst(:, 2:4) ) ) );
plot( DegEst )
ylim([-180 180])
legend('\theta_Z', '\theta_Y', '\theta_X')
% plot( QuatEst )
% legend('w', 'x', 'y', 'z')
title('Orientation Estimate: $\hat{\vec{\theta}}$', 'Interpreter', 'Latex')

subplot(1,3,2)
DegRef = rad2deg( quat2eul( cat(2, QuatRef(:,1), QuatRef(:, 2:4) ) ) );
plot( DegRef )
ylim([-180 180])
legend('\theta_Z', '\theta_Y', '\theta_X')
% plot(QuatRef)
% legend('w', 'x', 'y', 'z')
title('Orientation Reference: $\vec{\theta}$', 'Interpreter', 'Latex')

subplot(1,3,3)
plot(t, DegEst - DegRef )
str = 'Error: $\epsilon = \hat{\vec{\theta}} - \vec{\theta}$'; 
title(str, 'Interpreter', 'Latex')
xlabel('Time stamp (s)')
ylabel('Error (degrees)', 'Interpreter', 'Latex')
legend('\epsilon_{\theta_Z}', '\epsilon_{\theta_Y}', '\epsilon_{\theta_X}')

f = figure(2);
subplot(2,3,1)
plot( rad2deg( ThetaIn' ) )
title( 'Gyro Input ' )
legend('x','y','z')
subplot(2,3,2)
plot( rad2deg( quat2eul(QpRsd') ) )
title( 'Prediction Residual (degrees)' )
legend('x','y','z')
subplot(2,3,3)
plot( Rsd(1:3,:)' )
title( 'Prediction Error (Accelerometer)' )
legend('x','y','z')
subplot(2,3,4)
plot( Rsd(4:6,:)' )
title( 'Prediction Error (Magnetometer)' )
legend('x','y','z')
subplot(2,3,5)
plot( rad2deg( quat2eul(QeRsd') ) )
title( 'Filtering Residual (degrees)' )


