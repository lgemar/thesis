current_folder = pwd; 
data_folder = ([pwd, '\Data']); 
file_name = 'test1test1.csv'; 
M = csvread([data_folder, '\', file_name]); 
M = M(2:end, :); 
N = size(M, 1); % Number of samples

% Data vectors
QuatRef = M(:, 21:24)'; % (w, x, y, z)
AccelData = M(:, 9:11)'; 
GyroData = M(:, 12:14)'; 
MagnData = M(:, 15:17)'; 

% Output allocation
QuatEst = zeros(size(QuatRef)); 

% Time vector
t = M(:, 1); t = t - min(t); 

% Reference vectors in world coordinates
gw = [0; 0; 9.81]; % gravity vector
bw = [0.3; -1; 0]; % magnetic field vector (unsure)
T = [0 1 0; -1 0 0; 0 0 1]; % sensor alignment matrix from body to sensor

% Initalize Kalman Filter Variables
dT = t(2) - t(1); 
Sr = 10; Sq = 1; % trust measurements, trust model
Q = Sq * dT * eye(3,3); 
R = Sr * dT * eye(6,6); 

% Intialization of covariance (arbitrary)
P = dT * eye(3,3); 

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
q = rotm2quat(A); 

% Intialize state
QuatEst(:, 1) = [q(2), q(3), q(4), q(1)]; 

for k = 2:N
    % Time step update
    dT = t(k) - t(k-1); 
    
    % Covariance and state prediction
    P_minus = P + 0.25 * Q; 
    qX = [0 -QuatEst(3,k-1) QuatEst(2,k-1); QuatEst(3,k-1) 0 -QuatEst(1,k-1); -QuatEst(2,k-1) QuatEst(1,k-1) 0]; 
    q_minus = QuatEst(:, k-1) + [ (QuatEst(4,k-1) * eye(3,3) + qX); -QuatEst(1:3,k-1)' ] * (0.5 * GyroData(:,k) * dT); 
    q_minus = q_minus / norm(q_minus); 

    % Sensitivity matrix
    pB_g = quat2rotm([q_minus(4) q_minus(1:3)']) * (gw/norm(gw)); 
    pB_b = quat2rotm([q_minus(4) q_minus(1:3)']) * (bw/norm(bw));
    vX1 = [0 -pB_g(3) pB_g(2); pB_g(3) 0 -pB_g(1); -pB_g(2) pB_g(1) 0]; 
    vX2 = [0 -pB_b(3) pB_b(2); pB_b(3) 0 -pB_b(1); -pB_b(2) pB_b(1) 0]; 
    H = [-2*T*vX1; -2*T*vX2]; 
    
    % Kalman gain
    K = P_minus * H' / (H * P_minus * H' + R); 
    
    % Prediction error and gain
    a = AccelData(:,k) / norm(AccelData(:,k)); 
    b = MagnData(:,k) / norm(MagnData(:,k)); 
    z = [a; b]; 
    z_est = [ T*quat2rotm([q_minus(4) q_minus(1:3)'])*(gw/norm(gw)) ; T*quat2rotm([q_minus(4) q_minus(1:3)'])*(bw/norm(bw))]; 
    dq_vec = K * (z - z_est); 
    dq = [dq_vec; 1]; 
    
    % State update and prediction update
    QuatEst(:,k) = q_minus + [q_minus(4) -q_minus(3) q_minus(2); q_minus(3) q_minus(4) -q_minus(1); -q_minus(2) q_minus(1) q_minus(4); -q_minus(1) -q_minus(2) -q_minus(3)] * dq_vec;
    P = (eye(3,3) - K * H) * P_minus;
end

figure(1)
subplot(1,2,1)
plot(rad2deg(quat2eul([QuatEst(4,:)' -QuatEst(1:3,:)'])))
legend('Yaw', 'Pitch', 'Roll')
% plot(QuatEst)
% legend('w', 'x', 'y', 'z')
title('Tool Orientation (Estimate)')
subplot(1,2,2)
plot(rad2deg(quat2eul(cat(2, QuatRef(1,:)', QuatRef(2:4, :)'))))
legend('Yaw', 'Pitch', 'Roll')
% plot(QuatRef)
% legend('w', 'x', 'y', 'z')
title('Tool Orientation (Absolute)')


