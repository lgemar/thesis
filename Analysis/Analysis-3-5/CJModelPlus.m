%% Read in Data
current_folder = pwd; 
data_folder = ([pwd, '\Data']); 
file_name = 'test2test2.csv'; 
params1 = [277.17 276.8257 325.7718 253.6234]; % fx fy cx cy, tests 1-5, 7
dC = 25; % (mm), object size ... not sure if this is right

M = csvread([data_folder, '\', file_name]); 
M = M(2:2:end, :); N = size(M, 1);

% Time vector
t = M(:, 1); t = t - min(t); 
QuatRef = M(:, 21:24); % (w, x, y, z)
% position measurements from the undistorted image, image center already subtracted off
pU = M(:, 2:4)'; 
aS = M(:, 9:11)'; 

% Intrinsic camera parameters
params = params1; 
fx = params(1); 
fy = params(2); 
cx = params(3); cy = params(4); 
% Extrinsic camera parameters
R1 = [1 0 0; 0 0 -1; 0 1 0]; 
R2 = [0 1 0; -1 0 0; 0 0 1];
Ralign = R2 * R1; 
Talign = [38.2; -18.3; 490.5]; 
% Direct observations: position of the object in camera coordinates
zC = dC ./ (pU(3,:) * sqrt( 1/fx^2 + 1/fy^2 )); 
yC = zC .* pU(2,:) / fy; 
xC = zC .* pU(1,:) / fx; 
pC = cat(1, xC, yC, zC); 
% position of the object in world coordinates
pW = Ralign * pC + repmat(Talign, 1, size(pC, 2)); 

% Multiply by rotation matrix from body to world
sTb = [0 1 0; -1 0 0; 0 0 1]; 
bTs = sTb'; % rotation matrix from the sensor to the body
aW = zeros(size(aS)); 
for i = 1:N
    wRb = quat2rotm(QuatRef(i, :)); 
    aW(:, i) = wRb * bTs * aS(:, i); 
end

aW(3, :) = aW(3, :) - 9.81; 

%% Initialize Kalman Filter

% Initalize Variables
T = t(2) - t(1); 
Sr = 50; Sq = 1; 
R = Sr * T * eye(6,6); 
Q = Sq * T * eye(3,3); 

% State transition and observation matrices
F2 = [1 T T^2/2 T^3/6; 0 1 T T^2/2; 0 0 1 T; 0 0 0 1]; A = blkdiag(F2, F2, F2); 
H2 = [1 0 0 0; 0 0 1 0]; 
H = blkdiag(H2, H2, H2); 

%% Run Kalman Filter

% Initialization
x = zeros(12, N); 
x(:, 1) = [pW(1, 1) 0 aW(1,1) 0 pW(2, 1) 0 aW(2,1) 0 pW(3,1) 0 aW(3,1) 0]'; % start at 0 velocity
P = 1 * T * eye(12,12);

for i = 2:N
    % Update transition matrix
    T = t(i) - t(i-1); 
    F2 = [1 T T^2/2 T^3/6; 0 1 T T^2/2; 0 0 1 T; 0 0 0 1]; A = blkdiag(F2, F2, F2); 
    G2 = [T^3/6; T^2/2; T; 1]; G = blkdiag(G2, G2, G2); 

    % Prediction 
    Ppred = A * P * A' + G * Q * G'; 
    xpred = A * x(:, i-1);  
    
    % Update
    P = inv( inv(Ppred) + H' * R' * H ); 
    K = P * H' * R'; 
    x(:, i) = xpred + K * ([pW(1, i); aW(1,i); pW(2,i); aW(2, i); pW(3,i); aW(3,i)]  - H * xpred); 
end

figure(1)

subplot(1,4,1)
plot(t, aW')
title('Acceleration Measurement')
xlabel('Time stamp (s)')
ylabel('Position (m / s^2)')

subplot(1,4,2)
plot(t, pW')
title('State observations (World)')
xlabel('Time stamp (s)')
ylabel('Position (mm)')
legend('{x}_{w}', 'y_{w}', 'z_{w}')
xlim([min(t) max(t)])
ylim([-750 750])

subplot(1,4,3)
plot(t, x([1 5 9], :)')
title('State estimates (world)')
xlabel('Time stamp (s)')
ylabel('Position (mm)')
xlim([min(t) max(t)])
ylim([-750 750])

subplot(1,4,4)
plot(t, 1000*M(:, 18:20))
title('State absolute')
xlabel('Time stamp (s)')
ylabel('Position (mm)')
xlim([min(t) max(t)])
ylim([-750 750])