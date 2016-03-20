%% Read and display the raw data vectors
CurrentFolder = pwd; 
DataFolder = (['C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-17\', 'Data\']); 
AllTestNames = {'TsTest', 'ClockAlign', 'ClockAlign2', 'horzslow', 'horzmed', ...
                'horzfast', 'depthslow', 'depthmed', 'depthfast', 'vertslow', ...
                'vertmed', 'vertfast', 'depthveryfast', 'disthorz', 'disthorz1'...
                'disthorz2', 'distdepth', 'distdepth2', 'distvert1', 'distver2'...
                'clockalign3', 'failure1'}; 
ViconTestNames = {'TsTest', 'ClockAlign', 'ClockAlign2', 'AllTests', ...
                  'ClockAlign3', 'FailureTests'}; 

% Read Unity Data file
UnityTestName = AllTestNames{4}; 
UnityFile = [UnityTestName, '.csv']; 
UnityData = csvread([DataFolder, UnityFile]); UnityData = UnityData(2:end, :); 

% Read VICON data file
ViconTestName = ViconTestNames{4}; 
ViconFile = [ViconTestName, '_vicon.csv']; 
ViconDataMat = dlmread([DataFolder, ViconFile], ','); ViconDataMat = ViconDataMat(2:end, :); 

% Distance between UNIX time and the start of the app
dtva = 1.458245808409409e+09; 

% Align the time vectors relative to the clock on the sensor
tA = UnityData(:,1); 
tV = ViconDataMat(:,1) - (dtva + min(tA)); % Transform vicon time to app time
tA = tA - min(tA); % Transform app time to 0
N = length(tA); 

% Read the position and acceleration measurement vectors
pV = ViconDataMat(:,2:4) * 1000; % (x,y,z) in world coordinates (mm)
pA = UnityData(:,2:5); % (x,y,w,h) in distorted image coordinates
aW = UnityData(:,11:13); % (a_x, a_y,a_z) in body coordinates

% Compute sample rates and frequencies
Ts = (max(tA) - min(tA)) / length(tA); disp(['Sensor sample rate: ', num2str(1/Ts), ' Hz']); 
Tv = (max(tV) - min(tV)) / length(tV); disp(['Vicon sample rate: ', num2str(1/Tv), ' Hz']); 
Fv = 1/Tv; 
Fs = 1/Ts; 

% Downsample the VICONdata
tV = linspace(min(tV), max(tV), ceil( length(tV) * round(Fs) / round(Fv) ))'; 
pV = resample(pV,round(Fs),round(Fv));

% Recompute the time and position vectors relative to the trial start times
tVpr = tV( tV > 0 & tV < max(tA) ); % tV' and pV' 
pVpr = pV( tV > 0 & tV < max(tA), : ); 

% Recompute the sample rate of the vicon data after resampling
Tv = (max(tVpr) - min(tVpr)) / length(tVpr); disp(['Resampled vicon sample rate: ', num2str(1/Tv), ' Hz']); 
Fv = 1/Tv; 

% Display the raw vicon, acceleration, and image data
figure(1)
subplot(1,3,1)
plot( tVpr, pVpr )
legend('x','y','z')
subplot(1,3,2)
plot( aW )
subplot(1,3,3)
plot( tA, pA )
legend('x', 'y')

%% Find undistorted measurement points

% Create the position vector for the distorted image
pD = [pA(:,1) pA(:,2) (pA(:,1)+pA(:,3)) (pA(:,2)+pA(:,4))]; % [xD1 yD1 xD2 yD2] == [ (tl=top left) (br=bottom right) ]

% Load the camera calibration session
load('calibrationSession.mat'); 

% Load the camera parameters
params = calibrationSession.CameraParameters; 

% Load the camera calibration matrix (CM)
CM = params.IntrinsicMatrix'; 
fx = CM(1,1); fy = CM(2,2); cx = CM(1,3); cy = CM(2,3); 

% Undistort the measurement points
pU = [undistortPoints(pD(:,1:2), params) undistortPoints(pD(:,3:4),params)];

%% Plot the distorted and undistorted measurement points (sanity check)
figure(1)
subplot(1,2,1)
plot( tA, pD(:,1:2) )
title('Distorted points')
legend('x','y')
subplot(1,2,2)
plot( tA, pU(:,1:2) )
legend('x', 'y')
title('Undistorted points')

%% Undistorted image, camera measurements, world measurements

% Undistorted image measurements
zU = [0.5*(pU(:,1)+pU(:,3)) 0.5*(pU(:,2)+pU(:,4)) 0.5*(pU(:,3)-pU(:,1) + pU(:,4)-pU(:,2))]; 

% Camera measurements
dC = 30 ./ ( zU(:,3) * sqrt( 1/fx^2 + 1/fy^2 ) ); % small blue ball is x mm
xC = (dC / fx) .* ( pU(:,1) - cx ); 
yC = (dC / fy) .* ( cy - pU(:,2) ); 

zC = [xC yC dC]; % (x,y,z), obeying the right hand rule

% Camera rotation matrix

ViconFile = 'camerarotm.csv'; 
ViconDataMat = dlmread([DataFolder, ViconFile], ','); 
ViconDataMat = ViconDataMat(2:end, :); 
qC = ViconDataMat(:,5:8);

S = [0 0 -1; -1 0 0; 0 1 0]; 
A = AMat( [ sin(atan(13/50)/2) * [0;1;0]; cos(atan(13/50)/2) ] ); % attitude of camera, aka rotm from world to camera
wRc = S * A;  

T = [43.2; -18.3; 476.8];  
zW = (wRc * zC' + repmat(T,1,size(zC,1)))';  

% Raw vicon data with world measurements
figure(2)

subplot(1,3,1)
plot( tVpr, pVpr )
legend('x','y','z')
ylim([-800 800])

subplot(1,3,2)
plot( tA, zW )
legend('x', 'y', 'z')
ylim([-800 800])

subplot(1,3,3)
plot( tA(1:length(pVpr),:), zW(1:length(pVpr),:) - pVpr )
legend('x', 'y', 'z')
ylim([-800 800])

%% Constant jerk estimation model: http://www.jneurosci.org/content/5/7/1688.full.pdf, for assumptinos

% Initalize Variables
dt = tA(2) - tA(1); % application time step
Sr = 1; Sq = 1000; 
R = Sr * dt * eye(6,6); 
Q = Sq * dt * eye(3,3); 

% State transition and observation matrices
F2 = [1 dt dt^2/2 dt^3/6; 0 1 dt dt^2/2; 0 0 1 dt; 0 0 0 1]; A = blkdiag(F2, F2, F2); 
H2 = [1 0 0 0; 0 0 1 0]; H = blkdiag(H2, H2, H2); 

% Run Kalman Filter

% Initialization
x = zeros(12, N); 
x(:, 1) = [zW(1,1) 0 aW(1,1) 0 zW(1,2) 0 aW(1,2) 0 zW(1,3) 0 aW(1,3) 0]'; % start at 0 velocity
P = 1 * dt * eye(12,12);

for i = 2:N
    % Update transition matrix
    dt = tA(i) - tA(i-1); 
    F2 = [1 dt dt^2/2 dt^3/6; 0 1 dt dt^2/2; 0 0 1 dt; 0 0 0 1]; A = blkdiag(F2, F2, F2); 
    G2 = [dt^3/6; dt^2/2; dt; 1]; G = blkdiag(G2, G2, G2); 

    % Prediction 
    Ppred = A * P * A' + G * Q * G'; 
    xpred = A * x(:, i-1);  
    
    % Update
    P = inv( inv(Ppred) + H' * R' * H ); 
    K = P * H' * R'; 
    x(:, i) = xpred + K * ([zW(i,1); aW(i,1); zW(i,2); aW(i,2); zW(i,3); aW(i,3)]  - H * xpred); 
end

figure(1)

subplot(1,4,1)
plot(tA, aW)
title('Acceleration Measurement')
xlabel('Time stamp (s)')
ylabel('Position (m / s^2)')

subplot(1,4,2)
plot(tA, zW)
title('State observations (World)')
xlabel('Time stamp (s)')
ylabel('Position (mm)')
legend('{x}_{w}', 'y_{w}', 'z_{w}')
xlim([min(tA) max(tA)])
ylim([-1000 1000])

subplot(1,4,3)
plot(tA, x([1 5 9], :)')
title('State estimates (world)')
xlabel('Time stamp (s)')
ylabel('Position (mm)')
xlim([min(tA) max(tA)])
ylim([-1000 1000])

subplot(1,4,4)
plot(tVpr, pVpr)
title('State absolute')
xlabel('Time stamp (s)')
ylabel('Position (mm)')
xlim([min(tVpr) max(tVpr)])
ylim([-1000 1000])