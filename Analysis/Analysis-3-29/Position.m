AllTestNames = {'test1', 'clckalign1', 'horz1', 'horz2', 'horz3', ... 
                'depth1', 'depth2', 'depth3', 'fbsquare1', 'fbsquare2', ...
                'fbcircle1', 'fbcircle2', 'vert1', 'vert2', 'vert3'}; 
ViconTestNames = {'test1', 'clkalign1', 'alltests'}; 
ViconTestName = ViconTestNames{3}; 
UnityTestName = AllTestNames{6}; 

% notes: 
% horz2 (4) produces amazing graphs of position estimation - horizontal
% error
% vert1 (13) vertical error analyais
% i believe depth1 is used for depth error analysis

%% Clock alignment
DataFolderPr = (['C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\', 'Data\']); 
UnityData = csvread([DataFolderPr, 'clckalign1.csv']); UnityData  = UnityData (2:end, :); 
ViconData = csvread([DataFolderPr, 'clkalign1_vicon.csv']); ViconData = ViconData(2:end, :); 

tv = ViconData(:,1); 
tA = UnityData(:,1); 

% Compute the sample rates for the Vicon, App, and Sensor sensors in two
% different ways
temp = mean(diff(tv),1); disp(['Vicon sample rate: ', num2str(1/temp), ' Hz']); 
temp = mean(diff(tA),1); disp(['App sample rate: ', num2str(1/temp), ' Hz']); 

Tv = (max(tv) - min(tv)) / length(tv); disp(['Vicon sample rate: ', num2str(1/Tv), ' Hz']); 
Ta = (max(tA) - min(tA)) / length(tA); disp(['App sample rate: ', num2str(1/Ta), ' Hz']); 

Fv = 1/Tv; 
Fa = 1/Ta;  

% Read the interesting data vectors
yD = -UnityData(:,3); 
zw = ViconData(:,4); 

% Resample the data
rV = linspace(min(tv), max(tv), ceil( length(tv) * round(Fa) / round(Fv) ))'; 
rzW = resample(zw,round(Fa),round(Fv));

% Smooth the data
yD = conv(yD, ones(10,1)/10, 'same'); 
rzW = conv(rzW, ones(10,1)/10, 'same'); 

% Compute the cross correlation of the downsampled vicon reference with sensor and app
[acorav,lag] = xcorr(yD - mean(yD),rzW - mean(rzW));
[~,I] = max(abs(acorav));
lav = lag(I); 
dtva = (rV(-lav) - tA(1))

% Align the time vectors relative to the clock in the application
rVpr = rV - (dtva + min(tA)); % Transform vicon time to app time
tApr = tA - min(tA); % Transform app time to 0

%% Camera calibration

% Define images to process
imageFileNames = {'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image1.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image2.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image3.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image4.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image5.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image6.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image7.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image8.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image9.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image10.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image11.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image12.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image13.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image14.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image15.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image16.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image17.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image18.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image19.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image20.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image21.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image22.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image23.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image24.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image25.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image26.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image27.png',...
    'C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-29\Data\Image28.png',...
    };

% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Generate world coordinates of the corners of the squares
squareSize = 22;  % in units of 'mm'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'mm', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', []);

fx = cameraParams.FocalLength(1); 
fy = cameraParams.FocalLength(2);
cx = cameraParams.PrincipalPoint(1); cx = 320; 
cy = cameraParams.PrincipalPoint(2); cy = 240;

%% Read in the UNITY and VICON data

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Sensor Measurements from Camera and IMU
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
UnityFile = [UnityTestName, '.csv']; 
UnityData = csvread([DataFolder, UnityFile]); UnityData = UnityData(2:end, :); 

t = UnityData(:,1); % time vector
pu = UnityData(:, 2:4); % position of rear of tool in undistorted image
a = UnityData(:,11:13); % accelerometer observations
g = UnityData(:,14:16); % gyro measurements
m = UnityData(:,17:19); % magnetometer observations
qc = UnityData(:,7:10); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% VICON position and orientation reference data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ViconFile = [ViconTestName, '_vicon.csv']; 
ViconDataMat = dlmread([DataFolder, ViconFile], ','); ViconDataMat = ViconDataMat(2:end, :); 
tVicon = ViconDataMat(:,1); 

tv = tVicon - (dtva); 
pw = ViconDataMat(:,2:4) * 1000; % convert to mm
qw = ViconDataMat(:,5:8); 
pl1 = zeros(size(pw,1),3); 
for i = 1:size(pw,1)
    pl1(i,:) = (AMat(qflip(qw(i,:)))' * [-66; -26; 0])';
    pw(i,:) = pw(i,:) + pl1(i,:);
end

% Resample the VICON data down to the frame rate of the unity data

% Compute sample rates and frequencies
Ts = (max(t) - min(t)) / length(t); disp(['Sensor sample rate: ', num2str(1/Ts), ' Hz']); 
Tv = (max(tv) - min(tv)) / length(tv); disp(['Vicon sample rate: ', num2str(1/Tv), ' Hz']); 
Fv = 1/Tv; 
Fs = 1/Ts; 

% Downsample the VICONdata
pw = resample(pw,round(Fs),round(Fv),1);
tv = linspace(min(tv),max(tv),size(pw,1)); 
qw = resample(qw,round(Fs),round(Fv),1);


if 0
    % Raw position data
    figure; 
    plot(t, pu)
    legend('x', 'y', 's')
    title('Position in undistorted image')

    % IMU raw data
    figure; 

    subplot(2,2,1)
    plot(t,a)
    title('Accelerometer')
    xlabel('time (s)')
    ylabel('m/s^2')
    legend('X', 'Y', 'Z')

    subplot(2,2,2)
    plot(t,g)
    title('Gyroscope')
    xlabel('time (s)')
    s = sprintf('%c/s', char(176)); xlabel('time (s)'); 
    ylabel(s)
    legend('X', 'Y', 'Z')

    subplot(2,3,5)
    plot(t,m)
    title('Magnetometer')
    xlabel('time (s)')
    ylabel('\mu T')
    legend('X', 'Y', 'Z')

    % Position comparison 
    figure; 

    subplot(1,2,1)
    plot(t, pu)
    xlim([min(t) max(t)])
    title('Application')

    subplot(1,2,2)
    plot(tv, pw)
    xlim([min(t) max(t)])
    title('Vicon')

    % Orientation comparison 
    figure; 
    subplot(1,2,1)
    plot(t, qc)
    subplot(1,2,2)
    plot(tv, qw)
else 
    close all; 
    clf; 
end


% Undistorted image, camera measurements, world measurements

% Undistorted image measurements
zu = [pu(:,1) pu(:,2) pu(:,3)]; 

% Camera measurements
dc = 49 ./ ( zu(:,3) * sqrt( 1/fx^2 + 1/fy^2 ) ); % small blue ball is x mm
xc = (dc / fx) .* ( pu(:,1) - cx ); 
yc = (dc / fy) .* ( cy - pu(:,2) ); 

zc = [xc yc dc]; % (x,y,z), obeying the right hand rule

% Camera extrinsic parameters
wRc = [0 0 -1; -1 0 0; 0 1 0];  
cTw = [-8; -75.4; 501];  

% World observations
zw = (wRc * zc' + repmat(cTw,1,size(zc,1)))';  

% naive error analysis (delta position naive or dpn for short)
j = find(tv > min(t), 1, 'first'); N = length(t); % index of start, number number of samples
tvpr = tv(tv > min(t) & tv < max(t)); 
pwpr = pw(tv > min(t) & tv < max(t), :); 
dpn = zw - pw(j:(j+N-1), :); 
dpnbias = mean(dpn)
Ppn = cov(dpn)
[V,D] = eig(Ppn); 
disp(['Bias Naive: ', num2str(mean(dpn))])
disp(['Variance Naive: ', num2str([var(dpn(:,1)), var(dpn(:,2)), var(dpn(:,3))]) ]); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reference vs estimate vs error in x,y,z
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure; 

subplot(1,3,1)
plot( tvpr, pwpr )
title('Vicon')
legend('x','y','z')
xlim([min(t) max(t)])

subplot(1,3,2)
plot( t, zw )
title('Estimate')
legend('x', 'y', 'z')
xlim([min(t) max(t)])

subplot(1,3,3)
plot( t, dpn )
title('Error')
legend('x', 'y', 'z')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reference vs estimate in the image plane
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure; 
plot(tvpr, pwpr(:,2:3), t, zw(:,2:3))
title('Maneuver in the image plane')
ylabel('Position (mm)')
xlabel('Application time (s)')
h = legend('$y$', '$z$', '$\hat{y}$', '$\hat{z}$');
set(h,'Interpreter','latex')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ellipsoid of errors for image plane motion
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
scatter(dpn(:,2), dpn(:,3))
title('Error for vertical maneuver')
xlabel('Error in y (mm)')
ylabel('Error in z (mm)')
xlim([-100 100])
ylim([-100 100])

hold on

drawArrow = @(x,y, varargin) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0, varargin{:} ); 
x1 = [dpnbias(2) (dpnbias(2) + sqrt(Ppn(2,2)))];
y1 = [dpnbias(3) dpnbias(3)];
drawArrow(x1,y1, 'linewidth',3,'color','r'); 
x2 = [dpnbias(2) dpnbias(2)];
y2 = [dpnbias(3) (dpnbias(3) + sqrt(Ppn(3,3)))];
drawArrow(x2,y2, 'linewidth',3,'color','g')

hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reference vs estimate for depth maneuver
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure; 
plot(tvpr, pwpr(:,1), t, zw(:,1))
title('Maneuver in the depth dimension')
ylabel('Position (mm)')
xlabel('Application time (s)')
h = legend('$x$', '$\hat{x}$');
set(h,'Interpreter','latex')
xlim([min(t) min(t)+20]) % display 20 seconds (before outliers arise)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Error histogram for depth maneuver
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure;
histogram(dpn(:,1), 100)
title(['Distribution of errors for depth estimates ($\sigma_x = ', num2str(sqrt(Ppn(1,1)), '%.0f'), '$mm)'], 'Interpreter', 'Latex')
xlabel('Error (mm)')
ylabel('Number of samples')
xlim([-200 200])

%% Kalman Filter

dt = t(2) - t(1); % application time step

% Transform the acceleration readings into world coordinates
j = find(tv > min(t), 1, 'first'); % index of start
N = length(t); % number of samples
aw = zeros(N,3); 
for i = 1:N
    aw(i,:) = (AMat(qflip(qw((i+j-1),:)))' * a(i,:)' + [0;0;9.81])';
end
aw = aw - repmat(mean(aw,1),N,1); % get rid of calibration errors
aw = 1000*aw; 

% Initalize Filter varianbles
Sr = ((0.01)*[10 1 1]).^2; 
R = diag(Sr); 

naccel = (1000*150e-6); % convert to mm/s^2
Q2 = ((naccel^2) / dt)*[dt^4/4 dt^3/2; dt^3/2 dt^2/2]; Q = blkdiag(Q2, Q2, Q2); 

% State transition and observation matrices
F2 = [1 dt; 0 1]; A = blkdiag(F2, F2, F2); 
G2 = [dt^2/2; dt]; G = blkdiag(G2, G2, G2); 

% Run Kalman Filter

% Initialization
pr = zeros(N,3); 

xest = [zw(1,1) 0 zw(1,2) 0 zw(1,3) 0]'; % start at 0 velocity
H2 = [1 0]; H = blkdiag(H2, H2, H2);

Ppr = [100 100].^2; P = blkdiag(diag(Ppr), diag(Ppr), diag(Ppr));

pr(1, :) = xest([1 3 5])'; % x,y,z components of the position
for i = 2:N
   
    % Prediction 
    Ppred = A * P * A' + Q; 
    xpred = A * xest + G * aw(i-1,:)';  
    
    % Update
%     P = inv( inv(Ppred) + H' * R' * H ); 
%     K = P * H' * R';   
    K = Ppred * H' / (H * Ppred * H' + R);
    P = (eye(6,6) - K * H) * Ppred;
    
    xest = (xpred + K * ([zw(i,1); zw(i,2); zw(i,3)] - H * xpred)); 
    pr(i, :) = xest([1 3 5])'; % x,y,z components of the position
    
end

% naive error (delta position naive or dpn for short)
j = find(tv > min(t), 1, 'first'); % index of start
N = length(t); % number of samples
pws = pw(j:(j+N-1), :); 
dpk = pr - pws; 
biasdpk = mean(dpk)
Ppk = cov(dpk) 
disp(['Bias Kalman: ', num2str(mean(dpk))])
disp(['Variance Kalman: ', num2str([var(dpk(:,1)), var(dpk(:,2)), var(dpk(:,3))]) ]); 
% 
% % Find the affect on latency
% latency = finddelay( pws(:,2) - repmat(mean(pws(:,2)), N,1), pr(:,2) - repmat(mean(pr(:,2)), N,1) ) 
% dpnkpr = pr(latency:end, :) - pws(1:(end-latency+1), :); % corrected error with latency subtracted


figure; 

subplot(2,2,1)
plot( t, pws )
title('Vicon')
legend('x','y','z')
xlim([min(t) max(t)])

subplot(2,2,2)
plot( t, pr )
title('Estimate')
legend('x', 'y', 'z')
xlim([min(t) max(t)])

subplot(2,2,3)
plot( t, dpk )
title('Error')
legend('x', 'y', 'z')
ylim([-100 100])
% 
% subplot(2,2,4)
% plot(t(1:(end-latency+1)), dpnkpr)
% title('Corrected Error')
% ylim([-100 100])

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Error histogram for depth maneuver
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure;
histogram(dpn(:,1), 100)
title(['Distribution of errors for depth estimates ($\sigma_x = ', num2str(sqrt(Ppk(1,1)), '%.0f'), '$mm)'], 'Interpreter', 'Latex')
xlabel('Error (mm)')
ylabel('Number of samples')
xlim([-200 200])

