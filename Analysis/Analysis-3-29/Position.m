AllTestNames = {'test1', 'clckalign1', 'horz1', 'horz2', 'horz3', 'depth1', 'depth2', 'depth3'}; 
ViconTestNames = {'test1', 'clkalign1', 'alltests'}; 
ViconTestName = ViconTestNames{3}; 
UnityTestName = AllTestNames{4}; 

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
tv = resample(tv,round(Fs),round(Fv)); 
pw = resample(pw,round(Fs),round(Fv));
qw = resample(qw,round(Fs),round(Fv));


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


%% Undistorted image, camera measurements, world measurements

% Undistorted image measurements
zu = [pu(:,1) pu(:,2) pu(:,3)]; 

% Camera measurements
dc = 45 ./ ( zu(:,3) * sqrt( 1/fx^2 + 1/fy^2 ) ); % small blue ball is x mm
xc = (dc / fx) .* ( pu(:,1) - cx ); 
yc = (dc / fy) .* ( cy - pu(:,2) ); 

zc = [xc yc dc]; % (x,y,z), obeying the right hand rule

% Camera extrinsic parameters
wRc = [0 0 -1; -1 0 0; 0 1 0];  
cTw = [-8; -75.4; 501];  

% World observations
zw = (wRc * zc' + repmat(cTw,1,size(zc,1)))';  

% naive error (delta position naive or dpn for short)
j = find(tv > min(t), 1, 'first'); % index of start
N = length(t); % number of samples
dpn = zw - pw(j:(j+N-1), :); 

figure; 

subplot(1,3,1)
plot( tv, pw )
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