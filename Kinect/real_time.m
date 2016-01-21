%% Variables of interest

% Sample rates, frequencies, and periods
N = 10; % Number of seconds for the acquisition
Tfk = 2; % Number of Kinect samples per second
Tfa = 300; % Number of accelerometer samples per second
Tsk = 1/Tfk; % Period for Kinect samples
Tsa = 1/Tfa; % Period for accelerometer samples

% Estimate the number of data points taken in the acquisition
NKnct = N * Tfk + 1; 
NAcc = N * Tfa + 1; 

% Allocate memory for all the acquisitions
KnctData = zeros(3, NKnct);
AccData = zeros(3, NAcc);
GyroData = zeros(3, NAcc); 

% Allocate memory for the time stamps
tKnct = zeros(1, NKnct);
tAcc = zeros(1, NAcc); 

% Counters
j = 2; % Accelerometer sample counter
k = 2; % Kinect frame counter

%% Setup the Kinect acquisition variables
% Acquire data into memory before logging it
colorVid = videoinput('kinect',1); 
depthVid = videoinput('kinect',2);

% Set Kinect Properties
set([colorVid depthVid], 'FramesPerTrigger', 1);
set([colorVid depthVid], 'TriggerRepeat', Inf);
triggerconfig([colorVid depthVid], 'manual');

%% Set up accelerometer variables
addpath('..\Accelerometer')
com_port = 'COM6'; 
acqSize = 10000; 
A = Accelerometer(com_port); 
A = A.calibrate();

%% Start the color and depth device. This begins acquisition, but does not
% start logging of acquired data.
start([colorVid depthVid]);

% Trigger the devices to start logging of data.
trigger([colorVid depthVid]);

% Retrieve the acquired data
rgb_image = getdata(colorVid);

% Select the colored region to track
disp('Setting up the object tracker...')
tracker = initializeTracker(rgb_image);

%%
disp('Starting data acquisition...')
t0 = tic; t = 0;
while(t < N)
% Extend the time array
    t = toc(t0); % Get the time at the beginning of the loop
    if( (t - tAcc(j-1)) > Tsa )
        % Set the time stamp for the sample
        tAcc(j) = t; 
        % Get the data
        D = A.getDataSample(); 
        % Store the data for later
        AccData(1:3,j) = D(1:3)';
        GyroData(1:3,j) = D(4:6)';
        % Increment the sample counter
        j = j+1; 
    end
    if( (t - tKnct(k-1)) > Tsk )
        % Set the time stamp for the sample
        tKnct(k) = t; 
        % Get the data
        trigger([colorVid depthVid]);
        rgb_frame = getdata(colorVid); depth_frame = getdata(depthVid);
        [x, y, d] = findPosition(rgb_frame, depth_frame, tracker); 
        % Store the data for later
        KnctData(:,k) = [x;y;d]; 
        % Increment the sample counter
        k = k+1; 
        toc(t0)
    end
    % Here's where you'll try to recompute the actual position
end

%% Reduce the sample vectors to their true size
tKnct = tKnct(:,2:(k-1));
KnctData = KnctData(:,2:(k-1));
tAcc = tAcc(:,2:(j-1));
AccData = AccData(:,2:(j-1));
GyroData = GyroData(:,2:(j-1));
%%
disp(['Sampling frequency (Kinect): ' num2str(length(tKnct)/(max(tKnct)-min(tKnct)))])
disp(['Sampling frequency (Acc): ' num2str(length(tAcc)/(max(tAcc)-min(tKnct)))])

%% Visualize the raw Kinect data
% Plot the x-z dimensions of the Kinect data
c = linspace(1,10,size(KnctData,2));
figure;
subplot(1,2,1)
scatter(KnctData(1,:),640-KnctData(2,:),[],c)
xlim([0 640])
ylim([0 480])
% Plot the y-z dimensions of the Kinect data
subplot(1,2,2)
scatter(KnctData(3,:),640-KnctData(2,:),[],c)
xlim([700 2000])
ylim([0 480])

%% Visualize the raw accelerometer data
AccVel = integrate3D(tAcc,AccData); 
AccPos = integrate3D(tAcc,AccVel);

figure; 
subplot(1,3,1)
plot(tAcc,AccData(1,:),'r',tAcc,AccVel(1,:),'g',tAcc,AccPos(1,:),'b')
title('Raw Accelerometer Data (X)')
xlabel('time (s)')
ylabel('distance (m)')
subplot(1,3,2)
plot(tAcc,AccData(2,:),'r',tAcc,AccVel(2,:),'g',tAcc,AccPos(2,:),'b')
title('Raw Accelerometer Data (Y)')
xlabel('time (s)')
ylabel('distance (m)')
subplot(1,3,3)
plot(tAcc,AccData(3,:),'r',tAcc,AccVel(3,:),'g',tAcc,AccPos(3,:),'b')
title('Raw Accelerometer Data (Z)')
xlabel('time (s)')
ylabel('distance (m)')

figure;
c = linspace(1,10,size(AccData,2));
scatter(AccPos(1,:),AccPos(3,:),[], c)

%% Transform the kinect data
% Define the path of movement
% Trigger the devices to start logging of data.
trigger([colorVid depthVid]);
% Retrieve the acquired data
rgb_image = getdata(colorVid);
imshow(rgb_image)
[x,y] = ginput(10); 

%% Hard code the transformation values
% Compute the reference trajectory from the clicked points
xtransform = 0.13 / (368 - 301); % m / pixels
ztransform = 0.193 / (331 - 233); % m / pixels
ytransform =  1 / 1000; % m / values

%%
xorigin = x(1); 
zorigin = (640-y(1)); 

trajectory = [x-xorigin,640-y-zorigin] .* repmat([xtransform;ztransform]',length(x),1);  
KnctDataTransf = [KnctData(1,:)-xorigin;640-KnctData(2,:)-zorigin;KnctData(3,:)] .* repmat([xtransform;ztransform;ytransform],1,size(KnctData,2));

%% Plot the x-z dimensions of the Kinect data
c = linspace(1,10,size(KnctDataTransf,2));
figure;
hold on
scatter(100*KnctDataTransf(1,:),100*KnctDataTransf(2,:),[],c)
plot(100*(x-xorigin) * xtransform,100*(640-y-zorigin)*ztransform, 'r') % Plot the intended trajectory
title('X-Z Motion')
xlabel('x position (cm)')
ylabel('z position (cm)')

%% Compute the error of the Kinect values
Kncterr = zeros(1,size(KnctDataTransf,2)); 
for i = 1:size(KnctDataTransf,2)
    P = KnctDataTransf(1:2,1)'; 
    Kncterr(i) = spatialError(trajectory,P); 
end
mae = mean(Kncterr);

%% Transform the accelerometer data
window_size = 1;
acc_filter = ones(window_size,1) * 1/window_size; 
AccFiltData = [];
for i = 1:length(tAcc)
    if(i < window_size)
        AccFiltData = [AccFiltData, AccData(:,1:i) * acc_filter(1:i)];
    else
        AccFiltData = [AccFiltData, AccData(:,(i - window_size + 1):i) * acc_filter];
    end
end

AccFiltVel = integrate3D(tAcc,AccFiltData);
AccFiltPos = integrate3D(tAcc,AccFiltVel);

figure; 
subplot(1,3,1)
plot(tAcc,AccFiltData(1,:),'r',tAcc,AccFiltVel(1,:),'g',tAcc,AccFiltPos(1,:),'b')
title('Filtered Accelerometer Data (X)')
xlabel('time (s)')
ylabel('distance (m)')
subplot(1,3,2)
plot(tAcc,AccFiltData(2,:),'r',tAcc,AccFiltVel(2,:),'g',tAcc,AccFiltPos(2,:),'b')
title('Filtered Accelerometer Data (Y)')
xlabel('time (s)')
ylabel('distance (m)')
subplot(1,3,3)
plot(tAcc,AccFiltData(3,:),'r',tAcc,AccFiltVel(3,:),'g',tAcc,AccFiltPos(3,:),'b')
title('Filtered Accelerometer Data (Z)')
xlabel('time (s)')
ylabel('distance (m)')

%% Compute the absolute error of the accelerometers values vs. time
Accerr = zeros(1,size(AccFiltPos,2)); 
for i = 1:size(AccFiltPos,2)
    P = AccFiltPos(1:2,i)'; 
    Accerr(i) = spatialError(trajectory,P); 
end
figure;
plot(tAcc,100 * Accerr)
title('Absolute spatial error vs time')
xlabel('time (s)')
ylabel('Absolute error (cm)') 

%% Compute the abosolute error of the combined data streams
% Create a new data stream with drift correction
KnctVelx = diff(KnctDataTransf(1,:)) ./ tKnct(2:end); 
KnctVely = diff(KnctDataTransf(2,:)) ./ tKnct(2:end); 
KnctVelz = diff(KnctDataTransf(3,:)) ./ tKnct(2:end); 
KnctVel = [KnctVelx;KnctVely;KnctVelz];

%% Create an observer system
% Simulate the system
dt = Tsa; 
A = [eye(3),dt*eye(3); zeros(3), eye(3)]; 
B = [zeros(3); dt*eye(3)]; 
C = [eye(3), zeros(3)];
D = 0; 
sys = ss(A,B,C,D,dt); 
% y = lsim(sys,AccData',[],zeros(6,1));

% Compute the predicted values using ss model
state_estimate = zeros(6,1); 
PredictData = zeros(size(AccData)); 
j = 1; 
for k = 1:size(AccData,2)
    if(tKnct(j) < tAcc(k))
        if( j == 1)
            state_estimate(1:3) = [KnctDataTransf(1,j); KnctDataTransf(3,j); KnctDataTransf(2,j)];
        else
            state_estimate(1:3) = [KnctDataTransf(1,j); KnctDataTransf(3,j); KnctDataTransf(2,j)];
            velocity_estimate = (KnctDataTransf(:,j) - KnctDataTransf(:,j-1))/(tKnct(j) - tKnct(j-1));
            state_estimate(4:6) = [velocity_estimate(1); velocity_estimate(3); velocity_estimate(2)];
        end
        j = min(size(KnctDataTransf,2),j + 1); 
    end
    state_estimate = A * state_estimate + B * AccData(:,k); 
    PredictData(:,k) = C * state_estimate; 
end

figure;
c = linspace(1,10,size(AccData,2)); 
scatter(100*PredictData(1,:),100*PredictData(3,:),[],c)
title('Sensor fusion for X-Z motion (Attempt 1)')
xlabel('x position (cm)')
ylabel('z position (cm)')

%% Deinialize
disp('Cleaning up...')
stop([colorVid depthVid]);

A.close(); 
A.delete(); 
clear A

delete(colorVid)
delete(depthVid)
clear colorVid
clear depthVid
