%% Read in video data
trial_dir = '..\Prototype_Tests\'; 
test_name = 'test_MS'; 

%% Process Video - Select a region of interest
[rgb_video, rgb_ts, depth_video, depth_ts] = readKinectVideo([trial_dir, test_name]);
depth_ts = depth_ts - min(depth_ts(:)); 
rgb_image = rgb_video(:, :, :, 1); 
hsv_image = rgb2hsv(rgb_image); 

%% Track the depth of the region of interest
N = size(depth_video, 4); 
depth_vs_time = zeros(1, N);
y_vs_time = zeros(1, N); 
x_vs_time = zeros(1, N); 

shapeInserter = vision.ShapeInserter('BorderColor','Custom','CustomBorderColor',[1 0 0]);
tracker = vision.HistogramBasedTracker;
figure; imshow(rgb_image); objectRegion=round(getPosition(imrect));  
initializeObject(tracker, hsv_image(:, :, 1), objectRegion)

for i = 1:N
    depth_frame = depth_video(:, :, :, i);
    color_frame = rgb_video(:, :, :, i); 
    
    % Find the bounding box for the selected object
    hsv = rgb2hsv(color_frame);                   
    bbox = step(tracker, hsv(:,:,1));  
    x = bbox(1); 
    y = bbox(2); 
    width = bbox(3); 
    height = bbox(4); 
    
    % Draw the bounding box around the region
    J = step(shapeInserter, color_frame, int32(bbox'));
    % imshow(J)
    imshow(depth_frame)
    
    % Compute the average depth of the pixels
    c = [x x (x+width) (x+width)]; 
    r = [y (y+height) (y+height) y]; 
    x_vs_time(i) = x;
    y_vs_time(i) = y; 
    BW = roipoly(color_frame,c,r);
    idx = find(BW); 
    depth_vs_time(i) = mean(depth_frame(idx)); 
end

%% Draw the graph of depth vs. time
plot(depth_ts, depth_vs_time)
title('Depth vs time') 
xlabel('Time (s)')
ylabel('Depth (pixels)')

%% Pull data from the accelerometer and integrate to find position

% Read the accelerometer data
M = xlsread([trial_dir, test_name, '\accelerometer.csv']); 

% Calculate the measurement parameters
n = size(M, 1); % number of data points

% Read the time axis
tstamp = M(:, 21) - min(M(:, 21)); 

% Read accelerometer data
a = M(:, 22:24); 
ax = a(:, 1); 
ay = a(:, 2); 
az = a(:, 3); 

%% Plot the raw accelerometer data
ax1 = ax; 
figure(1)
% Plot the raw accelerometer data from phone
plot(tstamp, ax1)
hold on
% Compute the velocity over time
v = cumtrapz(tstamp, ax1); 
plot(tstamp, v)
% Compute the position over time
p = cumtrapz(tstamp, v); 
plot(tstamp, p)

title('Raw Accelerometer Data')
ylim([-0.5 1])
legend('Acceleration', 'Velocity', 'Position')
xlabel('Time (s)')
ylabel('Distance (m)')

%% Subtract the mean of the accelerometer data
% Filter the accelerometer data by subtracting the mean
ax2 = ax - mean(ax(:)); 

figure(2)
% Plot the raw accelerometer data from phone
plot(tstamp, ax2)
hold on
% Compute the velocity over time
v = cumtrapz(tstamp, ax2); 
plot(tstamp, v)
% Compute the position over time
p = cumtrapz(tstamp, v); 
plot(tstamp, p)

%% Take a high pass filter of the accelerometer data
% Use a low-pass filter to smooth the accelerometer data
ax3 = sgolayfilt(ax,3,31) - mean(ax(:));

figure(2)
% Plot the raw accelerometer data from phone
plot(tstamp, ax3)
hold on
% Compute the velocity over time
v = cumtrapz(tstamp, ax3); 
plot(tstamp, v)
% Compute the position over time
p = cumtrapz(tstamp, v); 
plot(tstamp, p)

title('Filtered Accelerometer Data')
ylim([-0.2 0.2])
legend('Acceleration', 'Velocity', 'Position')
xlabel('Time (s)')
ylabel('Distance (m)')

%% Compute a time series alignment betweeen the video data and the accelerometer data
Fs = 30; % sampling rate for camera

clf
figure
hold on
temp1 = abs(cumtrapz(resample(az - mean(az(:)), Fs, 100))); 
temp2 = abs(diff(y_vs_time)); 
plot(temp1/max(temp1(:)))
plot(temp2/max(temp2(:)))
title('Aligning Data Streams')
xlabel('Sample Number')
ylabel('Relative Magnitude')
legend('Integrated Acceleration', 'Differentiated Depth Position')
profile1 = temp1; 
profile2 = temp2; 

[acor, lag] = xcorr(profile2, profile1); 
[~,I] = max(abs(acor));
lagDiff = lag(I); 
timeDiff = lagDiff/Fs; 

figure
plot(lag, acor)
title(['Autocorrelation of velocity estimates (lag = ', num2str(lagDiff) ' frames)'])
xlabel('Sample Number')
ylabel('Magnitude')

%% Plot accelerometer data vs kinect data given the time difference
figure
hold on

plot(tstamp, v)
plot(depth_ts - timeDiff, (depth_vs_time - mean(depth_vs_time(:)))/(3* max(depth_vs_time(:))))
ylim([-.15 .15])
xlabel('Time (s)')
ylabel('Normalized Magnitude')
title('Position and Velocity Estimates')
legend('Velocity', 'Position')

