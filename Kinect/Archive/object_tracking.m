%% Load the video objects
K = Kinect(); 

%% Record video with the Kinect
disp('Setting acquisition properties...')
NUM_FRAMES = 500; 
disp('Starting the Kinect...')
K.startAcq(NUM_FRAMES);

%% Acquire the video into memory
disp('Acquiring the data into memory...'); 
[rgb_frames, depth_frames] = K.getFrames(NUM_FRAMES);

K.stopAcq();

%% Set up the object tracker
disp('Setting up the object tracker...')
tracker = initializeTracker(rgb_frames(:, :, :, 1));

%% Compute the position vs time
disp('Computing the position vs time...'); 
[row_data, column_data, d] = positionVsTime(depth_frames, rgb_frames, tracker); 

%% Compute the coordinate transformation and do the conversion
disp('Changing the coordinates...')
T = computeTransform('test', 0); 
V = [row_data; d; column_data; ones(1, NUM_FRAMES)]; 
W = T * V; 

%% Plot the data
disp('Plotting the data...'); 
figure(1) 

subplot(1, 3, 1)
plot(W(1,:)); title('X vs time'); xlabel('Sample number'); ylabel('x (m)'); 

subplot(1, 3, 2)
plot(W(2,:)); title('Depth vs time'); xlabel('Sample number'); ylabel('y (m)'); 

subplot(1, 3, 3)
plot(W(3,:)); title('Z vs time'); xlabel('Sample number'); ylabel('z (m)'); 
