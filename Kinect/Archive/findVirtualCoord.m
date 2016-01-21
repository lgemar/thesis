function v = findVirtualCoord(w)
% Given a real world coordinate, w, this function will prompt the user to 
% place a colored object at the coordinate and will compute the corresponding
% point in the virtual Kinect video space

%% Initialize the Kinect object
K = Kinect(); 
NUM_FRAMES = 100; 

% Define the world coordinates of interest
w0 = [0; 0; 0]; 

%% Place the colored object at 4 points in space
flag1 = 0; 

prompt = ['Place the colored object at (', num2str(w(1)), ',' num2str(w(2)), ',', num2str(w(3)), ').',' Enter "1" if you are finished: '];
while(flag1 ~= 1)
    flag1 = input(prompt);
    if(flag1==1)
        disp('Great! Please draw a box around the object...')
    else
        disp('Please follow the directions...')
    end
end

% Compute the average depth of a particular point
K.startAcq(NUM_FRAMES);
[rgb_frames, depth_frames] = K.getFrames(NUM_FRAMES);
K.stopAcq();
tracker = initializeTracker(rgb_frames(:, :, :, 1));
[x, y, d] = positionVsTime(depth_frames, rgb_frames, tracker); 
row_avg = mean(x); 
column_avg = mean(y);
depth_avg = mean(d); 
centroid = [row_avg, depth_avg, column_avg]
disp('This point has been registered!')

% Set the virtual coordinate
v = centroid;
