%% Variables of interest
N = 100; 
depth_vs_time = zeros(1, N);
y_vs_time = zeros(1, N); 
x_vs_time = zeros(1, N); 

% Acquire data into memory before logging it
colorVid = videoinput('kinect',1); 
depthVid = videoinput('kinect',2);

%% Set Kinect Properties
% Set the triggering mode to 'manual'
triggerconfig([colorVid depthVid],'manual');
colorVid.FramesPerTrigger = 1;
depthVid.FramesPerTrigger = 1;

% Start the color and depth device. This begins acquisition, but does not
% start logging of acquired data.
start([colorVid depthVid]);

% Trigger the devices to start logging of data.
trigger([colorVid depthVid]);

% Retrieve the acquired data
[rgb_image,colorTimeData,colorMetaData] = getdata(colorVid,1);
[depth_image,depthTimeData,depthMetaData] = getdata(depthVid,1);

% Select the colored region to track
hsv_image = rgb2hsv(rgb_image); 
shapeInserter = vision.ShapeInserter('BorderColor','Custom','CustomBorderColor',[1 0 0]);
tracker = vision.HistogramBasedTracker;
picfig = figure; 
imshow(rgb_image); 
title('Bounding Box');
objectRegion=round(getPosition(imrect));
initializeObject(tracker, hsv_image(:, :, 1), objectRegion)

% Initalize the depth tracking figure
figure; 
dax = line(nan, nan, 'Color', 'r');
title('Depth vs time') 
xlabel('Time (s)')
ylabel('Depth (pixels)')

t0 = tic; 
t = [0];
for i = 1:N
    % Acquire frame snapshots
    rgb_frame = getsnapshot(colorVid); 
    depth_frame = getsnapshot(depthVid);
	
	% Extend the time array
	if(i > 1)
        toc(t0);
		t = [t toc(t0)];
	end
    
    % Find the bounding box for the selected object
    hsv = rgb2hsv(rgb_frame);                   
    bbox = step(tracker, hsv(:,:,1));  
    x = bbox(1); 
    y = bbox(2); 
    width = bbox(3); 
    height = bbox(4); 
    
    % Draw the bounding box around the region
%     J = step(shapeInserter, rgb_frame, int32(bbox'));
%     figure(picfig)
%     imshow(J)
%     drawnow limitrate;
    
    % Compute the average depth of the pixels
    c = [x x (x+width) (x+width)]; 
    r = [y (y+height) (y+height) y]; 
    x_vs_time(i) = x;
    y_vs_time(i) = y; 
    BW = roipoly(rgb_frame,c,r);
    roi = depth_frame(BW); 
    pixels_of_interest = roi(roi > 100); 
    depth_vs_time(i) = mean(pixels_of_interest); 
    
	% Plot the results
	set(dax, 'Xdata', t, 'Ydata', depth_vs_time(1:i));
    drawnow limitrate;
end

%% 
stop([colorVid depthVid]);

%% 
% Clear up the workspace
delete(colorVid)
delete(depthVid)
clear colorVid
clear depthVid
