% Number of frames
%% Kinect setup
% Kinect objects
colorVid = videoinput('kinect',1); 
depthVid = videoinput('kinect',2);

% Kinect properties
set([colorVid depthVid], 'FramesPerTrigger', 1);
set([colorVid depthVid], 'TriggerRepeat', Inf);
triggerconfig([colorVid depthVid], 'manual');

%% Start acquisition
% Start the Kinect and trigger the first frame
start([colorVid depthVid]);
trigger([colorVid depthVid]);
rgb_frame = getdata(colorVid); % Get first frame data

%% Select the colored region to track

% Set up depth display
figure(2) 
haxis2 = bar([ 10 10]); 
ylim([0 2000])

disp('Setting up the object tracker...')
hsv_image = rgb2hsv(rgb_frame); 
shapeInserter = vision.ShapeInserter('BorderColor','Custom','CustomBorderColor',[1 0 0]);
tracker = vision.HistogramBasedTracker;
figure(1)
haxis1 = imshow(rgb_frame); 
objectRegion=round(getPosition(imrect));
initializeObject(tracker, hsv_image(:, :, 1), objectRegion)

% Initialize the parameters for distance tracking

%% Data acquisition and tracking
disp('Starting data acquisition...')
i = 1; t0 = tic; t = 0;
while( 1 )
    % Trigger the color and depth video cameras
    trigger([colorVid depthVid]);
    
    % Get data off the Kinect
    rgb_frame = getdata(colorVid); 
    depth_frame = getdata(depthVid);
    
    % Find the position of the colored object
    % Find the bounding box for the selected object
    hsv = rgb2hsv(rgb_frame);                   
    bbox = step(tracker, hsv(:,:,1));  
    x_box = bbox(1); 
    y_box = bbox(2); 
    width = bbox(3); 
    height = bbox(4); 

    % Compute the average depth of the pixels
    c = [x_box x_box (x_box+width) (x_box+width)]; 
    r = [y_box (y_box+height) (y_box+height) y_box]; 
    BW = roipoly(rgb_frame,c,r);
    roi = depth_frame(BW); 
    pixels_of_interest = roi(roi > 100); 

    % Compute the coordinates of the object in pixels
    x = (x_box + (x_box + width)) / 2;
    y = (y_box + (y_box + height)) / 2; 
    d = mean(pixels_of_interest);  

    % Display the tracking results
    if(~ishandle(haxis1) || ~ishandle(haxis2))
        break; 
    end
    cla(haxis1); 
    set(haxis1, 'Cdata', rgb_frame); 
    r1 = rectangle('Position',bbox,'EdgeColor','r'); 
    
    set(haxis2, 'YData', [d, (bbox(3) + bbox(4))/2]); 
    
    % Increment the frame counter
    i = i + 1; 
end

%% Deinitialize
disp('Cleaning up...')
stop([colorVid depthVid]);

delete(colorVid)
delete(depthVid)

clear colorVid
clear depthVid