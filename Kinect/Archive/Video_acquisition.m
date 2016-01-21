%% Detect the Kinect sensor
info = imaqhwinfo('kinect'); 
info.DeviceInfo(1)
info.DeviceInfo(2)

%% Create the videoinput object for the color sensor. 
% DeviceID 1 is used for the color sensor.
vid = videoinput('kinect',1,'RGB_640x480');

% Look at the device-specific properties on the source device, 
% which is the color sensor on the Kinect camera.
src = getselectedsource(vid);

% Preview the color stream by calling preview on the color sensor object. 
preview(vid);
% Pause for a couple seconds while the video stream is up and running
pause(5) 
% When you are done previewing, close the preview window.
closepreview(vid);

%% Create the depth sensor video object and preview it
vid2 = videoinput('kinect', 2, 'Depth_640x480'); 

src = getselectedsource(vid2); 

preview(vid2);
pause(5)
closepreview(vid2); 

%% Start the second videoinput object (the depth stream).
start(vid2);
% Get the data on the object.
[rgb_frames, ts, rgb_metaData] = getdata(vid2);

%% Write the image data to disk 
% Name the acquisition 
acq = 'acq3'; 

% Open the Kinect Video object
vid = videoinput('kinect',1,'RGB_640x480');
vid2 = videoinput('kinect', 2, 'Depth_640x480'); 

% Set the properties of the video object
NUM_FRAMES = 1000; 
vid.FramesPerTrigger = NUM_FRAMES; % Use  help imaqhelp to find these props
vid2.FramesPerTrigger = NUM_FRAMES;

% Acquire data into memory before logging it
start(vid)
start(vid2)

% Acquire the video
disp('Acquire data'); 
while(vid.FramesAcquired ~= NUM_FRAMES || vid2.FramesAcquired ~= NUM_FRAMES)
    pause(.1); 
end
[rgb_frames, ts1, rgb_metaData] = getdata(vid, NUM_FRAMES);
[depth_frames, ts2, depth_metaData] = getdata(vid2, NUM_FRAMES);

% Put metadata into memory
T1 = struct2table(rgb_metaData);
T2 = struct2table(depth_metaData); 

% Clear up the workspace
delete(vid)
delete(vid2)
clear vid
clear vid2

% Make sure that the metadata is available for later! 
disp('Writing rgb metadata to file'); 
writetable(T1, [acq, '_rgb_metadata', '.csv']); 

disp('Writing depth metadata to file'); 
writetable(T2(:,[1 2 12 15]), [acq, '_depth_metadata','.csv']); 

% Make sure that the rgb frames are available for later! 
disp('Writing rgb frame data to file'); 
v1 = VideoWriter([acq, '_rgb', '.mj2'], 'Motion JPEG 2000'); 
open(v1); 
for i = 1:NUM_FRAMES
    writeVideo(v1, rgb_frames(:, :, :, i)); 
end
close(v1);

% Make sure that the depth frames are available for later! 
disp('Writing depth frame data to file'); 
v2 = VideoWriter([acq, '_depth', '.mj2'], 'Motion JPEG 2000'); 
open(v2); 
for i = 1:NUM_FRAMES
    writeVideo(v2, depth_frames(:, :, :, i)); 
end
close(v2);

%% Read the frames back into memory for analysis
acq = 'acq3';
[N, rgb, rgb_meta, d, dmeta] = readKinectData(acq); 

% Read in the metadata to memory
T = readtable([acq, '_depth_metadata', '.csv']); 

%% Play the video that was read back
playVideo(rgb)
playVideo(d)

%% Obsolete pieces of code
% v = VideoReader([acq, '_rgb', '.mj2']);
% video = read(v); 
% Write the size of the video 
% size(video) 

