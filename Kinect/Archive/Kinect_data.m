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
[frame, ts, metaData] = getdata(vid2);
