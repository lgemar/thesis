function [ ] = recordKinectVideo( num_frames, acq_name )

% Name the acquisition 
acq = ['C:\Users\lukasgemar\OneDrive\Documents\ES100\Prototype_Tests\', acq_name];
mkdir(acq); 

% Open the Kinect Video object
vid = videoinput('kinect',1,'RGB_640x480');
vid2 = videoinput('kinect', 2, 'Depth_640x480'); 

% Set the properties of the video object
NUM_FRAMES = num_frames; 
vid.FramesPerTrigger = NUM_FRAMES; % Use  help imaqhelp to find these props
vid2.FramesPerTrigger = NUM_FRAMES;

% Acquire data into memory before logging it
start(vid)
% Insert a fixed delay time
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
writetable(T1, [acq, '/rgb_metadata', '.csv']); 

disp('Writing depth metadata to file'); 
writetable(T2(:,[1 2 12 15]), [acq, '/depth_metadata','.csv']); 

% Make sure that the rgb frames are available for later! 
disp('Writing rgb frame data to file'); 
v1 = VideoWriter([acq, '/rgb_video', '.mj2'], 'Motion JPEG 2000'); 
open(v1); 
for i = 1:NUM_FRAMES
    writeVideo(v1, rgb_frames(:, :, :, i)); 
end
close(v1);

% Make sure that the depth frames are available for later! 
disp('Writing depth frame data to file'); 
v2 = VideoWriter([acq, '/depth_video', '.mj2'], 'Motion JPEG 2000'); 
open(v2); 
for i = 1:NUM_FRAMES
    writeVideo(v2, depth_frames(:, :, :, i)); 
end
close(v2);


end

