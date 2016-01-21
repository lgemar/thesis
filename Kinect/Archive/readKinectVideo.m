function [ RGB, RGBts, DEPTH, DEPTHts ] = readKinectVideo( filedir )
%readKinectData Given an acquisition ID, read back the vid and metadata
%  OUTPUTS: 
%  N = number of frames
%  RBG = rbg video data
%  D = depth video data
%  RGBMeta = RGB metadata
%  DMeta = Depth metadata

% Read the rgb video data
addpath(filedir); 
v1 = VideoReader(['rgb_video', '.mj2']);
v2 = VideoReader(['depth_video', '.mj2']);
rgb_video = read(v1); % Condense variables 
depth_video = read(v2); 

% Read in the metadata to memory
rgb_table = readtable(['rgb_metadata', '.csv']); 
depth_table = readtable(['depth_metadata', '.csv']); 

% Grab the sec time stamps - command for this, Google it, datenum
rgb_ts = table2array(rgb_table(:, 5)) * 60 + table2array(rgb_table(:, 6)); 
depth_ts = table2array(depth_table(:, 5)) * 60 + table2array(depth_table(:, 6)); 

% Find the starting time
start_time = max(rgb_ts(1, 1), depth_ts(1, 1)); 
end_time = min(rgb_ts(end, 1), depth_ts(end, 1)); 

% Assume that the rgb video started first
rgbIDX = [find(rgb_ts <= start_time, 1, 'last'), find(rgb_ts >= end_time, 1, 'first')]; 
depthIDX = [find(depth_ts <= start_time, 1, 'last'), find(depth_ts >= end_time, 1,   'first')];

% Index into the data sets and return
RGB_temp = rgb_video(:, :, :, rgbIDX(1):rgbIDX(2)); 
RGBts_temp = rgb_ts(rgbIDX(1):rgbIDX(2)); 
DEPTH = depth_video(:, :, :, depthIDX(1):depthIDX(2)); 
DEPTHts = depth_ts(depthIDX(1):depthIDX(2)); 

% Adjust for the fact that RGB video and depth video have differing frame rates
% Resample the rgb video to match the length of the depth video
if(size(RGB_temp, 4) > size(DEPTH, 4))
    RGB = uint8(zeros(size(DEPTH, 1), size(DEPTH, 2), 3, size(DEPTH, 4))); 
    RGBts = zeros(size(DEPTHts)); 
    for i = 1:size(DEPTH, 4)
        ts = DEPTHts(i); 
        idx = find(RGBts_temp <= ts, 1, 'last');
        disp(num2str(idx))
        RGB(:, :, :, i) = RGB_temp(:, :, :, idx); 
        RGBts(i) = RGBts_temp(idx); 
    end
else
    RGB = RGB_temp; 
    RGBts = RGBts_temp; 
end
end

