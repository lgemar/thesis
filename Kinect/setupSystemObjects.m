 function obj = setupSystemObjects()
% Initialize Video I/O
% Create objects for reading a video from a file, drawing the tracked
% objects in each frame, and playing the video.
% Create a video file reader.
% obj.reader = vision.VideoFileReader('atrium.avi');

% Create two video players, one to display the video,
% and one to display the foreground mask.
obj.videoPlayer = vision.VideoPlayer('Position', [20, 400, 640, 480]);
obj.maskPlayer = vision.VideoPlayer('Position', [740, 400, 640, 480]);
% Create System objects for foreground detection and blob analysis
% The foreground detector is used to segment moving objects from
% the background. It outputs a binary mask, where the pixel value
% of 1 corresponds to the foreground and the value of 0 corresponds
% to the background.
obj.detector = vision.ForegroundDetector('NumGaussians', 3, ...
'NumTrainingFrames', 40, 'MinimumBackgroundRatio', 0.7);

% Connected groups of foreground pixels are likely to correspond to moving
% objects. The blob analysis System object is used to find such groups
% (called 'blobs' or 'connected components'), and compute their
% characteristics, such as area, centroid, and the bounding box.
obj.blobAnalyser = vision.BlobAnalysis('BoundingBoxOutputPort', true, ...
'AreaOutputPort', true, 'CentroidOutputPort', true, ...
'MinimumBlobArea', 10);
end