function [centroids, bboxes, mask] = detectObjects(obj,dframe)

    % Detect the foreground by using the Kinect depth data
    minDepth = 1950; % mm
    maxDepth = 2300; % mm
    depthMask = (dframe > minDepth & dframe < maxDepth);
    
    % Detect foreground. % mask = obj.detector.step(frame);
    mask = depthMask;
    
    % Apply morphological operations to remove noise and fill in holes.
    mask = imopen(mask, strel('rectangle', [3,3]));
    mask = imclose(mask, strel('rectangle', [5, 5]));
    mask = imfill(mask, 'holes');
    
    % Perform blob analysis to find connected components.
    [~, centroids, bboxes] = obj.blobAnalyser.step(mask);
end