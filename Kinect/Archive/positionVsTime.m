function [x, y, d] = positionVsTime(depth_frames, rgb_frames, tracker)

N = size(depth_frames, 4); 
x = zeros(1, N); 
y = zeros(1, N); 
d = zeros(1, N);

if( N ~= size(rgb_frames, 4) )
    error('Color and depth frame sets are different sizes'); 
end

for i = 1:N
    % Pull out the individual rgb frames
    rgb_frame = rgb_frames(:, :, :, i); 
    depth_frame = depth_frames(:, :, :, i); 
    
    % Find the bounding box for the selected object
    hsv = rgb2hsv(rgb_frames(:, :, :, i));                   
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
    x(i) = (x_box + width) / 2;
    y(i) = (y_box + height) / 2; 
    d(i) = mean(pixels_of_interest); 
end