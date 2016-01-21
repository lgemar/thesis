function [bbox, x, y, d] = findPosition(rgb_frame, depth_frame, tracker)

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