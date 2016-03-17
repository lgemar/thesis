% Taken from https://www.youtube.com/watch?v=Z2-kDVF37FQ, with

cam = webcam(1); 

%%
colorVid = videoinput('kinect',1); 

set([colorVid depthVid], 'FramesPerTrigger', Inf);
colorVid.FrameGrabInterval=3; 
depthVid.FrameGrabInterval=3; 

start([colorVid depthVid]);

fig1 = figure;
set(fig1, 'Position', [100 100 400 300])
imaxis = imshow(getsnapshot(colorVid)); 

fig2 = figure;
set(fig2, 'Position', [600 100 400 300])
ylim([0 2000])
title('Object Depth')
legend('Depth from Kinect', 'Depth from object size')
depth_act = []; 
depth_est = []; 
width_est = []; 

while( colorVid.FramesAcquired <= 80 )
    data = getsnapshot(colorVid);
    depth = getsnapshot(depthVid); 
       
    adj_im =  0.7*data(:,:,1)+ 0.2*data(:,:,2)+0.1*data(:,:,3); 
    diff_im = imsubtract(adj_im,rgb2gray(data)); 
    diff_im = medfilt2(diff_im,[5,5]); 
    diff_im = im2bw(diff_im,0.18); 
    diff_im = bwareaopen(diff_im,200); 
    
    bw = bwlabel(diff_im,8); 
    stats = regionprops(bw,'BoundingBox','Centroid'); 
   
    figure(fig1)
    cla(imaxis); 
    set(imaxis,'CData',data);
   
    hold on
    
    for object = 1:length(stats)
        bb = stats(object).BoundingBox; 
        bc = stats(object).Centroid; 
        
        r1 = rectangle('Position',bb,'EdgeColor','r','LineWidth',1);
        r2 = plot(bc(1),bc(2),'-m+'); 
    end
    
    hold off
    
    objwidth = 92.3; 
    ff = 531.5; 
    
    % (object distance from camera) = distance of centroid 
    depth_act = [depth_act (depth(round(bc(2)), round(bc(1))))]; 
       
    % (Object distance from camera) = 
    % (object size * focal length) / (object size in image)
    depth_est = [depth_est (objwidth * ff) / bb(3)]; 
    
    width_est = [width_est bb(3)]; 
    
    figure(fig2)
    
    hold on 
    
    plot(depth_act, 'r')
    plot(depth_est, 'b')
    
    hold off
end

%%
depth_act = double(depth_act); 
disp(['Mean of the actual depth: ', num2str(mean(depth_act))]) 
disp(['Mean of the estimated depth: ', num2str(mean(depth_est))]) 
disp(['Mean of the width estimate: ', num2str(mean(width_est))]) 

disp(['Std of the actual depth: ', num2str(std(depth_act))]) 
disp(['Std of the estimated depth: ', num2str(std(depth_est))]) 
disp(['Std of the width estimate: ', num2str(std(width_est))]) 
disp(['Mean absolute error of estimated depth: ', num2str(mean(abs(depth_est - depth_act)))]) 

%%
stop([colorVid depthVid]);

flushdata([colorVid depthVid]); 

clear all; 