%% VARIABLE AND OBJECT INITIALIZATION

% =========================================================================
% INITIALIZE AND CALIBRATE CAMERA
% =========================================================================

% % Define images to process
% imageFileNames = {'C:\Users\Lukas Gemar\thesis\Image1.png',...
%     'Image2.png',...
%     'Image3.png',...
%     'Image4.png',...
%     'Image5.png',...
%     'Image6.png',...
%     'C:\Users\Lukas Gemar\thesis\Image7.png',...
%     'C:\Users\Lukas Gemar\thesis\Image8.png',...
%     'C:\Users\Lukas Gemar\thesis\Image9.png',...
%     'C:\Users\Lukas Gemar\thesis\Image10.png',...
%     'C:\Users\Lukas Gemar\thesis\Image11.png',...
%     'C:\Users\Lukas Gemar\thesis\Image12.png',...
%     'C:\Users\Lukas Gemar\thesis\Image13.png',...
%     'C:\Users\Lukas Gemar\thesis\Image14.png',...
%     'C:\Users\Lukas Gemar\thesis\Image15.png',...
%     'C:\Users\Lukas Gemar\thesis\Image16.png',...
%     'C:\Users\Lukas Gemar\thesis\Image17.png',...
%     'C:\Users\Lukas Gemar\thesis\Image18.png',...
%     'C:\Users\Lukas Gemar\thesis\Image19.png',...
%     'C:\Users\Lukas Gemar\thesis\Image20.png',...
%     };
% 
% % Detect checkerboards in images
% [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
% imageFileNames = imageFileNames(imagesUsed);
% 
% % Generate world coordinates of the corners of the squares
% squareSize = 22;  % in units of 'mm'
% worldPoints = generateCheckerboardPoints(boardSize, squareSize);
% 
% % Calibrate the camera
% [cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
%     'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
%     'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'mm', ...
%     'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', []);

cam = webcam(1); % Connect to the webcam.

% =========================================================================
% INITIALIZE INERTIAL MEASUREMENT UNIT (IMU)
% =========================================================================
baudRate = 115200;
comPort = '/dev/cu.usbmodem1451'; 
IMU = serial(comPort, 'BaudRate', baudRate);
fopen(IMU); 

% [atoolprev, gtoolprev] = getIMUData(IMU);
% qtoolprev = eul2quat(flip(atoolprev));
% [atool, gtool] = getIMUData(IMU);
% qtoolprev = orientationModel(atool,gtool,qtoolprev,0); 

vFrame = LCMCoordinateFrame('vicon',JLCMCoder(vicon.ViconLCMCoder()),'v');
vFrame.subscribe('VICON_wand');

% =========================================================================
% INITIALIZE CLOCKS
% =========================================================================
t0 = tic; tGlobal = toc(t0); 
tOrientation = tGlobal; tTarget = tGlobal; tTool = tGlobal; 
dtOrientation = 0; dtTarget = 0; dtTool = 0; 

% =========================================================================
% INITIALIZE ADDITIONAL VARIABLES
% =========================================================================
ptarget = [0 0 0]; 
ptool = [0 0 0]; 

% =========================================================================
% INITIALIZE THE TOOL DISPLAY
% =========================================================================

% fig = figure; 
% 
% % Plot a camera pointing along the y -axis.
% R = [1 0 0; 0 1 0; 0 0 1];
% camPlot = plotCamera('Location',[0 0 0],'Orientation',R,'Opacity',0);
% 
% % Make the space large enough for the animation.
% xlim([-15,1000]);
% ylim([-15,1000]);
% zlim([15,1000]);
% 
% % Set the view properties.
% grid on
% axis equal
% axis manual
% 
% % Plot initial point
% plothandle = plot3(0, 0, 0, '*');

% =========================================================================
% INITIALIZE THE DATA LOGS
% =========================================================================

rawData = zeros(1,17);  
videoData = zeros(720,1280,3);  

%% DATA ACQUISITION LOOP

disp('Acq starting...');

while( tGlobal < 60 )
    
    % =====================================================================
    % FIND TARGET POSITION
    % =====================================================================
    
    % Update target clock
    tGlobal = toc(t0); 
    dtTarget = tGlobal - tTarget; 
    
    if( dtTarget > 0.0 )
        
        % Update target clock
        tTarget = tTarget + dtTarget; 
        
        % Get camera image
        I = snapshot(cam);
        
        % Acquire data
        videoData = cat(4,videoData,I); 
        
        % Update target model
        % ptarget = targetModel(I,cameraParams); 
        
        % Optional: Display image
        % image(I); 
    
    end
    
    % =====================================================================
    % FIND TOOL ORIENTATION
    % =====================================================================
    
    % Update orientation clock
    tGlobal = toc(t0); 
    dtOrientation = tGlobal - tOrientation; 
    tOrientation = tOrientation + dtOrientation; 
    
    % Get IMU Data
    % [atool, gtool] = getIMUData(IMU);
    data = getRawData(IMU);
    y = vFrame.getNextMessage(1000);
    if isempty(y)
        y = zeros(7,1);
    end
    rawData = cat(1, rawData, cat(2,tGlobal,data',y'));    
    
    % Update orientation model
    % qtool = orientationModel(atool,gtool,qtoolprev,0); 
    % qtoolprev = qtool; 
    

    % =====================================================================
    % FIND TOOL POSITION 
    % =====================================================================  
    
    % Update tool clock
    tGlobal = toc(t0); 
    dtTool = tGlobal - tTool; 
    tTool = tTool + dtTool;
    
    % Update tool model
    % thetatool = flip(quat2eul(qtool)); 
    % ptool = ptarget + 36 * thetatool; 
    
    % =====================================================================
    % PLOT TOOL POSITION
    % =====================================================================
           
% %     hold on; 
% %     
% %     Make the space large enough for the animation.
% %     xlim([-15,500]);
% %     ylim([-15,500]);
% %     zlim([15,500]);
% %     
% %     Set the view properties.
% %     grid on
% %     axis equal
% %     axis manual  
% %     
% %     plot3([ptarget(1) ptool(1)], [ptarget(2) ptool(2)], [ptarget(3) ptool(3)])  
% %     drawnow limitrate;
% %     
% %     hold off;  

end

disp('Acq ending...')

%% DATA LOGGING

% Write video
v = VideoWriter('videodata2.avi');
open(v)
for i = 1:size(videoData,4)
    writeVideo(v,videoData(:,:,:,i))
end
close(v); 

% Write accelerometer data
csvwrite('acceldata2.csv', rawData); 


%% APPLICATION DEINITIALIZATION

% DEINITIALIZE CAMERA
clear('cam');

% DEINITIALIZE IMU
fclose(IMU); 
delete(IMU);
clear serialPort;