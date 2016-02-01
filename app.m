%% VARIABLE AND OBJECT INITIALIZATION

% =========================================================================
% INITIALIZE AND CALIBRATE CAMERA
% =========================================================================
cam = webcam(1); % Connect to the webcam.
I = snapshot(cam); % Acquire a Frame

% Define images to process
imageFileNames = {'C:\Users\Lukas Gemar\thesis\Image1.png',...
    'C:\Users\Lukas Gemar\thesis\Image2.png',...
    'C:\Users\Lukas Gemar\thesis\Image3.png',...
    'C:\Users\Lukas Gemar\thesis\Image4.png',...
    'C:\Users\Lukas Gemar\thesis\Image5.png',...
    'C:\Users\Lukas Gemar\thesis\Image6.png',...
    'C:\Users\Lukas Gemar\thesis\Image7.png',...
    'C:\Users\Lukas Gemar\thesis\Image8.png',...
    'C:\Users\Lukas Gemar\thesis\Image9.png',...
    'C:\Users\Lukas Gemar\thesis\Image10.png',...
    'C:\Users\Lukas Gemar\thesis\Image11.png',...
    'C:\Users\Lukas Gemar\thesis\Image12.png',...
    'C:\Users\Lukas Gemar\thesis\Image13.png',...
    'C:\Users\Lukas Gemar\thesis\Image14.png',...
    'C:\Users\Lukas Gemar\thesis\Image15.png',...
    'C:\Users\Lukas Gemar\thesis\Image16.png',...
    'C:\Users\Lukas Gemar\thesis\Image17.png',...
    'C:\Users\Lukas Gemar\thesis\Image18.png',...
    'C:\Users\Lukas Gemar\thesis\Image19.png',...
    'C:\Users\Lukas Gemar\thesis\Image20.png',...
    };

% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Generate world coordinates of the corners of the squares
squareSize = 22;  % in units of 'mm'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'mm', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', []);

% =========================================================================
% INITIALIZE INERTIAL MEASUREMENT UNIT (IMU)
% =========================================================================
baudRate = 250000;
comPort = 'COM5'; 
IMU = serial(comPort, 'BaudRate', baudRate);
fopen(IMU); 

[atoolprev, gtoolprev] = getIMUData(IMU);
qtoolprev = eul2quat(flip(atoolprev));
[atool, gtool] = getIMUData(IMU);
qtoolprev = orientationModel(atool,gtool,qtoolprev,0); 

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

%% DATA ACQUISITION LOOP

while (tGlobal < 5)
    
    % =====================================================================
    % FIND TOOL ORIENTATION
    % =====================================================================
    
    % Update orientation clock
    tGlobal = toc(t0); 
    dtOrientation = tGlobal - tOrientation; 
    tOrientation = tOrientation + dtOrientation; 
    
    % Get IMU Data
    [atool, gtool] = getIMUData(IMU);
    
    % Update orientation model
    qtool = orientationModel(atool,gtool,qtoolprev,0); 
    qtoolprev = qtool; 
    
    % =====================================================================
    % FIND TARGET POSITION
    % =====================================================================
    
    % Update target clock
    tGlobal = toc(t0); 
    dtTarget = tGlobal - tTarget; 
    
    if( dtTarget > 0.01 )
        
        % Update target clock
        tTarget = tTarget + dtTarget; 
        
        % Get camera image
        I = snapshot(cam);
        
        % Update target model
        ptarget = targetModel(I,cameraParams); 
        
        % Optional: Display image
        image(I); 
        
    end
    
    % =====================================================================
    % FIND TOOL POSITION 
    % =====================================================================  
    
    % Update tool clock
    tGlobal = toc(t0); 
    dtTool = tGlobal - tTool; 
    tTool = tTool + dtTool;
    
    % Update tool model
    ptool = ptarget; 
    
    % =====================================================================
    % DISPLAY TOOL POSITION
    % =====================================================================
    R = [1 0 0; 0 1 0; 0 0 1]; % Plot a camera pointing along the y -axis.
    cam = plotCamera('Location',[0 0 0],'Orientation',R,'Opacity',0);
    % Make the space large enough for the animation.
    xlim([-15,20]);
    ylim([-15,20]);
    zlim([15,25]);
    
    % Set the view properties.
    grid on
    axis equal
    axis manual
    
    plot3(ptool(1),ptool(2),ptool(3),'*')
    
end

%% APPLICATION DEINITIALIZATION

% DEINITIALIZE CAMERA
clear('cam');

% DEINITIALIZE IMU
fclose(IMU); 
delete(IMU);
clear serialPort;