%% VARIABLE AND OBJECT INITIALIZATION

% =========================================================================
% INITIALIZE CAMERA
% =========================================================================
cam = webcam(1); % Connect to the webcam.
img = snapshot(cam); % Acquire a Frame
image(img); % Display the frame in a figure window.

% =========================================================================
% INITIALIZE INERTIAL MEASUREMENT UNIT (IMU)
% =========================================================================
baudRate = 250000;
comPort = 'COM5'; 
serialPort = serial(comPort, 'BaudRate', baudRate);
formatString=['a/g/m:\t','%d\t','%d\t','%d\t','%d\t','%d\t','%d\n'];
fopen(serialPort); 

data = fscanf(serialPort, formatString); % Acquire a data frame. 
while( size(data) ~= 6 )
    data = fscanf(serialPort, formatString);
end

% =========================================================================
% INITIALIZE CLOCKS
% =========================================================================
t0 = tic; tGlobal = toc(t0); 
tOrientation = tGlobal; tTarget = tGlobal; tTool = tGlobal; 
dtOrientation = 0; dtTarget = 0; dtTool = 0; 

%% DATA ACQUISITION LOOP

while true
    
    % =====================================================================
    % FIND TOOL ORIENTATION
    % =====================================================================
    
    % Update orientation clock
    tGlobal = toc(t0); 
    dtOrientation = tGlobal - tOrientation; 
    tOrientation = tOrientation + dtOrientation; 
    
    % Get IMU Data
    
    % Update orientation model
    
    % =====================================================================
    % FIND TARGET POSITION
    % =====================================================================
    
    % Update target clock
    tGlobal = toc(t0); 
    dtTarget = tGlobal - tTarget; 
    
    if( dtTarget > 0.01 )
        
        tTarget = tTarget + dtTarget; 
        img = snapshot(cam);
        image(img);
        
    end
    
    % =====================================================================
    % FIND TOOL POSITION 
    % =====================================================================  
    
    % Update tool clock
    tGlobal = toc(t0); 
    dtTool = tGlobal - tTool; 
    tTool = tTool + dtTool; 
    
end

%% APPLICATION DEINITIALIZATION

clear cam 