%% Close, clear, and set up for acquisition
clear all;
acqSize = 1000; 

%% Open the serial port
com_port = 'COM6'; 
baud_rate = 250000; 
s = serial(com_port, 'BaudRate', baud_rate);
s.InputBufferSize = 2048; 
fopen(s); 

%% Set up the filters and parameters for acquisition
i=1;
dt=0;
GyroRate=zeros(3,acqSize);
Acc=zeros(3,acqSize);
t = zeros(1, acqSize);

%% Do the acquisition

% Accelerometer Data
counter = 0; 
disp('Starting acquisition...')
t0 = tic;
while(i<=acqSize)
    if(i>1)
        t(i)=toc(t0);
    end

    % Get the data
    [D, count, msg] = fread(s, 6, 'int16');

	% Get the data from the sampler
    Acc(1:3,i) = D(1:3)';
    GyroRate(1:3,i) = D(4:6)';
    
    i=i+1; 
end
disp('Finishing acquisition...')
disp(['Sample rate: ', num2str(length(t)/max(t))])
disp('Cleaning up...')
    
%% Clean up the serial port from the workspace
fclose(s); 
delete(s)
clear s
