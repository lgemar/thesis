com_port = 'COM5'; 
acqSize=500;

%% Open the serial port
disp('Open the serial port...')
baud_rate = 38400; 
s = serial(com_port, 'BaudRate', baud_rate);
fopen(s); 
formatspec  = ['a/g/m:\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\n']; 

%% Set up the filters and parameters for acquisition
i=1;
dt=0;
GyroRate=zeros(3,acqSize);
Acc=zeros(3,acqSize);
Magn=zeros(3,acqSize);
t = zeros(1, acqSize); kjkj

%% Do the acquisition
h1 = figure(1); 

% Accelerometer Data
subplot(3, 1, 1); sax = line(nan, nan, 'Color', 'r'); say = line(nan, nan, 'Color', 'g'); saz = line(nan, nan, 'Color', 'b');
legend('X-axis', 'Y-axis', 'Z-axis')
title('Accelerometer'); xlabel('s'); ylabel('m/s^2'); 

% Velocity Data
subplot(3, 1, 2); svx = line(nan, nan, 'Color', 'r'); svy = line(nan, nan, 'Color', 'g'); svz = line(nan, nan, 'Color', 'b');
legend('X-axis', 'Y-axis', 'Z-axis')
title('Velocity'); xlabel('s'); ylabel('m/s'); 

% Position Data
subplot(3, 1, 3); spx = line(nan, nan, 'Color', 'r'); spy = line(nan, nan, 'Color', 'g'); spz = line(nan, nan, 'Color', 'b');
legend('X-axis', 'Y-axis', 'Z-axis')
title('Position'); xlabel('s'); ylabel('m'); 

disp('Starting acquisition...')
t0 = tic;
while(i<=acqSize)
    if(i>1)
        t(i)=toc(t0);
    end

    % Get the data
    D = getDataSample(s);

	% Get the data from the sampler
    Acc(1:3,i) = D(1:3)';
    GyroRate(1:3,i) = D(4:6)';
    Magn(1:3,i) = D(7:9)';
    
    i=i+1;
end

disp('Ending acquisition...')
disp('Cleaning up...')   
% Clean up the serial port from the workspace
disp('Closing...')
fclose(s); 
disp('Deleting...')
delete(s)
disp('Clearing...')
clear s

%% Process the data
disp('Processing data...')
% Set the accelerometer data
set(sax, 'XData', t, 'YData', Acc(1,1:end));
set(say, 'XData', t, 'YData', Acc(2,1:end));
set(saz, 'XData', t, 'YData', Acc(3,1:end));

% Set the velocity data
vx = cumtrapz(t, Acc(1,1:end));
vy = cumtrapz(t, Acc(2,1:end));
vz = cumtrapz(t, Acc(3,1:end) - repmat(Acc(3,1),1,size(Acc,2)));
set(svx, 'XData', t, 'YData', vx);
set(svy, 'XData', t, 'YData', vy);
set(svz, 'XData', t, 'YData', vz);

% Set the position data
px = cumtrapz(t, vx);
py = cumtrapz(t, vy);
pz = cumtrapz(t, vz);
set(spx, 'XData', t, 'YData', px);
set(spy, 'XData', t, 'YData', py);
set(spz, 'XData', t, 'YData', pz);



