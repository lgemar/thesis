% Number of samples
N = 1000; 

% Open the serial port
com_port = 'COM5'; 
baud_rate = 38400; 
s = serial(com_port, 'BaudRate', baud_rate);
fopen(s); 

% Set the format specification for reading from the serial port
formatspec  = ['a/g/m:\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t']; 

% Allocate memory for the various data streams
t = zeros(1, N); 
ax = zeros(1, N); ay = zeros(1, N); az = zeros(1, N); 
gx = zeros(1, N); gy = zeros(1, N); gz = zeros(1, N); 
mx = zeros(1, N); my = zeros(1, N); mz = zeros(1, N);

idx = 1; 
while (idx <= N)
    data = fscanf(s, formatspec); %do some process
    t(idx) = datenum(datetime('now')); 
    ax(idx) = data(1); ay(idx) = data(2); az(idx) = data(3); 
    gx(idx) = data(4); gy(idx) = data(5); gz(idx) = data(6); 
    mx(idx) = data(7); my(idx) = data(8); mz(idx) = data(9); 
    
    % Clean up the loop
    idx = idx + 1; 
    % drawnow %flush the graphics queue
end

% Set the time axis to be the time (s) from the start of the trial
taxis = 86400*(t - min(t)); 

% Open the figures
h1 = figure(1); h2 = figure(2); h3 = figure(3); 

% Plot the accelerometer data
figure(h1) 
subplot(3, 1, 1); plot(taxis, ax); 
subplot(3, 1, 2); plot(taxis, ay); 
subplot(3, 1, 3); plot(taxis, az); 

% Plot the gyroscope data
figure(h2) 
subplot(3, 1, 1); plot(taxis, gx); 
subplot(3, 1, 2); plot(taxis, gy); 
subplot(3, 1, 3); plot(taxis, gz); 

% Plot the magnetometer data
figure(h3) 
subplot(3, 1, 1); plot(taxis, mx); 
subplot(3, 1, 2); plot(taxis, my); 
subplot(3, 1, 3); plot(taxis, mz); 

% Clean up the serial port from the workspace
fclose(s)
delete(s)
clear s