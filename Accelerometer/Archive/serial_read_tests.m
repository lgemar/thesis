%% Open the serial port
com_port = 'COM5'; 
baud_rate = 74880; 
s = serial(com_port, 'BaudRate', baud_rate);
s.InputBufferSize = 2048; 
fopen(s); 

%% Read from the serial port
[A, count, msg] = fread(s, 1024, 'int16');
A
count
s.BytesAvailable

%% Clean up the serial port
% Clean up the serial port from the workspace
fclose(s); 
delete(s)
clear s