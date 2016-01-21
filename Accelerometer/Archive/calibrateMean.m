function means = calibrateMean(com_port, channel)
% channel is 1, 2, or 3.

if( channel == 1 )
    i1 = 1; i2 = 2; i3 = 3; 
elseif( channel == 2 )
    i1 = 4; i2 = 5; i3 = 6; 
elseif( channel == 3 )
    i1 = 7; i2 = 8; i3 = 9; 
else
    error('Invalid channel identifier')
end
    
% Number of samples
N = 100; 

% Open the serial port
baud_rate = 38400; 
s = serial(com_port, 'BaudRate', baud_rate);
fopen(s); 

% Set the format specification for reading from the serial port
formatspec  = ['a/g/m:\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\n']; 

% Initialize the 
% The try-catch structure here ensures that the comport doesn't get
% screwed up with errors
D = [];
taxis = []; 
tic
idx = 1; 
while (idx <= N)
		data = fscanf(s, formatspec); %do some process
		if( size(data) ~= 9 )
			continue; % If the serial read was bad, try again
		end

		t = toc;
		taxis = [taxis t]; 

		% Calibration for accelerometer: 2^14 ~ 1g
		x = data(i1); 
		y = data(i2); 
		z = data(i3);
		D = [D; x y z];

        % Clean up the loop
        idx = idx + 1; 
end

% Clean up the serial port from the workspace
fclose(s); 
delete(s)
clear s

means = mean(D); 
