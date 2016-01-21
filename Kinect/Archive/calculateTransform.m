function A = calculateTransform(date)

%% Read the reference data into Matlab
file_name = ['reference_frame_' date]; 
path = ['C:\Users\Lukas Gemar\Documents\MATLAB\es100\Kinect\Calibration\', file_name, '.xlsx'];
num = xlsread(path, 'B2:C13'); 

%% Set up the transformation matrices

% In real world coordinates
wo = num(1:3,1); 
w1 = num(4:6,1) - wo; 
w2 = num(7:9,1) - wo; 
w3 = num(10:12,1) - wo; 

% In virtual coordinates
vo = num(1:3,2); 
v1 = num(4:6,2) - vo; 
v2 = num(7:9,2) - vo; 
v3 = num(10:12,2) - vo; 

%% Set up the transformation problem: X*b = y

% Compute the X matrix
temp1 = [v1' zeros(1,6); v2' zeros(1,6); v3' zeros(1,6)]; 
temp2 = [zeros(1,3), v1', zeros(1,3); zeros(1,3), v2', zeros(1,3); zeros(1,3), v3', zeros(1,3)];
temp3 = [zeros(1,6) v1'; zeros(1,6) v2'; zeros(1,6) v3'];

X = [temp1; temp2; temp3]; 

% Compute the y matrix
y = [w1(1); w2(1); w3(1); w1(2); w2(2); w3(2); w1(3); w2(3); w3(3)];

% Solve for b and reshape
b = (X' * X) \ X' * y;
A = horzcat([b(1:3)'; b(4:6)'; b(7:9)'; zeros(1,3)], [-vo; 1]); 
