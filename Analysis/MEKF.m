current_folder = pwd; 
data_folder = ([pwd, '\Data']); 
file_name = 'test1test1.csv'; 
M = csvread([data_folder, '\', file_name]); 
M = M(2:end, :); 
N = size(M, 1); % Number of samples

% Data vectors
QuatRef = M(:, 21:24); % (w, x, y, z)
AccelData = M(:, 9:11); 
GyroData = M(:, 12:14); 
MagnData = M(:, 15:17); 

% Time vector
t = M(:, 1); t = t - min(t); 

% Reference vectors in world coordinates
gw = [0; 0; 9.81]; % gravity vector
bw = [0.3; -1; 0]; % magnetic field vector (unsure)

% Initalize Kalman Filter Variables
dT = t(2) - t(1); 
Sr = 0.1; Sq = 1; % trust measurements, trust model
Q = Sq * dT * eye(4,4); 
R = Sr * dT * eye(6,6); 

% Intialization of covariance (arbitrary)
P = 0.01 * dT * eye(4,4); 

% Intialization of state (use triad algorithm).
% Note: the quaternion should be referenced from world to body coordinates
v1 = gw; v2 = bw; 
r1 = v1; 
r2 = cross(v1, v2) / norm(cross(v1, v2)); 
r3 = cross(r1, r2); 


