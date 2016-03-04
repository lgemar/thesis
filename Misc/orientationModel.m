function qtool = orientationModel(atool,gtool,qtoolprev,dt)

% Algorithm taken from handout. General description: 
% - accelerometer tells us: "You are now at position Racc" 
% - we say "Thank you, but let me chec." 
% - then correct this information with gyroscope data as well as past Rest
%   data and then output a new estimated vector Rest
% - We consider Rest to be our "best bet" as to the current position of
%   the device

% Racc is the intertial force vector from accelerometer
Racc = atool / norm(atool); % Normalize so that length is always 1

% Use previous orientation and angular velocities to estimate new
% orientation

RestPrev = flip(quat2eul(qtoolprev)); 
Rgyro = gtool; 

AxzPrev = atan2(RestPrev(1),RestPrev(3)); 
AyzPrev = atan2(RestPrev(2),RestPrev(3)); 

Axz = AxzPrev + Rgyro(2) * dt; 
Ayz = AyzPrev + Rgyro(1) * dt; 

RxGyro = sin(Axz) / sqrt(1 + cos(Axz)^2 * tan(Ayz)^2); 
RyGyro = sin(Ayz) / sqrt(1 + cos(Ayz)^2 * tan(Axz)^2); 

if( (RestPrev(3) >= 0) ) sign = 1; else sign = -1; end
RzGyro = sign * sqrt(1 - RxGyro^2 - RyGyro^2); 

Rgyro = [RxGyro RyGyro RzGyro]; 

% Combine estimates from the gyroscope and the accelerometer
wGyro = 1; % How we trust the gyro compared to accelerometer (Range: ~5-20) 
Rest = (Racc + Rgyro * wGyro) / (1 + wGyro); 
Rest = Rest / norm(Rest); 

% Compute quaternion
qtool = eul2quat(flip(Rest)); 




