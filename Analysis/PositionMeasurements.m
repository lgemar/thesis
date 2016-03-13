current_folder = pwd; 
data_folder = ([pwd, '\Data']); 
file_name = 'test3test3.csv'; 
params1 = [277.17 276.8257 325.7718 253.6234]; % fx fy cx cy, tests 1-5, 7
dC = 25; % (mm), object size ... not sure if this is right

M = csvread([data_folder, '\', file_name]); 
M = M(2:2:end, :); 
t = M(:, 1);
pU = M(:, 2:4)'; % position measurements from the undistorted image, image center already subtracted off

params = params1; 

fx = params(1); 
fy = params(2); 
cx = params(3); cy = params(4); % don't need these, cx, cy already subtracted

% Direct observations
zC = dC ./ (pU(3,:) * sqrt( 1/fx^2 + 1/fy^2 )); 
yC = zC .* pU(2,:) / fy; 
xC = zC .* pU(1,:) / fx; 

pC = cat(1, xC, yC, zC); % position of the object in camera coordinates

% Extrinsic camera parameters
R1 = [1 0 0; 0 0 -1; 0 1 0]; 
R2 = [0 1 0; -1 0 0; 0 0 1];
R3 = [1 0 0; 0 1 0; 0 0 1]; 

Ralign = R3 * R2 * R1; 
Talign = [38.2; -18.3; 490.5]; 
pW = Ralign * pC + repmat(Talign, 1, size(pC, 2)); % position of the object in world coordinates

f = figure(1)
subplot(1,3,1)
plot(t, pW')
title('State Estimates: $\hat{\vec{x}}$', 'Interpreter', 'Latex')
xlabel('Time stamp (s)')
ylabel('Position (mm)')
legend('x_{w}', 'y_{w}', 'z_{w}')
subplot(1,3,2)
plot(t, 1000*M(:, 18:20))
title('Absolute Position: $\vec{x}$', 'Interpreter', 'Latex')
xlabel('Time stamp (s)')
ylabel('Position (mm)')
legend('x_{w}', 'y_{w}', 'z_{w}')
subplot(1,3,3)
plot(t, pW' - 1000*M(:, 18:20))
str = 'Error: $\epsilon = \hat{\vec{x}} - \vec{x}$'; 
title(str, 'Interpreter', 'Latex')
xlabel('Time stamp (s)')
ylabel('Position error (mm)')
legend('\epsilon_x', '\epsilon_y', '\epsilon_z')

% % create the data
% d = [1 2 3; 4 5 6; 7 8 9];
% 
% % Create the column and row names in cell arrays 
% cnames = {'E[\epsilon_i]','E[\','Z-Data'};
% rnames = {'First','Second','Third'};
% 
% % Create the uitable
% t = uitable(f,'Data',d,...
%             'ColumnName',cnames,... 
%             'RowName',rnames);
% 
% % Set width and height
% t.Position(3) = t.Extent(3);
% t.Position(4) = t.Extent(4);