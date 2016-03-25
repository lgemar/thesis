current_folder = pwd; 
data_folder = (['C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-5', '\Data']); 
file_name = 'test1test1.csv'; 
params1 = [277.17 276.8257 325.7718 253.6234]; % fx fy cx cy, tests 1-5, 7
dC = 38; % (mm), object size ... not sure if this is right

M = csvread([data_folder, '\', file_name]); 
M = M(2:2:end, :); 
t = M(:, 1);
pU = M(:, 2:4)'; % position measurements from the undistorted image, image center already subtracted off

N = size(t,1); 
qref = M(:,21:24); 

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

% Add to the positions the offset between the vicon dots and the tracking
% ball
for i = 1:N
    wRb = quat2rotm(qref(i,:)); 
    Tb = [0 50 0]'; 
    pW(:,i) = pW(:,i) + wRb * Tb;
end

f = figure(1)

subplot(1,3,1)
plot(t, pW(1,:)' / 10, 'r', t, pW(2,:)' / 10, 'g', t, pW(3,:)' / 10, 'b') 
title('State Estimates: $\hat{{p}^{\prime}_S}$', 'Interpreter', 'Latex')
xlabel('Time stamp (s)')
ylabel('Position (cm)')
legend('x_{w}', 'y_{w}', 'z_{w}')
ylim([-75 75])

subplot(1,3,2)
plot(t, 100*M(:, 18), 'r', t, 100*M(:, 19), 'g', t, 100*M(:, 20), 'b')
title('Absolute Position: $p^{\prime}_S$', 'Interpreter', 'Latex')
xlabel('Time stamp (s)')
ylabel('Position (cm)')
legend('x_{w}', 'y_{w}', 'z_{w}')
ylim([-75 75])

subplot(1,3,3)
plot(t, pW(1,:)' / 10 - 100*M(:, 18), 'r', t, pW(2,:)' / 10 - 100*M(:, 19), 'g', t, pW(3,:)' / 10 - 100*M(:, 20), 'b')
str = 'Error: $\epsilon = \hat{{p}^{\prime}_S} - p^{\prime}_S$'; 
title(str, 'Interpreter', 'Latex')
xlabel('Time stamp (s)')
ylabel('Position error (cm)')
legend('\epsilon_x', '\epsilon_y', '\epsilon_z')
ylim([-40 40])

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