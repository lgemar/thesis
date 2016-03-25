%% Read in Data
current_folder = pwd; 
data_folder = (['C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-5', '\Data']); 
file_name = 'test4test4.csv'; 
params1 = [277.17 276.8257 325.7718 253.6234]; % fx fy cx cy, tests 1-5, 7
params4 = [279.9386 279.0004 327.9626 247.2915]; % fx fy cx cy, tests 1-5, 7

dC = 38; % (mm), object size ... not sure if this is right

M = csvread([data_folder, '\', file_name]); 
M = M(2:2:end, :);

% Time vector
t = M(:, 1); t = t - min(t); 
M = M(t < 25, :); 
t = t(t < 25); 

N = size(t,1); 
qref = M(:,21:24); 

dt = mean(diff(t(~isnan(t)))); 
% position measurements from the undistorted image, image center already subtracted off
pU = M(:, 2:4)'; 

% Intrinsic camera parameters
params = params1; 
fx = params(1); 
fy = params(2); 
cx = params(3); cy = params(4); 
% Extrinsic camera parameters
R1 = [1 0 0; 0 0 -1; 0 1 0]; 
R2 = [0 1 0; -1 0 0; 0 0 1];
Ralign = R2 * R1; 
Talign = [38.2; -18.3; 490.5]; 
% Direct observations: position of the object in camera coordinates
zC = dC ./ (pU(3,:) * sqrt( 1/fx^2 + 1/fy^2 )); 
yC = zC .* pU(2,:) / fy; 
xC = zC .* pU(1,:) / fx; 
pC = cat(1, xC, yC, zC); 
% position of the object in world coordinates
pW = Ralign * pC + repmat(Talign, 1, size(pC, 2)); 

% Add to the positions the offset between the vicon dots and the tracking
% ball
for i = 1:N
    wRb = quat2rotm(qref(i,:)); 
    Tb = [0 60 0]'; 
    pW(:,i) = pW(:,i) + wRb * Tb;
end

%% Initialize Kalman Filter

% Initalize Variables
T = dt; 
Sq = 1e-2; Sr = 1; 
Q2 = (Sq / dt) * [dt^4/4 dt^3/2 dt^2/2; dt^3/2 dt^2/2 dt; dt^2/2 dt 1]; 
Q = blkdiag(Q2, Q2, Q2); 
R = Sr * eye(3,3); 

% State transition and observation matrices
F2 = [1 T T^2; 0 1 T; 0 0 1]; A = blkdiag(F2, F2, F2); 
H = blkdiag([1 0 0], [1 0 0], [1 0 0]); 

%% Run Kalman Filter

% Initialization
x = zeros(9, N); 
x(:, 1) = [pW(1, 1) 0  0 pW(2, 1) 0  0 pW(3, 1) 0 0]'; % start at 0 velocity
P = 1 * T * eye(9,9);

for i = 2:N
    % Update transition matrix
    T = dt; 
    F2 = [1 T T^2/2; 0 1 T; 0 0 1]; A = blkdiag(F2, F2, F2); 

    % Prediction 
    Ppred = A * P * A' + Q; 
    xpred = A * x(:, i-1);  

    % Update
    P = inv( inv(Ppred) + H' * R' * H ); 
    K = P * H' * R'; 
    x(:, i) = xpred + K * (pW(:, i) - H * xpred);
end

% Rescale the results to cm
pW = pW / 10; 
x = x / 10; 
M(:,18:20) = 100*M(:, 18:20); 

% Error of naive estimates
naiveerr = pW' - M(:, 18:20); 
naivebias = nanmean( naiveerr  ); 
naivevar = nanvar( naiveerr ); 

% error of filtered estimates
err = x([1 4 7], :)' - M(:, 18:20);
bias = nanmean(err); 
variance = nanvar(err); 

% Compute the error by time, error mean, error variance

figure(1)
plot(t, M(:, 18), 'r', t, M(:, 19), 'g', t, M(:, 20), 'b')
title('Absolute Position: $p^{\prime}_S$', 'Interpreter', 'Latex')
xlabel('Time stamp (s)')
ylabel('Position (cm)')
legend('x_{w}', 'y_{w}', 'z_{w}')
ylim([-75 75])

figure(2)
subplot(2,2,1)
plot(t, pW(1,:)', 'r', t, pW(2,:)', 'g', t, pW(3,:)', 'b') 
title('Naive state estimates: $\hat{{p}^{\prime}_S}$', 'Interpreter', 'Latex')
xlabel('Time stamp (s)')
ylabel('Position (cm)')
legend('x_{w}', 'y_{w}', 'z_{w}')
ylim([-75 75])

subplot(2,2,2)
plot(t, x([1], :)', 'r', t, x([4], :)', 'g', t, x([7], :)', 'b')
title('Filtered state estimates: $\hat{{p}^{\prime}_S}$', 'Interpreter', 'Latex')
xlabel('Time stamp (s)')
ylabel('Position (cm)')
legend('x_{w}', 'y_{w}', 'z_{w}')
ylim([-75 75])

subplot(2,2,3)
plot(t, naiveerr(:,1), 'r', t, naiveerr(:,2),'g', t, naiveerr(:,3), 'b')
str = 'Naive error: $\epsilon = \hat{{p}^{\prime}_S} - p^{\prime}_S$'; 
title(str, 'Interpreter', 'Latex')
str = ['$ E[\epsilon] = \left[\begin{array}{c c c}', num2str(naivebias(1)), ' & ' ...
                                                     num2str(naivebias(2)), ' & ' ...
                                                     num2str(naivebias(3)), ' \end{array} \right]^{T} $'];
                                                 
text(3,30,str,'Interpreter','latex')
str = ['$ var[\epsilon] = \left[\begin{array}{c c c}', num2str(naivevar(1)), ' & ' ...
                                                     num2str(naivevar(2)), ' & ' ...
                                                     num2str(naivevar(3)), ' \end{array} \right]^{T} $'];
                                                 
text(3,20,str,'Interpreter','latex')
xlabel('Time stamp (s)')
ylabel('Position error (cm)')
legend('\epsilon_x', '\epsilon_y', '\epsilon_z')
ylim([-40 40])


subplot(2,2,4)
plot(t, err(:,1), 'r', t, err(:,2),'g', t, err(:,3), 'b')
str = 'Filtered error: $\epsilon = \hat{{p}^{\prime}_S} - p^{\prime}_S$'; 
title(str, 'Interpreter', 'Latex')
str = ['$ E[\epsilon] = \left[\begin{array}{c c c}', num2str(bias(1)), ' & ' ...
                                                     num2str(bias(2)), ' & ' ...
                                                     num2str(bias(3)), ' \end{array} \right]^{T} $'];
text(3,30,str,'Interpreter','latex')
str = ['$ var[\epsilon] = \left[\begin{array}{c c c}', num2str(variance(1)), ' & ' ...
                                                     num2str(variance(2)), ' & ' ...
                                                     num2str(variance(3)), ' \end{array} \right]^{T} $'];
                                                 
text(3,20,str,'Interpreter','latex')
xlabel('Time stamp (s)')
ylabel('Position error (cm)')
legend('\epsilon_x', '\epsilon_y', '\epsilon_z')
ylim([-40 40])

% Plot error as a function of position
figure(3)
scatter(abs(pU(1,:)'), abs(err(:,2)))
title('Error vs distance from image center')
xlabel('Distance from center (pixels)')
ylabel('Error (cm)')

% Plot error as a function of velocity
% Plot error as a function of position
figure(4)

subplot(1,3,1)
scatter(abs(diff(x(1,:))' / dt), abs(err(2:end,1)))
xlabel('Tool velocity (cm / s)')
ylabel('|\epsilon_X| (cm)')
xlim([0 20])
ylim([0 20])

subplot(1,3,2)
scatter(abs(diff(x(4,:))' / dt), abs(err(2:end,2)))
xlabel('Tool velocity (cm / s)')
ylabel('|\epsilon_Y| (cm)')
xlim([0 20])
ylim([0 20])

subplot(1,3,3)
scatter(abs(diff(x(7,:)') / dt), abs(err(2:end,3)))
xlabel('Tool velocity (cm / s)')
ylabel('|\epsilon_Y| (cm)')
xlim([0 20])
ylim([0 20])



