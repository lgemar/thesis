current_folder = pwd; 
data_folder = ([pwd, '\Data']); 
file_name = 'test1test1.csv'; 
M = csvread([data_folder, '\', file_name]); 
M = M(2:end, :); 
N = size(M, 1); 

% Time vector
t = M(:, 1); t = t - min(t); 

QuatRef = M(:, 21:24); % (w, x, y, z)
AccelData = M(:, 9:11); 
MagnData = M(:, 15:17); 

QuatEst = zeros(size(QuatRef)); 
AccelEst = zeros(size(AccelData)); 
MagnEst = zeros(size(MagnData));

NTrials = 100; 
ad = linspace(0,2*pi,NTrials); 
DegError = zeros(NTrials,3); 
QuatError = zeros(NTrials,4); 

for j = 1:NTrials

    for i = 1:N

        gw = [0; 0; 9.81];
        %bw = [cos(pi)*19.5; sin(pi)*(-5.2); -48.2]; 
        bw = [cos(ad(j))*19.5; sin(ad(j))*-5.2; -48.2]; 
        v1 = gw / norm(gw); 
        v2 = bw / norm(bw); 

        r1 = v1; 
        r2 = cross(v1, v2) / norm(cross(v1, v2)); 
        r3 = cross(r1, r2); 

        T = [0 1 0; -1 0 0; 0 0 1]; % sensor alignment matrix from body to sensor

        m1 = AccelData(i, :)'; % w1 = T * w1; 
        m2 = MagnData(i, :)'; % w2 = T * w2; 
        m1 = m1 / norm(m1);
        m2 = m2 / norm(m2);

        s1 = m1; 
        s2 = cross(m1, m2) / norm(cross(m1, m2)); 
        s3 = cross(s1, s2); 

        M_r = cat(2, r1, r2, r3); 
        M_s = cat(2, s1, s2, s3);

        R = (T' * M_s) * M_r';

        q = rotm2quat(R'); 
        QuatEst(i, :) = [q(1), q(2), q(3), q(4)]; 

        % Initial State Estimate (Qe)
        Qe = [q(2); q(3); q(4); q(1)]; 

        % Check the observation model accuracy
        AccelEst(i,:) = (T * AMat( Qe ) * v1);
        MagnEst(i,:) = (T * AMat( Qe ) * v2);

    end
    
    DegEst = rad2deg( quat2eul( cat(2, QuatEst(:,1), QuatEst(:, 2:4) ) ) );
    DegRef = rad2deg( quat2eul( cat(2, QuatRef(:,1), QuatRef(:, 2:4) ) ) );
    DegError(j,:) = mean(DegEst - DegRef,1); 
    QuatError(j,:) = mean(QuatEst - QuatRef,1); 

end

f = figure(1); 

subplot(2,2,1)
DegEst = rad2deg( quat2eul( cat(2, QuatEst(:,1), QuatEst(:, 2:4) ) ) );
DegRef = rad2deg( quat2eul( cat(2, QuatRef(:,1), QuatRef(:, 2:4) ) ) );
plot(t, DegEst - DegRef )
str = 'Euler Angle Error: $\epsilon_{\theta} = \hat{\vec{\theta}} - \vec{\theta}$'; 
title(str, 'Interpreter', 'Latex')
xlabel('Time stamp (s)')
ylabel('Error (degrees)', 'Interpreter', 'Latex')
legend('\epsilon_{\theta_Z}', '\epsilon_{\theta_Y}', '\epsilon_{\theta_X}')

subplot(2,2,2)
plot( t, AccelEst - AccelData ./ repmat(sqrt(sum(AccelData.^2,2)),1,3) )
legend('\epsilon_{a_x}', '\epsilon_{a_y}', '\epsilon_{a_z}')
title('Accelerometer Observation Error: $\hat{\vec{a}} - \vec{a}$', 'Interpreter', 'Latex')

subplot(2,2,3)
plot( t, MagnEst - MagnData ./ repmat(sqrt(sum(MagnData.^2,2)),1,3)  )
legend('m_x', 'm_y', 'm_z')
title('Magnetometer Observation Error: $\hat{\vec{m}} - \vec{m}$', 'Interpreter', 'Latex')

subplot(2,2,4)

% plot( rad2deg(ad), DegError )
% str = 'Expected Euler Angle Error: $E[\epsilon_{\theta}] = E[\hat{\vec{\theta}} - \vec{\theta}]$'; 
% title(str, 'Interpreter', 'Latex')
% xlabel('$\vec{b}$ field displacement from north (degrees)', 'Interpreter', 'Latex')
% ylabel('Error (degrees)', 'Interpreter', 'Latex')
% legend('\epsilon_{\theta_Z}', '\epsilon_{\theta_Y}', '\epsilon_{\theta_X}')

QuatErrorMag = sqrt(sum(QuatError.^2,2)); 
plot( rad2deg(ad), QuatErrorMag )
str = 'Expected Quaternion Error: $| E[\epsilon_{\overline{{q}}}] | = | E[\hat{\overline{{q}}} - \overline{{q}}] |$'; 
title(str, 'Interpreter', 'Latex')
xlabel('$\vec{b}$ field displacement from north (degrees)', 'Interpreter', 'Latex')
ylabel('Error', 'Interpreter', 'Latex')
legend('\epsilon_{q_w}', '\epsilon_{q_x}', '\epsilon_{q_y}','\epsilon_{q_z}')

MinErrorAngle = ad( QuatErrorMag == min(QuatErrorMag) ); 
disp(['Minimum quaternion error occurs at, ' num2str( MinErrorAngle ), ' rad or ', num2str( rad2deg( MinErrorAngle ) ), ' degrees']); 

f2 = figure(2); 

subplot(1,2,1)
DegEst = rad2deg( quat2eul( cat(2, QuatEst(:,1), QuatEst(:, 2:4) ) ) );
plot( DegEst )
legend('\theta_Z', '\theta_Y', '\theta_X')
% plot( QuatEst )
% legend('w', 'x', 'y', 'z')
title('Orientation Estimate: $\hat{\vec{\theta}}$', 'Interpreter', 'Latex')

subplot(1,2,2)
DegRef = rad2deg( quat2eul( cat(2, QuatRef(:,1), QuatRef(:, 2:4) ) ) );
plot( DegRef )
legend('\theta_Z', '\theta_Y', '\theta_X')
% plot(QuatRef)
% legend('w', 'x', 'y', 'z')
title('Orientation Reference: $\vec{\theta}$', 'Interpreter', 'Latex')


