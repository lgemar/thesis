current_folder = pwd; 
data_folder = ([pwd, '\Data']); 
file_name = 'test1test1.csv'; 
M = csvread([data_folder, '\', file_name]); 
M = M(2:end, :); 
N = size(M, 1); 

QuatRef = M(:, 21:24); % (w, x, y, z)
AccelData = M(:, 9:11); 
MagnData = M(:, 15:17); 

QuatEst = zeros(size(QuatRef)); 

for i = 1:N
  
    v1 = [0; 0; 9.81];
    v2 = [0.3; -1; 0]; % v2 = [0.3; -1; 0];
    v1 = v1 / norm(v1); 
    v2 = v2 / norm(v2); 
    
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
end

figure(1)
subplot(1,2,1)
plot(rad2deg(quat2eul(QuatEst)))
legend('Yaw', 'Pitch', 'Roll')
% plot(QuatEst)
% legend('w', 'x', 'y', 'z')
title('Tool Orientation (Estimate)')
subplot(1,2,2)
plot(rad2deg(quat2eul(cat(2, QuatRef(:,1), QuatRef(:, 2:4)))))
legend('Yaw', 'Pitch', 'Roll')
% plot(QuatRef)
% legend('w', 'x', 'y', 'z')
title('Tool Orientation (Absolute)')

