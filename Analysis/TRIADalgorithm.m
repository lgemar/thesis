current_folder = pwd; 
data_folder = ([pwd, '\Data']); 
file_name = 'test8test8.csv'; 
M = csvread([data_folder, '\', file_name]); 
M = M(2:end, :); 
N = size(M, 1); 

QuatRef = M(:, 21:24); % (w, x, y, z)
AccelData = M(:, 9:11); 
MagnData = M(:, 15:17); 

QuatEst = zeros(size(QuatRef)); 

for i = 1:N
  
    % v1 = [0; 0; -9.81]; 
    % v2 = [18006; -1566; 53490]; 
    v1 = [0; 0; -9.81]; 
    v2 = [1; 1.4; 0]; 
    v1 = v1 / norm(v1); 
    v2 = v2 / norm(v2); 
    
    r1 = v1; 
    r2 = cross(v1, v2) / norm(cross(v1, v2)); 
    r3 = cross(r1, r2); 
    
    T = [0 1 0; -1 0 0; 0 0 -1]; % sensor alignment matrix
    w1 = AccelData(i, :)'; w1 = T * w1; 
    w2 = MagnData(i, :)'; w2 = T * w2; 
    w1 = w1 / norm(w1);
    w2 = w2 / norm(w2);
    
    b1 = w1; 
    b2 = cross(w1, w2) / norm(cross(w1, w2)); 
    b3 = cross(b1, b2); 
    
    M_r = cat(2, r1, r2, r3); 
    M_b = cat(2, b1, b2, b3);
   
    R = M_b * M_r'; 
    
    A = R;  
    
    q = rotm2quat(A'); 
    QuatEst(i, :) = q; 
end

figure(1)
subplot(1,2,1)
plot(rad2deg(quat2eul(QuatEst)))
title('Tool Orientation (Estimate)')
legend('Yaw', 'Pitch', 'Roll')
subplot(1,2,2)
plot(rad2deg(quat2eul(QuatRef)))
title('Tool Orientation (Absolute)')
legend('Yaw', 'Pitch', 'Roll')
