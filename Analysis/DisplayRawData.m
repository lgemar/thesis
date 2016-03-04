current_folder = pwd; 
data_folder = ([pwd, '\Data']); 
file_name = 'test5test5.csv'; 

M = csvread([data_folder, '\', file_name]); 
M = M(2:end, :); 

figure(1)
subplot(1,2,1)
plot(M(:, 2:4))
title('Camera Position')
legend('x', 'y', 'z')
subplot(1,2,2)
plot(M(:, 18:20))
title('Absolute Position')
legend('x', 'y', 'z')


figure(2)
subplot(1,2,1)
plot(quat2eul(M(:, 5:8)))
title('Tool Orientation (Estimate)')
legend('Yaw', 'Pitch', 'Roll')
subplot(1,2,2)
plot(M(:, 21:24))
title('Tool Orientation (Absolute)')
legend('Yaw', 'Pitch', 'Roll')


