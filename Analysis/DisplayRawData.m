current_folder = pwd; 
data_folder = ([pwd, '\Data']); 
file_name = 'test3test3.csv'; 

M = csvread([data_folder, '\', file_name]); 
M = M(2:end, :); 

figure(1)
subplot(1,2,1)
plot(M(:,1), M(:, 2:4))
title('State observations')
xlabel('Time stamp (s)')
ylabel('Magnitude (pixels)')
legend('x_{u}', 'y_{u}', 'd_{u}')
subplot(1,2,2)
plot(M(:,1), 1000*M(:, 18:20))
title('Absolute Position')
xlabel('Time stamp (s)')
ylabel('Position (mm)')
legend('x_{w}', 'y_{w}', 'z_{w}')


figure(2)
subplot(1,2,1)
plot(quat2eul(M(:, 5:8)))
title('Tool Orientation (Estimate)')
legend('Yaw', 'Pitch', 'Roll')
subplot(1,2,2)
plot(M(:, 21:24))
title('Tool Orientation (Absolute)')
legend('Yaw', 'Pitch', 'Roll')


