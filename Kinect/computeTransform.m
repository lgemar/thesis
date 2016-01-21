function T = computeTransform(file_name, recompute_reg_points)

if(recompute_reg_points == 1)
    % Define the world coordinates of interest
    w0 = [0, 0, 0]; 
    w1 = [0.4, 0, 0]; 
    w2 = [0, 1, 0]; 
    w3 = [0, 1, 0.5]; 

    % Find virtual coordinates
    v0 = findVirtualCoord(w0);
    v1 = findVirtualCoord(w1); 
    v2 = findVirtualCoord(w2); 
    v3 = findVirtualCoord(w3); 

    %% Set up the transformation problem: X*b = y

    reg_points = [w0', w1', w2', w3', v0', v1', v2', v3']; 

    % Write registration points to file

    csvwrite([file_name, '_registration_points.csv'], reg_points); 
else
    reg_points = csvread([file_name, '_registration_points.csv']);

    w0 = reg_points(:, 1)';
    w1 = reg_points(:, 2)';
    w2 = reg_points(:, 3)';
    w3 = reg_points(:, 4)';

    v0 = reg_points(:, 5)';
    v1 = reg_points(:, 6)';
    v2 = reg_points(:, 7)';
    v3 = reg_points(:, 8)';
end

% Compute the X matrix
temp1 = [v0 1 zeros(1,12); v1 1 zeros(1,12); v2 1 zeros(1,12); v3 1 zeros(1,12)]; 
temp2 = [zeros(1,4), v0, 1, zeros(1,8); zeros(1,4), v1, 1, zeros(1,8); zeros(1,4), v2, 1, zeros(1,8); zeros(1,4), v3, 1, zeros(1,8)];
temp3 = [zeros(1,8), v0, 1, zeros(1,4); zeros(1,8), v1, 1, zeros(1,4); zeros(1,8), v2, 1, zeros(1,4); zeros(1,8), v3, 1, zeros(1,4)];
temp4 = [zeros(1,12) v0 1; zeros(1,12) v1 1; zeros(1,12) v2 1; zeros(1,12) v3 1];

X = [temp1; temp2; temp3; temp4]; 

% Compute the y matrix
y = [w0(1); w1(1); w2(1); w3(1); w0(2); w1(2); w2(2); w3(2); w0(3); w1(3); w2(3); w3(3); 1; 1; 1; 1];

% Solve for b and reshape
b = (X' * X) \ X' * y;
T = [b(1:4)'; b(5:8)'; b(9:12)'; b(13:16)']; 

% Write to file
csvwrite([file_name, '_transform.csv'], T); 
