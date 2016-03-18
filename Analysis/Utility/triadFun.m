function q = triadFun(r1, m1, r2, m2, T)

% r1, r2 == reference vectors (g and b)
% m1, m2 == measurement vectors (gm and bm) 
% T == sensor alignment matrix (sensor to body)
% q == output in column format

% Normalize the reference vectors
r1 = r1 / norm(r1);
r2 = r2 / norm(r2); 

% Compute reference triad
v1 = r1; 
v2 = cross(r1, r2) / norm(cross(r1, r2)); 
v3 = cross(v1, v2); 

% Compute measurement triad
m1 = (T * m1) / norm(m1);
m2 = (T * m2) / norm(m2);
b1 = m1; 
b2 = cross(m1, m2) / norm(cross(m1, m2)); 
b3 = cross(b1, b2); 

% Construct reference and sensor matrices
M_r = cat(2, v1, v2, v3); 
M_b = cat(2, b1, b2, b3);
A = (M_r) * M_b'; % rotation matrix from body to reference frame

% Compute quaternion
q = rotm2quat(A);

q = qflip( q ); % flip to column format

end