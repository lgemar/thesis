function errorout = spatialError(trajectory, point)
% Trajectory is a NxD array, where N is the number of points that define
%   the trajectory and D is the dimension of the coordinate space
% Point is some point on the coordinate plane with dimensions 1xD

N = size(trajectory,1); 
D = size(trajectory,2); 
error_values = zeros(1,N-1); 
P = point; 

for i=1:(N-1)
    Q1 = trajectory(i,:); 
    Q2 = trajectory(i+1,:);
    if( D == 2 )
        error_values(i) = norm(det([Q2-Q1;P-Q1]))/norm(Q2-Q1); % P,Q1,Q2 are row vectors
    elseif( D == 3)
        error_values(i) = norm(cross(Q2-Q1,P-Q1))/norm(Q2-Q1);
    else 
        error('The dimension of the input trajectory points doesnt make sense'); 
    end
end
errorout = min(error_values); 