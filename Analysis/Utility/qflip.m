function qout = qflip(qin) 

    if( size(qin,1) == 4 ) 
         % If the quaternion is in row format, assume that it is written
         % scalar-first
        qout = [qin(4), qin(1:3)'];
    elseif (size(qin,2) == 4)
        % If the quaternion is in column format, assume that it is written
        % scalar-last
        qout = [qin(2:4)'; qin(1)];
    else
        error('Invalid input quaternion')
    end
    
end