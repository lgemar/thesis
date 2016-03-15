function qX = CrossMat( q )

if(length(q) ~= 3)
    error('q must be a 3-vector')
end

qX = [0 -q(3) q(2); q(3) 0 -q(1); -q(2) q(1) 0]; 

end

