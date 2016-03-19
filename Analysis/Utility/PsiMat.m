function x = PsiMat( q )

if(length(q) ~= 4)
    error('q must be a 4-vector quaternion')
end

x = [q(4) * eye(3,3) - CrossMat(q(1:3)); -q(1:3)']; 
    
end