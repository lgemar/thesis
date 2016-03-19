function A = AMat( q )

% A = XiMat(q)' * PsiMat(q); 

A = (q(4)^2 - norm(q(1:3))^2) * eye(3,3) + 2 * q(1:3) * q(1:3)' - 2 * q(4) * CrossMat(q(1:3)); 

end