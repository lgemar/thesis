%% 

T = [1 0 0; 0 0 -1; 0 1 0]; 
pCpr = [0 10 0]'; 
pC = T * pCpr; 
pCprX = CrossMat(pCpr); 
pCX = CrossMat(pC); 
Pdthetapr = diag([deg2rad(10).^2 0 deg2rad(10).^2]); 

Pdtheta1 = pCX * T * Pdthetapr * T' * pCX'

Pdtheta2 = T * pCprX * Pdthetapr * pCprX' * T'