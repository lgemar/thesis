%% 

T = [1 0 0; 0 0 -1; 0 1 0]; % rotation matrix (A')
pCpr = [0 10 0]'; % body coordinates
pC = T * pCpr; % world coordinates
pCprX = CrossMat(pCpr); % direction of variance in body coordinates
pCX = CrossMat(pC); % direction of variance in world coordinates
Pdthetapr = diag([deg2rad(10).^2 0 deg2rad(10).^2]); % variance of angles in body coordinates

Pdtheta1 = pCX * T * Pdthetapr * T' * pCX'

Pdtheta2 = T * pCprX * Pdthetapr * pCprX' * T'