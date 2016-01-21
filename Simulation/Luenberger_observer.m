% Define the system parameters
A = [0 1; 0 0]; 
B = [0; 1]; 
C = [1 0];
D = 0;
dt = 0.01; 

% Check observability
Wo=obsv(A,C);

% Design the observer matrix
L=place(A',C',[-1;-1.5])';

% Simulate the acceleration input
sys=ss(A,B,C,0);
[u,t]=gensig('sin',5,20,dt); % 5 s period, 20 s long

figure(1) 
plot(t,u)
title('Acceleration Profile')

% Simulate the position measurement from the acceleration
[y,t,x]=lsim(sys,u,t);

figure(2)
plot(t,y)
title('Ideal output')

% Add noise to the output
y=y+0.1*randn(size(y)); 

% simple observer:
xh1=y;
xh2=diff([0;y])/dt;

% Luenberger observer:
Ao=A-L*C;
sysO=ss(Ao,[L B],eye(2),zeros(2,2));
xh=lsim(sysO,[y u],t);