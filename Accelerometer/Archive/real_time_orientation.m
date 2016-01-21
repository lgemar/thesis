com_port = 'COM5'; 
acqSize=500;
addpath('..\Orientation\KalmanFilter_V11\');

%% Gyroscope statistics
disp('Calibrating...')
resolution = (250 / 2^15); % degrees/s/bit
% Calibrate functions require exclusive access to the comport
gyro_means = resolution * calibrateMean(com_port, 2);
gyro_var = (resolution * calibrateStd(com_port, 2)).^2; 
Offset=[gyro_means(1),gyro_means(2),gyro_means(3)]';
var=[gyro_var(1) gyro_var(2) gyro_var(3)]';
% Previous: var=[(0.7698/180*pi)^2 (0.4925/180*pi)^2 (0.5144/180*pi)^2]';

%% Acquisition variables
GyroRate=zeros(3,acqSize);
Acc=zeros(3,acqSize);
Magn=zeros(3,acqSize);
Angles=zeros(3,acqSize);
AccF=zeros(3,acqSize);
MagnF=zeros(3,acqSize);
mu=zeros(1,acqSize);
dqnorm=zeros(1,acqSize);
dq=zeros(4,acqSize);

qUpdate=zeros(4,acqSize);

%Initial quaternion values
qUpdate(:,1)=[1 0 0 0]';

%Observation vector
qObserv=zeros(4,acqSize);
qObserv(:,1)=[1 0 0 0]';

%% KALMAN MATRIXES
% PROCESS COVARIANCE, Q
Q1=[var(1,1)+var(2,1)+var(3,1) -var(1,1)+var(2,1)-var(3,1) -var(1,1)-var(2,1)+var(3,1) var(1,1)-var(2,1)-var(3,1)];
Q2=[-var(1,1)+var(2,1)-var(3,1) var(1,1)+var(2,1)+var(3,1) var(1,1)-var(2,1)-var(3,1) -var(1,1)-var(2,1)+var(3,1)];
Q3=[-var(1,1)-var(2,1)+var(3,1) var(1,1)-var(2,1)-var(3,1) var(1,1)+var(2,1)+var(3,1) -var(1,1)+var(2,1)-var(3,1)];
Q4=[var(1,1)-var(2,1)-var(3,1) -var(1,1)+var(2,1)-var(3,1) -var(1,1)+var(2,1)-var(3,1) var(1,1)+var(2,1)+var(3,1)];
Qmatrix=[Q1;Q2;Q3;Q4];

% MEASUREMENT MATRIX, H
H=eye(4,4);

% MEAUREMENT COVARIANCE, R
sigmaR=[0.01 0.01 0.01 0.01]';
R=[sigmaR(1,1) 0 0 0;0 sigmaR(2,1) 0 0;0 0 sigmaR(3,1) 0;0 0 0 sigmaR(4,1)];

% VARIABLE INITIALIZATION
qPredicted=zeros(4,acqSize); %qPredicted(:,1)=[0.5 0.5 0.5 0.5]';
qPredicted(:,1)=[1 0 0 0]';
P_Update=eye(4,4)*2;

%% Connect to the IMU device
% Previous: [handle_dev pFD]=INEMO_Connection();

% Open the serial port
disp('Open the serial port...')
baud_rate = 38400; 
s = serial(com_port, 'BaudRate', baud_rate);
fopen(s); 
formatspec  = ['a/g/m:\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\n']; 

%% Set up the filters and parameters for acquisition
t=[0];

i=1;
dt=0;

[bAcc,aAcc] = butter(3,0.0075,'low');
[bMagn,aMagn] = butter(2,0.06,'low');

magnF_Length=13;
accF_Length=13;

disp('Bring up the filters...')
while(i<=accF_Length+4)
    if(i>1)
        dt = toc(t0);
        t=[t t(length(t))+dt];
    end

    % Previous: [errre pFD]=calllib('iNEMO2_SDK','INEMO2_GetDataSample',handle_dev,pFD);
	D = getDataSample(s, formatspec);
    t0 = tic;

        %----------
        pause(0.01)
        %---------
        
    % Calibration for accelerometer: 2^14 ~ 1g
    Acc(1,i)=D.ax;
    Acc(2,i)=D.ay;
    Acc(3,i)=D.az;
    Magn(1,i)=D.mx;
    Magn(2,i)=D.my;
    Magn(3,i)=D.mz;
    GyroRate(1,i)=((D.gx-Offset(1,1))/180)*pi;
    GyroRate(2,i)=((D.gy-Offset(2,1))/180)*pi;
    GyroRate(3,i)=((D.gz-Offset(3,1))/180)*pi;
    
    Acc(:,i)=Acc(:,i)/norm(Acc(:,i));
    Magn(:,i)=Magn(:,i)/norm(Magn(:,i));
    if(i<=accF_Length)
        AccF(:,i)=MyFilter(bAcc,aAcc,Acc(:,:));
    else
        AccF(:,i)=MyFilter(bAcc,aAcc,Acc(:,i-accF_Length:i));
    end
    if(i<=magnF_Length)
        MagnF(:,i)=MyFilter(bMagn,aMagn,Magn(:,:));
    else
        MagnF(:,i)=MyFilter(bMagn,aMagn,Magn(:,i-magnF_Length:i));
    end
    MagnF(:,i)=MagnF(:,i)/norm(MagnF(:,i));
    AccF(:,i)=AccF(:,i)/norm(AccF(:,i));
    i=i+1;
    qPredicted(:,i)=[0.5 0.5 0.5 0.5]';
    qUpdate(:,i)=qPredicted(:,i);
    qObserv(:,i)=qPredicted(:,i);
end

%% Do the acquisition
h1 = figure(1); 

subplot(3, 1, 1); sr = line(nan, nan, 'Color', 'g'); 
subplot(3, 1, 2); sp = line(nan, nan, 'Color', 'r');
subplot(3, 1, 3); sy = line(nan, nan, 'Color', 'b'); 

disp('Complete acquisition...')
while(i<=acqSize)
    if(i>2)
        dt = toc(t0);
        t=[t t(length(t))+dt];
    end
    %dt=0.015;
    %----Acquisition
        % [errre pFD]=calllib('iNEMO2_SDK','INEMO2_GetDataSample',handle_dev,pFD);
		D = getDataSample(s, formatspec);
        t0 = tic;

        %----------
        pause(0.01)
        %---------
        
    
    Acc(1,i)=D.ax;
    Acc(2,i)=D.ay;
    Acc(3,i)=D.az;
    Magn(1,i)=D.mx;
    Magn(2,i)=D.my;
    Magn(3,i)=D.mz;
    GyroRate(1,i)=((D.gx-Offset(1,1))/180)*pi;
    GyroRate(2,i)=((D.gy-Offset(2,1))/180)*pi;
    GyroRate(3,i)=((D.gz-Offset(3,1))/180)*pi;
    GyroRate(1,i)=(GyroRate(1,i)+GyroRate(1,i-1))/2;
    GyroRate(2,i)=(GyroRate(2,i)+GyroRate(2,i-1))/2;
    GyroRate(3,i)=(GyroRate(3,i)+GyroRate(3,i-1))/2;
    
    %Normalization and filtering
    Acc(:,i)=Acc(:,i)/norm(Acc(:,i));
    Magn(:,i)=Magn(:,i)/norm(Magn(:,i));
    
    AccF(:,i)=MyFilter(bAcc,aAcc,Acc(:,i-accF_Length:i));
    MagnF(:,i)=MyFilter(bMagn,aMagn,Magn(:,i-magnF_Length:i));
    
    MagnF(:,i)=MagnF(:,i)/norm(MagnF(:,i));
    AccF(:,i)=AccF(:,i)/norm(AccF(:,i));
    %----End Acquisition
    
    %OBSERVATION COMPUTING
    
    %Gradient Descent
    dq(:,i)=0.5*(QuaternionProduct(qUpdate(:,i-1),[0 GyroRate(1,i) GyroRate(2,i) GyroRate(3,i)]'));
    dqnorm(1,i)=norm(dq(:,i));
    mu(1,i)=10*dqnorm(1,i)*dt;
    qObserv(:,i)=GradientDescent(AccF(:,i),MagnF(:,i),qObserv(:,i-1),mu(1,i));
    qObserv(:,i)=qObserv(:,i)/norm(qObserv(:,i));
    %END OSSERVATION COMPUTING
    
    %KALMAN FILTERING
    const=dt/2;
	% SYSTEM DYNAMICS (PHI estimate)
    %F (phi, the system transition matrix) matrix computing
    F1=[1 -const*GyroRate(1,i) -const*GyroRate(2,i) -const*GyroRate(3,i)];
    F2=[const*GyroRate(1,i) 1 const*GyroRate(3,i) -const*GyroRate(2,i)];
    F3=[const*GyroRate(2,i) -const*GyroRate(3,i) 1 const*GyroRate(1,i)];
    F4=[-const*GyroRate(3,i) const*GyroRate(2,i) -const*GyroRate(1,i) 1];
    F=[F1;F2;F3;F4];

	% (1) PREDICTION STEP
	% Predicted state
    qPredicted(:,i)=F*qUpdate(:,i-1);
    
	% Prediction covariance
    Q=Qmatrix;
    P_Predicted=F*P_Update*F'+Q;
    
	% (2) UPDATE STEP
	% Kalman gain
    K=P_Predicted*H'*(H*P_Predicted*H'+R)^-1;
	% Estimated covariance
    P_Update=(eye(4,4)-K*H)*P_Predicted;
	% Estimated state
	% Estimate the quaternion state
    qUpdate(:,i)=qPredicted(:,i)+K*(qObserv(:,i)-H*qPredicted(:,i));
    qUpdate(:,i)=qUpdate(:,i)/norm(qUpdate(:,i));
    Angles(:,i)=GetAnglesFromQuaternion(qUpdate(:,i)); % q->angle
    
    % Plot the angles in real time
    set(sr, 'XData', t, 'YData', Angles(1,1:i));
    set(sp, 'XData', t, 'YData', Angles(2,1:i));
    set(sy, 'XData', t, 'YData', Angles(3,1:i));
    drawnow limitrate
    
    %END KALMAN FILTERING
    i=i+1;
end

%% Plot the results
disp('Cleaning up...')
figure;
    subplot(3,1,1);plot(t,Acc(1,:),'b',t,AccF(1,:),'r',t,Magn(1,:),'g',t,MagnF(1,:),'c');legend('AccX','AccFX','MagnX','MagnFX');grid;
    subplot(3,1,2);plot(t,Acc(2,:),'b',t,AccF(2,:),'r',t,Magn(2,:),'g',t,MagnF(2,:),'c');legend('AcY','AccFY','MagnY','MagnFY');grid;
    subplot(3,1,3);plot(t,Acc(3,:),'b',t,AccF(3,:),'r',t,Magn(3,:),'g',t,MagnF(3,:),'c');legend('AccZ','AccFZ','MagnZ','MagnFZ');grid;


figure;
    subplot(4,1,1);plot(t,qObserv(1,1:acqSize));grid;legend('q0 Observed');
    subplot(4,1,2);plot(t,qObserv(2,1:acqSize));grid;legend('q1 Observed');
    subplot(4,1,3);plot(t,qObserv(3,1:acqSize));grid;legend('q2 Observed');
    subplot(4,1,4);plot(t,qObserv(4,1:acqSize));grid;legend('q3 Observed');

figure;
    subplot(4,1,1);plot(t,qUpdate(1,1:acqSize));hold on;plot(t,qObserv(1,1:acqSize),'r');grid;legend('q0 Estimated','q0 Observed');xlabel('time (sec)');ylabel('Quaternion value');
    subplot(4,1,2);plot(t,qUpdate(2,1:acqSize));hold on;plot(t,qObserv(2,1:acqSize),'r');grid;legend('q1 Estimated','q1 Observed');xlabel('time (sec)');ylabel('Quaternion value');
    subplot(4,1,3);plot(t,qUpdate(3,1:acqSize));hold on;plot(t,qObserv(3,1:acqSize),'r');grid;legend('q2 Estimated','q2 Observed');xlabel('time (sec)');ylabel('Quaternion value');
    subplot(4,1,4);plot(t,qUpdate(4,1:acqSize));hold on;plot(t,qObserv(4,1:acqSize),'r');grid;legend('q3 Estimated','q3 Observed');xlabel('time (sec)');ylabel('Quaternion value');    
    
figure;
    subplot(3,1,1);plot(t,Angles(1,1:acqSize));grid;legend('Roll');xlabel('time (sec)');ylabel('Angle (deg)');
    subplot(3,1,2);plot(t,Angles(2,1:acqSize));grid;legend('Pitch');xlabel('time (sec)');ylabel('Angle (deg)');
    subplot(3,1,3);plot(t,Angles(3,1:acqSize));grid;legend('Yaw');xlabel('time (sec)');ylabel('Angle (deg)');
    
% Clean up the serial port from the workspace
fclose(s); 
delete(s)
clear s
