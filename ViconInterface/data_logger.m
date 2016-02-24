javaaddpath java %Add LCM stuff to Java classpath
%system('gnome-terminal -e bin/vicon-client'); %start the vicon-lcm bridge

vFrame = ViconCoordinateFrame('yak','y');
vFrame.subscribe();

Nsamp = 182*30;

y = zeros(7,Nsamp);
t = zeros(1,Nsamp);
k = 1;
while k <= Nsamp
    [y(:,k),t(k)] = vFrame.getNextMessage(10);
    k = k+1;
end
