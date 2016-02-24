function [t, y] = measure_delay(s)

addpath('../SerialInterface');

if nargin == 0
    s = '/dev/cu.usbmodem1411';
end

javaaddpath java %Add LCM stuff to Java classpath
%system('gnome-terminal -e bin/vicon-client'); %start the vicon-lcm bridge

vFrame = ViconCoordinateFrame('vicon','v');

vFrame.subscribe('VICON_extra_tail');

Nsamp = 600; %3 seconds
y = zeros(7,Nsamp);
t = zeros(1,Nsamp);

send_command_mex(s,0,106,106,106);

[y(:,1),t(1)] = vFrame.getNextMessage(10);
k = 2;
while k <= Nsamp
    [y(:,k), t(k)] = vFrame.getNextMessage(10);
    if k == 200
        send_command_mex(s,0,106,106,215);
    end
    k = k+1;
end

t = (t - t(1)); %time in milliseconds starting at zero
send_command_mex(s,0,106,106,106); %set controls back to neutral position

end

