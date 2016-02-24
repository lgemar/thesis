javaaddpath java %Add LCM stuff to Java classpath
system('gnome-terminal -e bin/vicon-client'); %start the vicon-lcm bridge

vFrame = LCMCoordinateFrame('vicon',JLCMCoder(vicon.ViconLCMCoder()),'v'); %Use Java LCMCoder

vFrame.subscribe('VICON_plane');

Nsamp = 60*100;

y = zeros(7,Nsamp);
k = 1;
tlast = vFrame.getLastTimestamp();
while k <= Nsamp
tnow = vFrame.getLastTimestamp();
if tnow - tlast >= 0.01
    y(:,k) = vFrame.getCurrentValue();
    tlast = tnow;
    k = k+1;
end
end

yvar = var(y,0,2)
