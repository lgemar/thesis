% javaaddpath java %Add LCM stuff to Java classpath
%system('gnome-terminal -e bin/vicon-client'); %start the vicon-lcm bridge

% --- Use MATLAB LCM Coder --- %
%vFrame = LCMCoordinateFrame('vicon',ViconLCMCoder(),'v'); %what is the prefix for?

% --- Use Java LCM Coder --- %
vFrame = LCMCoordinateFrame('vicon',JLCMCoder(vicon.ViconLCMCoder()),'v');
vFrame.subscribe('VICON_wand');

while true
y = vFrame.getNextMessage(1000)
end