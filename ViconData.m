% =========================================================================
%% INITIALIZE VICON 
% =========================================================================
% temp = pathdef
% 
% addpath(temp); 
% 
% addpath('C:\Users\Lukas Gemar\thesis\'); 
% addpath('C:\Users\Lukas Gemar\thesis\ViconInterface')
% addpath('C:\Users\Lukas Gemar\thesis\ViconInterface\java')
% javaaddpath('C:\Users\Lukas Gemar\thesis\ViconInterface')
% javaaddpath('C:\Users\Lukas Gemar\thesis\ViconInterface\java')
% javaaddpath('C:\Users\Lukas Gemar\thesis\ViconInterface\java\vicon')
% 
% savepath 'C:\Users\Lukas Gemar\thesis\pathdef.m'

% =========================================================================
%% INITIALIZE TCP and VICON CONNECTION 
% =========================================================================

M = 7; % number of data elements from VICON

vFrame = LCMCoordinateFrame('vicon',JLCMCoder(vicon.ViconLCMCoder()),'v');
vFrame.subscribe('VICON_computer');

% =========================================================================
% READ FROM TCP CONNECTION 
% =========================================================================

disp('Connected made. Starting acquisition'); 
pause(0.1); 

%%
Observations = zeros(1, M+1); 
time_stamp = 0; 
file_name = 'Computer';
size = 10; 
while(1) 
    y = vFrame.getNextMessage(1000);
    t = vFrame.getLastTimestamp() / (10^6);
    if( length(y) == M )
        Observations = cat(1, Observations, [t y']);
    end
end
%% 

disp('Saving the data');     
% csvwrite(['C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-17\Data\', file_name, '_vicon.csv'], Observations); 
dlmwrite(['C:\Users\Lukas Gemar\thesis\Analysis\Analysis-3-17\Data\', file_name, '_vicon.csv'], Observations, 'delimiter', ',', 'precision', 16); 


