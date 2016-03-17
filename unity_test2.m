% =========================================================================
% INITIALIZE TCP and VICON CONNECTION 
% =========================================================================

M = 7; % number of data elements from VICON

vFrame = LCMCoordinateFrame('vicon',JLCMCoder(vicon.ViconLCMCoder()),'v');
vFrame.subscribe('VICON_wand2');

% =========================================================================
% READ FROM TCP CONNECTION 
% =========================================================================

disp('Connected made. Starting acquisition'); 
pause(0.1); 

%%
Observations = zeros(1, M+1); 
time_stamp = 0; 
file_name = 'veryfasthorz_vicon';
size = 10; 
while(1) 
    y = vFrame.getNextMessage(1000);
    t = vFrame.getLastTimestamp();
    if( length(y) == M )
        Observations = cat(1, Observations, [t y']);
    end
end
%% 

disp('Saving the data');   
STATE = READY;      
csvwrite(['C:\Users\Lukas Gemar\thesis\Analysis\Data-3-16\', file_name, '.csv'], Observations); 


