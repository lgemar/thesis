% =========================================================================
% INITIALIZE TCP and VICON CONNECTION 
% =========================================================================

t = tcpip('0.0.0.0', 30000, 'NetworkRole', 'server');
fopen(t);
vFrame = LCMCoordinateFrame('vicon',JLCMCoder(vicon.ViconLCMCoder()),'v');
vFrame.subscribe('VICON_wand');

% =========================================================================
% READ FROM TCP CONNECTION 
% =========================================================================

STATE = 0; 
READY = 0; 
ACQUIRING = 1; 
SAVING = 2; 

Observations = zeros(1, 24); 
time_stamp = 0; 
file_name = '';
size = 10; 
while(1) 
    if(t.BytesAvailable > size)
        data = fread(t, t.BytesAvailable);
        str = char(data)';
        if( str(1) == 'B' && STATE == READY)
            Observations = zeros(1, 24); 
            time_stamp = sscanf(str, 'B: %s \n'); 
            disp(time_stamp); 
            STATE = ACQUIRING;
        elseif ( str(1) == 'E' && ACQUIRING )
            file_name = sscanf(str, 'E: %s \n'); 
            disp(file_name);
            STATE = SAVING; 
        elseif( STATE == ACQUIRING ) 
            C = strsplit(char(data)', '\t');
            D = str2double(C); 

            y = vFrame.getNextMessage(1000); %// Vicon data
            if isempty(y)
                 y = zeros(7,1);
            end
            
            if( length(D) == 17 )
                Observations = cat(1, Observations, cat(2, D, y')); % note:  cat(2, 3,[4; 5; 6]', [7; 8; 9]') works
            end
        end
        if( STATE == SAVING ) 
            disp('Saving the data'); 
            STATE = READY; 
            csvwrite([file_name, '.csv'], Observations); 
        end
    end
end


%%

% =========================================================================
% READ VICON 
% =========================================================================

% y = vFrame.getNextMessage(1000); // Vicon data
% if isempty(y)
%     y = zeros(7,1);
% end
% Observations = cat(1, Observations, cat(2,tGlobal,data',y')); // note:  cat(2, 3,[4; 5; 6]', [7; 8; 9]') works

