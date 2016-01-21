N = 10000; 

% Open the serial port
com_port = 'COM5'; 
baud_rate = 38400; 
s = serial(com_port, 'BaudRate', baud_rate);
fopen(s); 

% Set the format specification for reading from the serial port
formatspec  = ['a/g/m:\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\n']; 

% Allocate memory for the various data streams
tgxis = zeros(1, N); 
ax = zeros(1, N); ay = zeros(1, N); az = zeros(1, N); 
gx = zeros(1, N); gy = zeros(1, N); gz = zeros(1, N); 
mx = zeros(1, N); my = zeros(1, N); mz = zeros(1, N);

% Open the figures
h1 = figure(1);
subplot(3, 2, 1); smx = line(0, 0, 'Color', 'b'); 
subplot(3, 2, 2); svx = line(0, 0, 'Color', 'r');
subplot(3, 2, 3); smy = line(0, 0, 'Color', 'b');
subplot(3, 2, 4); svy = line(0, 0, 'Color', 'r'); 
subplot(3, 2, 5); smz = line(0, 0, 'Color', 'b');
subplot(3, 2, 6); svz = line(0, 0, 'Color', 'r'); 

% Initialize the 
% The try-catch structure here ensures that the comport doesn't get
% screwed up with errors
try
    start_time = datenum(datetime('now')); 
    idx = 2; 
    while (ishandle(h1))
        data = fscanf(s, formatspec); %do some process
        if( size(data) ~= 9 )
            continue; % If the serial read was bad, try again
        end
        t = 86400 * (datenum(datetime('now')) - start_time);
        tgxis(idx) = t; 
        dt = t - tgxis(idx - 1); 
        
        % Calibration for accelerometer: 2^14 ~ 1g
        ax = data(1) / 2^14; 
        ay = data(2) / 2^14; 
        az = data(3) / 2^14;
        % Calibration for the gyroscope: 2^15 ~ 250 d/s, minus steady-state
        % mean
        gx = (data(4) * (250 / 2^15)) - 1.4085; 
        gy = (data(5) * (250 / 2^15)) - (-1.1659); 
        gz = (data(6) * (250 / 2^15)) - 0.5216; 
        % Calibration for magnetometer: 2^12 ~ 1200 uT
        mx = data(7) * (1200 / 2^12); 
        my = data(8) * (1200 / 2^12); 
        mz = data(9) * (1200 / 2^12); % 

        % Plot the accelerometer data (x)
        tmx = get(smx, 'XData');
        ymx = get(smx, 'YData'); 
        tmx = [tmx t];
        ymx = [ymx mx];
        set(smx, 'XData', tmx, 'YData', ymx);
        
		% Plot the velocity data (x)
        tvx = get(svx, 'XData');
        yvx = get(svx, 'YData'); 
        tvx = [tvx t];
        dv = 0.5*mx*dt; 
        yvx = [yvx (yvx(end)+dv)]; % integrate the acceleration
        set(svx, 'XData', tvx, 'YData', yvx);
        
        % Plot the accelerometer data (y)
        tmy = get(smy, 'XData');
        ymy = get(smy, 'YData');
        tmy = [tmy t];
        ymy = [ymy my];
        set(smy, 'XData', tmy, 'YData', ymy);

		% Plot the velocity data (y)
        tvy = get(svy, 'XData');
        yvy = get(svy, 'YData'); 
        tvy = [tvy t];
        dv = 0.5*my*dt; 
        yvy = [yvy (yvy(end)+dv)]; % integrate the acceleration
        set(svy, 'XData', tvy, 'YData', yvy);

        % plot the accelerometer data (z)
        tmz = get(smz, 'xdata');
        ymz = get(smz, 'YData');
        tmz = [tmz t];
        ymz = [ymz mz];
        set(smz, 'XData', tmz, 'YData', ymz);

		% Plot the velocity data (z)
        tvz = get(svz, 'XData');
        yvz = get(svz, 'YData'); 
        tvz = [tvz t];
        dv = 0.5*mz*dt; 
        yvz = [yvz (yvz(end)+dv)]; % integrate the acceleration
        set(svz, 'XData', tvz, 'YData', yvz);

        % Clean up the loop
        idx = mod((idx + 1), 1000) + 1; 
        
        % Plot the data after every 10 frames to increase speed
		%flush the graphics queue
        drawnow limitrate 
    end

    % Clean up the serial port from the workspace
    fclose(s); 
    delete(s)
    clear s
catch
    % Clean up the serial port from the workspace
    fclose(s); 
    delete(s)
    clear s
end
