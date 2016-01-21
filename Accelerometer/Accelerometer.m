classdef Accelerometer
   properties
      % Basic properties of the connection with the Accelerometer
      BaudRate
      ComPort
      SerialPort
      FormatString 
      
      % Biases of the device
      AccelBias
      GyroBias
   end
   methods
      function A = Accelerometer(com_port)
         A.BaudRate = 250000;
         A.ComPort = com_port; 
         A.SerialPort = serial(A.ComPort, 'BaudRate', A.BaudRate);
         A.FormatString=['a/g/m:\t','%d\t','%d\t','%d\t','%d\t','%d\t','%d\n'];
	 fopen(A.SerialPort); 
      end

      function A = calibrate(A)
        % Define the calibration size
        acqSize = 100; % @~50 samples/s

        % Allocate variables for the calibration
        i = 1;
        GyroRate=zeros(3,acqSize);
        Acc=zeros(3,acqSize);
        Magn=zeros(3,acqSize);
        t = zeros(1, acqSize);

        % Start the acquisition clock
        t0 = tic;
        while(i<=acqSize)
            if(i>1)
                t(i)=toc(t0);
            end

            % Make sure to read a valid data sample with all 9 fields
            data = fscanf(A.SerialPort, A.FormatString);
            while( size(data) ~= 6 )
                    data = fscanf(A.SerialPort, A.FormatString);
            end

            % Adjust the data according to the sensitivity of the sensor
            accel = (4*9.81) * data(1:3) / (2^16);
            gyro = (2*250) * data(4:6) / (2^16);  
            % magnet = (2*1200) * data(7:9) / (2^16);

            % Store for later
            Acc(1:3,i) = accel';
            GyroRate(1:3,i) = gyro';
            % Magn(1:3,i) = magnet';
            
            i=i+1;
        end

        % Set the biases on the data
        A.AccelBias = mean(Acc, 2);
        A.GyroBias = mean(GyroRate, 2); 
      end

      function s = getDataSample(A)
        % Allocate the output
        s = zeros(1, 6);
        formatspec  = ['a/g/m:\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\t' '%d\n']; 

        % Make sure to read a valid data sample with all 9 fields
        data = fscanf(A.SerialPort, A.FormatString);
        while( size(data) ~= 6 )
                data = fscanf(A.SerialPort, A.FormatString);
        end

        % Read in and adjust the accelerometer readings
        accel = (4*9.81) * data(1:3) / (2^16);
        accel_calib = -A.AccelBias + accel;

        % Read in and adjust the gyroscope data
        gyro = (2*250) * data(4:6) / (2^16);  
        gyro_calib = gyro - A.GyroBias;

        % Read in and adjust the magnetometer readings
        % magn = (2*1200) * data(7:9) / (2^16);
        % magn_calib = magn;

        s(1:3) = accel_calib; 
        s(4:6) = gyro_calib; 
        % s(7:9) = magn_calib;
      end

      function close(A)
         fclose(A.SerialPort);         
      end

      function delete(A)
        delete(A.SerialPort);
        clear A.SerialPort;
      end
   end
end
