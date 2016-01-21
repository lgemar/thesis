function [accelerometer, flag] = setupSerial(ComPort)
    flag = 1;
    accelerometer.s = serial(ComPort);
    set(accelerometer.s,'DataBits',8 );
    set(accelerometer.s,'StopBits',1 );
    set(accelerometer.s,'BaudRate', 57600);
    set(accelerometer.s,'Parity', 'none' );
    fopen(accelerometer.s);
    
    fprintf(accelerometer.s,'%c','a')
     
    mbox = msgbox('Serial Communication setup'); uiwait(mbox);
    fscanf(accelerometer.s,'%u');
end