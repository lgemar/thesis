classdef ViconLCMCoder < LCMCoder
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods
        
        function d = dim(obj)
            d = 7;
        end
        
        function str = timestampName(obj)
            str = 'utime';
        end
        
        function names = coordinateNames(obj)
            names = {'x1' 'x2' 'x3' 'q1' 'q2' 'q3' 'q4'};
        end
        
        function [x,t] = decode(obj,data)
            msg = vicon.body_t(data);
            t = msg.utime;
            x = zeros(7,1);
            x(1) = msg.trans(1);
            x(2) = msg.trans(2);
            x(3) = msg.trans(3);
            x(4) = msg.quat(1);
            x(5) = msg.quat(2);
            x(6) = msg.quat(3);
            x(7) = msg.quat(4);
        end
        
        function msg = encode(obj,t,x)
            msg = vicon.body_t();
            msg.utime = t;
            msg.trans = [x(1); x(2); x(3)];
            msg.quat = [x(4); x(5); x(6); x(7)];
        end
    end
    
end

