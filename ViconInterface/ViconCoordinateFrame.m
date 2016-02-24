classdef ViconCoordinateFrame < CoordinateFrame & LCMSubscriber
    %VICONCOORDINATEFRAME Provides an interface to a Vicon motion capture
    %system through LCM
    
    properties
        channel;
    end
    
    properties (SetAccess = private, Hidden = true)
        pointer; %Pointer to the C++ class instance
    end
    
    methods
        function obj = ViconCoordinateFrame(name, prefix)
            
            typecheck(name,'char');
            typecheck(prefix,'char');
            sizecheck(prefix,1);
            
            obj = obj@CoordinateFrame(name,7,prefix,{'x1','x2','x3','q0','q1','q2','q3'});
            obj.channel = strcat('VICON_',name);
            
            %Call the C++ constructor
            obj.pointer = vicon_lcm_mex('new');
        end
        
        function delete(obj)
            %Call the C++ destructor
            %This is causing MATLAB to crash for some reason...
            %vicon_lcm_mex(obj.pointer, 'delete'); 
        end
        
        function obj = subscribe(obj,channel)
            if(nargin == 2)
                obj.channel = channel;
            end
            vicon_lcm_mex(obj.pointer, 'subscribe', obj.channel)
        end
        
        function [x,t] = getNextMessage(obj,timeout)
            [x,t] = vicon_lcm_mex(obj.pointer, 'getNextMessage', timeout);
        end
        
        function [x,t] = getCurrentValue(obj)
            [x,t] = vicon_lcm_mex(obj.pointer, 'getCurrentValue');
        end
        
        function t = getLastTimestamp(obj)
            t = vicon_lcm_mex(obj.pointer, 'getLastTimestamp');
        end
        
        function str = defaultChannel(obj)
            str = obj.channel;
        end
        
    end
    
end

