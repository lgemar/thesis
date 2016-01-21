classdef Kinect
   properties
      % Basic properties of the connection with the Kinect
      colorVid
      depthVid
   end
   methods
      function K = Kinect()
        K.colorVid = videoinput('kinect',1); 
        K.depthVid = videoinput('kinect',2);
      end

      function startAcq(K, NUM_FRAMES)
        % Set the acquisition properties
        K.colorVid.FramesPerTrigger = NUM_FRAMES;
        K.depthVid.FramesPerTrigger = NUM_FRAMES;

        % Configure the trigger for recording purposes
        triggerconfig([K.colorVid K.depthVid],'manual');

        % Start the color and depth device. 
        start([K.colorVid K.depthVid]);

        % Trigger the devices to start logging of data.
        trigger([K.colorVid K.depthVid]);
      end

      function [rgb_frames, depth_frames] = getFrames(K, NUM_FRAMES)
        while(K.colorVid.FramesAcquired ~= NUM_FRAMES || K.depthVid.FramesAcquired ~= NUM_FRAMES)
            pause(.1); 
        end
        [rgb_frames, ts1, rgb_metaData] = getdata(K.colorVid, NUM_FRAMES);
        [depth_frames, ts2, depth_metaData] = getdata(K.depthVid, NUM_FRAMES);
      end

      function stopAcq(K)
        stop([K.colorVid K.depthVid]);
      end
   end
end
