package vicon;

import java.io.*;
import java.lang.*;
import java.util.Arrays;
import lcm.lcm.*;

public class ViconLCMCoder implements drake.util.LCMCoder
{
    public ViconLCMCoder() {}
    
    public drake.util.CoordinateFrameData decode(byte[] data)
    {
      try {
        body_t msg = new body_t(data);
        return decode(msg);
      } catch (IOException ex) {
        System.out.println("Exception: " + ex);
      }
      return null;
    }
 
    public int dim() {
      return 7;
    }

    public drake.util.CoordinateFrameData decode(body_t msg)
    {
      drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();

      fdata.t = (double)msg.utime/1000000.0; //What's going on here?

      fdata.val = new double[7];

      fdata.val[0] = msg.trans[0];
      fdata.val[1] = msg.trans[1];
      fdata.val[2] = msg.trans[2];

      fdata.val[3] = msg.quat[0];
      fdata.val[4] = msg.quat[1];
      fdata.val[5] = msg.quat[2];
      fdata.val[6] = msg.quat[3];

      return fdata;
    }

    public LCMEncodable encode(drake.util.CoordinateFrameData d)
    {
      body_t msg = new body_t();

      msg.utime = (long)(d.t*1000000); //What's going on here?

      msg.trans[0] = d.val[0];
      msg.trans[1] = d.val[1];
      msg.trans[2] = d.val[2];

      msg.quat[0] = d.val[3];
      msg.quat[1] = d.val[4];
      msg.quat[2] = d.val[5];
      msg.quat[3] = d.val[6];

      return msg;
    }
    
    public String timestampName()
    {
      return "utime";
    }
    
    public String[] coordinateNames() {
      String[] coords = new String[dim()];
      coords[0] = "x1";
      coords[1] = "x2";
      coords[2] = "x3";
      coords[3] = "q1";
      coords[4] = "q2";
      coords[5] = "q3";
      coords[6] = "q4";
      return coords;
    }
}
