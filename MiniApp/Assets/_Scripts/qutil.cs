using UnityEngine;
using System.Collections;
using OpenCVForUnity; 

public static class qutil {


    public static Quaternion qflip(Quaternion2 q)
    {
        Quaternion qout = new Quaternion(q.x, q.y, q.z, q.w);
        return qout; 

    }

    public static Quaternion2 qflip(Quaternion q)
    {
        Quaternion2 qout = new Quaternion2(q.x, q.y, q.z, q.w);
        return qout; 
    }

}
