using UnityEngine;
using System.Collections;
using System;
using System.Runtime.Serialization.Formatters.Binary;
using System.IO;
using System.Text;
using System.Collections.Generic;

using OpenCVForUnity;

using System.Net.Sockets;

// Singleton design pattern: there will only be one game object at a given time
// Based off of https://unity3d.com/learn/tutorials/modules/beginner/live-training-archive/persistence-data-saving-loading
public class DemoController : MonoBehaviour {

    // Self reference
    DemoController control; 

    public string DataPath; 
    public string FileName;
    public bool acquiring;
    public float fps;

    // Intrinsic matrix properties
    public float fx;
    public float fy;
    public float cx;
    public float cy;

    // Distortion parameters
    public float k1; 
    public float k2; 
    public float p1; 
    public float p2; 
    public float k3; 
    
    // Reprojection error
    public float reprojectionError;

    Mat cameraMatrix;
    Mat distCoeffs;

    // Sample rate counter
    float updateInterval = 0.5F;
    double lastInterval;
    int frames = 0;

    // Stream writer test file
    StreamWriter writetext;


    // Use this for initialization
    void Awake () {

        // ensure that there is only one instance of this object
        if(control == null)
        {
            DontDestroyOnLoad(gameObject);
            control = this;
        }
        else if(control != this)
        {
            Destroy(gameObject);
        }

	}

    void Start() {
        if ( this == control )
        {
            // Build distortion coefficient matrix
            control.distCoeffs = new Mat(1, 5, CvType.CV_64FC1);
            control.distCoeffs.put(0, 0, k1);
            control.distCoeffs.put(0, 1, k2);
            control.distCoeffs.put(0, 2, p1);
            control.distCoeffs.put(0, 3, p2);
            control.distCoeffs.put(0, 4, k3);

            // Build camera calibration matrix
            control.cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
            control.cameraMatrix.put(0, 0, fx);
            control.cameraMatrix.put(1, 1, fy);
            control.cameraMatrix.put(0, 2, cx);
            control.cameraMatrix.put(1, 2, cy);
            control.cameraMatrix.put(2, 2, 1f);
        }
    }

    void Update()
    {

        if (acquiring)
            writeData();

        // Sample rate update
        ++frames;
        float timeNow = Time.realtimeSinceStartup;
        if (timeNow > lastInterval + updateInterval)
        {
            fps = (float)(frames / (timeNow - lastInterval));
            frames = 0;
            lastInterval = timeNow;
        }
    }


    public void beginTest()
    {
        Debug.Log("Begin test");
        string extension = ".csv";
        writetext = new StreamWriter(DataPath + FileName + extension );
    } 

    public void endTest()
    {
        writetext.Close();
        Debug.Log("Test ended");
    } 

    public void writeData()
    {
        if( writetext != null )
        {
            float t = Time.realtimeSinceStartup;
            string str = t.ToString() + ",";
            writetext.WriteLine(str);
            Debug.Log(str);
        }
    } 

	// Update is called once per frame
	void OnGUI () {
	}
}

