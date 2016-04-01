using UnityEngine;
using System.Collections;
using System;
using System.Runtime.Serialization.Formatters.Binary;
using System.IO;
using System.Text;

using OpenCVForUnity;
using System.Collections.Generic;

using System.Net.Sockets;

// Singleton design pattern: there will only be one game object at a given time
// Based off of https://unity3d.com/learn/tutorials/modules/beginner/live-training-archive/persistence-data-saving-loading
public class AppControl : MonoBehaviour {

    public string DataPath; 
    public string FileName;
    public bool acquiring;
    public float fps;

    // Breadboard data variables
    public string[] BbStringData;
    float[] BbFloatData;

    // Target variables
    public OpenCVForUnity.Rect TargetBox = new OpenCVForUnity.Rect(0,0,0,0); 

    // There is one of these objects in the scenes
    public static AppControl control;


    // Camera calibration parameters
    public bool calibrationComplete = false;
    public Mat cameraMatrix;
    public Mat distCoeffs;
    public float fx;
    public float fy;
    public float cx;
    public float cy;
    public float reprojectionError;

    // virtual environment control
    public bool virtualEnvironmentEnabled;

    // Networking variables
    TcpClient DataClient = new TcpClient();
    NetworkStream DataStream;

    // COM variables
    ComPort BbDataSource; // bb = breadboard
    public string comValue;
    public int BaudRate;
    public int BbN = 5;  // Number of data elements per line


    // Sample rate counter
    float updateInterval = 0.5F;
    double lastInterval;
    int frames = 0;

    // Stream writer test file
    StreamWriter writetext;

    // Observations
    public double[] camObservations;
    public double[] inertialObservations;

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
            // Open COM communication with breadboard 
            control.LoadCameraParameters();

            // Connect to data sink 
            // ConnectMatlab();

            // Load the camera calibration parameters
//            BbDataSource = new ComPort(comValue, BaudRate);
            BbDataSource.Open();
            BbStringData = new string[BbN];
            BbFloatData = new float[BbN];
        }
    }

    void ConnectMatlab()
    {
        DataClient = new TcpClient();
        if (DataClient != null)
        {
            try { DataClient.Connect("localhost", 30000); }
            catch (SocketException e)
            {
                Debug.Log("Error opening Socket = " + e.Message);
            }
        }
    }

    void LoadCameraParameters()
    {
        int Calibrated = PlayerPrefs.GetInt("Calibrated");
        if (Calibrated == 1)
        {
            control.reprojectionError = PlayerPrefs.GetFloat("Error");
            control.fx = PlayerPrefs.GetFloat("fx");
            control.fy = PlayerPrefs.GetFloat("fy");
            control.cx = PlayerPrefs.GetFloat("cx");
            control.cy = PlayerPrefs.GetFloat("cy");

            float k1 = PlayerPrefs.GetFloat("k1");
            float k2 = PlayerPrefs.GetFloat("k2");
            float p1 = PlayerPrefs.GetFloat("p1");
            float p2 = PlayerPrefs.GetFloat("p2");
            float k3 = PlayerPrefs.GetFloat("k3");

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

            control.calibrationComplete = true;
        }

    }

    void Update()
    {
        // Read data from the breadboard
//        BbDataSource.ReadData(ref BbStringData, ref BbFloatData, BbN);
        // BbDataSource.ReadLine(ref BbStringData, ref BbFloatData, BbN);

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
        /*
        if( DataClient.Connected )
        {
            // DataStream = DataClient.GetStream();
            // byte[] myWriteBuffer = System.Text.Encoding.ASCII.GetBytes("B: " + Time.realtimeSinceStartup + "\n");
            // DataStream.Write(myWriteBuffer, 0, myWriteBuffer.Length);
        }
            Debug.Log("No connection has been made with Matlab service. Trying again...");
            ConnectMatlab();
            if (DataClient.Connected)
            {
                Debug.Log("Success!");
                // beginTest();
            }
            else
            {
                // acquiring = false;
            }
        */
        string extension = ".csv";
        writetext = new StreamWriter(DataPath + FileName + extension );
    } 

    public void endTest()
    {
        /*
        byte[] myWriteBuffer = System.Text.Encoding.ASCII.GetBytes("E: " + FileName + "\n");
        DataStream.Write(myWriteBuffer, 0, myWriteBuffer.Length);
        DataStream.Flush();
        if( DataClient.Connected )
        {
            // DataStream = DataClient.GetStream();
        }
        */
        writetext.Close();
        Debug.Log("Test ended");
    } 

    public void writeData()
    {
        if( writetext != null )
        {
            float t = Time.realtimeSinceStartup;
            string str = t.ToString() + ",";
            str += TargetBox.x.ToString() + ",";
            str += TargetBox.y.ToString() + ",";
            str += TargetBox.width.ToString() + ",";
            str += TargetBox.height.ToString() + ",";

            float t_sensor = BbFloatData[0] / 1000f; 
            str += t_sensor.ToString();
            for(int i = 1; i < BbStringData.Length; i++)
            {
                str += ",";
                str += BbStringData[i];
            }
            writetext.WriteLine(str);
            Debug.Log(str);
        }
        /*
        if( DataClient.Connected )
        {
            DataStream = DataClient.GetStream();

            float t = Time.realtimeSinceStartup;
            string str = t.ToString() + "\t";
            str += TargetBox.x.ToString() + "\t";
            str += TargetBox.y.ToString() + "\t";
            str += TargetBox.width.ToString() + "\t";
            str += TargetBox.height.ToString() + "\t";

            float t_sensor = BbFloatData[0] / 1000f; 
            str += t_sensor.ToString();
            for(int i = 1; i < BbStringData.Length; i++)
            {
                str += "\t";
                str += BbStringData[i];
            }
            str += "\n";

            /*
            // create a byte array and copy the floats into it...
            byte[] myWriteBuffer2 = System.Text.Encoding.ASCII.GetBytes(str);
            DataStream.Write(myWriteBuffer2, 0, myWriteBuffer2.Length);
            DataStream.Flush();
        }
        */

    } 

	// Update is called once per frame
	void OnGUI () {
	}
}

[Serializable]
class AppData
{
    // Test variables
    public float health;
    public float experience;

    // Camera calibration parameters
    public bool calibrationComplete;
    public Mat cameraMatrix;
    public Mat distCoeffs;
    public float reprojectionError;
}
