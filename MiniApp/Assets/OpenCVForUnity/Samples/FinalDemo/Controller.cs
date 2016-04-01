using UnityEngine;
using System.Runtime.Serialization.Formatters.Binary;
using System.IO;
using System.Text;
using System.Collections.Generic;

using Matrix = MathNet.Numerics.LinearAlgebra.Matrix<float>;
using Vector = MathNet.Numerics.LinearAlgebra.Vector<float>;

using System.Threading;


using OpenCVForUnity;

using System.Net.Sockets;

// Singleton design pattern: there will only be one game object at a given time
// Based off of https://unity3d.com/learn/tutorials/modules/beginner/live-training-archive/persistence-data-saving-loading
public class Controller : MonoBehaviour {

    // Position and orientation scripts
    public GameObject ThreeDTool;
    public UnityEngine.Camera Cam;
    public GameObject Quad;
    public Position PositionTracker;
    public Orientation OrientationTracker;

    // Data containers
    public bool ret; // thread return value, true if data has been read
    public float[] floatData;
    string[] stringData;

    // COM Location and serial port object, num elements
    public string comValue;
    public int BaudRate;
    ComPort DataSource;
    public int N = 5;

    // Threading variables
    Thread DataThread;
    bool threadStop = false;

    // Self reference
    Controller control; 

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

    // Virtual reality mode
    bool VrMode = false;

    // Use this for initialization
    void Awake () {

        // ensure that there is only one instance of this object
        if(control == null)
        {
            DontDestroyOnLoad(gameObject);
            control = this;
            this.control = this;
        }
        else if(control != this)
        {
            Destroy(gameObject);
        }

	}

    void Start() {
        if ( this == control )
        {
            stringData = new string[N];
            floatData = new float[N];

            ///////////////////////////////////////////////////////////////////
            // COMPORT INITIALIZATION
            ///////////////////////////////////////////////////////////////////
            DataSource = new ComPort(comValue, BaudRate);
            DataSource.Open();

            DataThread = new Thread(new ThreadStart(ThreadFun));
            DataThread.Start();
            while (!DataThread.IsAlive);

            ///////////////////////////////////////////////////////////////////
            // Cam Calibration Matrix
            ///////////////////////////////////////////////////////////////////
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

    void ThreadFun()
    {
        while(!threadStop)
        {
            try
            {
                ret = DataSource.ReadData(ref stringData, ref floatData, N);
                Debug.Log("Thread is having fun");
            }
            catch {  }
        }
    }


    void Update()
    {

        if (acquiring)
            writeData();

        ///////////////////////////////////////////////////////////////////////
        // Update the tool POSITION 
        ///////////////////////////////////////////////////////////////////////

        Vector pr = PositionTracker.pr.Clone();

        Vector pd = Vector.Build.DenseOfArray(new float[] { 0, 0, 400 });
        Matrix Ad = Matrix.Build.DenseOfArray(new float[,] { { -1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } });
        Vector prpr = pd - Ad * pr;

        Vector3 prprUnity = new Vector3(prpr[0], prpr[1], prpr[2]);
        prprUnity = 0.001f * prprUnity;
        ThreeDTool.transform.position = Cam.transform.position + prprUnity;

        ///////////////////////////////////////////////////////////////////////
        // Update the tool ORIENTATION 
        ///////////////////////////////////////////////////////////////////////

        Vector qt = OrientationTracker.qt.Clone(); // tool orientation, from MEKF
        Matrix As = OrientationTracker.AMat(qt); // sensor attitude

        Matrix sRb = Matrix.Build.DenseOfArray(new float[,] { { 0, 0, -1 }, { 0, 1, 0 }, { 1, 0, 0 } }); // body to sensor rotation matrix
        Matrix wRv = Matrix.Build.DenseOfArray(new float[,] { { 1, 0, 0 }, { 0, 0, -1 }, { 0, -1, 0 } }); // rotation from virtual to world coords
        Vector qs = OrientationTracker.rotm2quat(sRb * As * wRv);

        // Graphics update
        Quaternion qinv = new Quaternion(qs[0], -qs[1], qs[2], qs[3]); // body to world
        ThreeDTool.transform.rotation = qinv;
        ThreeDTool.transform.rotation = Quaternion.Euler(0, OrientationTracker.AlignYaw, 0) * ThreeDTool.transform.rotation;

        ///////////////////////////////////////////////////////////////////////
        // Update the sample rate counter and clock
        ///////////////////////////////////////////////////////////////////////
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
            // Write the current time in the first column
            float t = Time.realtimeSinceStartup;
            string str = t.ToString() + ",";

            float pux = PositionTracker.pux;
            float puy = PositionTracker.puy;
            float pus = PositionTracker.pus;

            str += pux.ToString() + ",";
            str += puy.ToString() + ",";
            str += pus.ToString() + ",";
            str += pus.ToString() + ",";

            float ts = floatData[0] / 1000f;  // sensor time clock
            str += ts.ToString();
            for(int i = 1; i < floatData.Length; i++)
            {
                str += ",";
                str += floatData[i];
            }
            writetext.WriteLine(str);
            Debug.Log(str);
        }
    } 

    void OnGUI()
    {
        int buttonWidth = 200;
        int buttonHeight = 30;
        int padding = 5;

        if(GUI.Button(new UnityEngine.Rect(Screen.width - buttonWidth - padding, buttonHeight + padding, buttonWidth, buttonHeight), "Enter Virtual Environment")) {
            VrMode = true;
            turnOnVE();
        }
        if(GUI.Button(new UnityEngine.Rect(Screen.width - buttonWidth - padding, 2 * buttonHeight + 2 * padding, buttonWidth, buttonHeight), "Exit Virtual Environment")) {
            VrMode = false;
            turnOffVE();
        }
        if(GUI.Button(new UnityEngine.Rect(Screen.width - buttonWidth - padding, 3 * buttonHeight + 3 * padding, buttonWidth, buttonHeight), "Start Test")) {
            beginTest();
            acquiring = true;
        }
        if(GUI.Button(new UnityEngine.Rect(Screen.width - buttonWidth - padding, 4 * buttonHeight + 4 * padding, buttonWidth, buttonHeight), "End Test")) {
            acquiring = false;
            endTest();
        }
    }

    void turnOnVE()
    {
        Quad.transform.position = new Vector3(-800, 300, 0);
        Cam.transform.position = new Vector3(0, 1, -800);
        Cam.fieldOfView = 60;
    }

    void turnOffVE()
    {
        Quad.transform.position = new Vector3(0, 0, 0);
        Cam.transform.position = new Vector3(0, 1, -400);
        Cam.fieldOfView = 60;
    }

    void OnDestroy()
    {
        DataSource.Close();
        threadStop = true;
        DataThread.Abort();
        DataThread.Join();
    }

}

