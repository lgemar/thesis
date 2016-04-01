using UnityEngine;
using System.Collections;
using System.IO.Ports;
using System.Threading;
using Matrix = MathNet.Numerics.LinearAlgebra.Matrix<float>;
using Vector = MathNet.Numerics.LinearAlgebra.Vector<float>;

public class Utility : MonoBehaviour
{

    public SerialPort sp = new SerialPort("COM2", 9600);
    public Thread serialThread;

    public float updateInterval = 0.5F;
    private double lastInterval;
    private int frames = 0;
    public float fps;

    public Quaternion qrigid;
    public Vector3 accel;
    public Vector3 gyro;
    public Vector3 magn;


    GameObject mainCamera;

    public float stick_length = 160f;

    public Vector3 SensorAlignmentAngles = new Vector3(0, 0, 0);
    Quaternion SensorAlignment;

    public Quaternion btranslate;
    public Quaternion wtranslate;
    public Quaternion ctranslate;

    public Vector3 startingPosition = new Vector3(0, 0, 0);
    public Vector3 stickTranslation;

    // Use this for initialization
    void Start()
    {
        openPort();
        mainCamera = GameObject.FindWithTag("MainCamera");
        // connect();
        AppControl.control.inertialObservations = new double[] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    }


    void parseValues(string av)
    {
        string[] split = av.Split(',');

        // http://stackoverflow.com/questions/28673777/convert-quaternion-from-right-handed-to-left-handed-coordinate-system
        if (split.Length == 14)
        {
            Debug.Log("REad full data frame");
            qrigid.w = float.Parse(split[1]);
            qrigid.x = float.Parse(split[2]);
            qrigid.y = float.Parse(split[3]);
            qrigid.z = float.Parse(split[4]);

            accel.x = float.Parse(split[5]);
            accel.y = float.Parse(split[6]);
            accel.z = float.Parse(split[7]);

            gyro.x = float.Parse(split[8]);
            gyro.y = float.Parse(split[9]);
            gyro.z = float.Parse(split[10]);

            magn.x = float.Parse(split[11]);
            magn.y = float.Parse(split[12]);
            magn.z = float.Parse(split[13]);

            SensorAlignment = Quaternion.Euler(SensorAlignmentAngles.z, SensorAlignmentAngles.y, SensorAlignmentAngles.x);
            qrigid = Quaternion.Inverse(qrigid) * SensorAlignment;

            btranslate = new Quaternion(0, 0.001f * stick_length, 0, 0);
            wtranslate = qrigid * btranslate * Quaternion.Inverse(qrigid);
            ctranslate = mainCamera.transform.rotation * wtranslate * Quaternion.Inverse(mainCamera.transform.rotation);
            stickTranslation = new Vector3(ctranslate.x, ctranslate.y, ctranslate.z);
            gameObject.transform.position = startingPosition + stickTranslation;

        }
    }

    // Taken from http://planning.cs.uiuc.edu/node153.html
    void rotm2quat(Matrix A, ref Quaternion target)
    {
        target.w = 0.5f * Mathf.Sqrt(A[0, 0] + A[1, 1] + A[2, 2] + 1);
        if (AlmostEquals(target.w, 0f, 0.00001f))
        {
            float denom = Mathf.Sqrt(Mathf.Pow(A[0, 1], 2) * Mathf.Pow(A[0, 2], 2) + Mathf.Pow(A[0, 1], 2) * Mathf.Pow(A[1, 2], 2) + Mathf.Pow(A[0, 2], 2) * Mathf.Pow(A[1, 2], 2));
            target.x = A[0, 2] * A[0, 1] / denom;
            target.y = A[0, 1] * A[1, 2] / denom;
            target.z = A[0, 2] * A[1, 2] / denom;
        }
        else
        {
            target.x = (A[2, 1] - A[1, 2]) / (4 * target.w);
            target.y = (A[0, 2] - A[2, 0]) / (4 * target.w);
            target.z = (A[1, 0] - A[0, 1]) / (4 * target.w);
        }
    }

    public static bool AlmostEquals(float double1, float double2, float precision)
    {
        return (Mathf.Abs(double1 - double2) <= precision);
    }

    void crossProduct(Vector x, Vector y, ref Vector target)
    {
        float[] floatarray = new float[] { x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0] };
        target = Vector.Build.DenseOfArray(floatarray);
    }

    /*
    void moveObj(float x, float y, float z)
    {
        int speed = 10;
        Vector3 move = Vector3.zero;
        move.y = y;
        move.Normalize();
        move.y = Mathf.Lerp(move.y, prevY, speed * Time.deltaTime);
        transform.Translate(Vector3.up * (move.y * speed) * Time.deltaTime, Space.World);
        readyToMove = false;
        prevY = move.y;
    }
    */

    void connect()
    {
        Debug.Log("Connection started");
        try
        {
            sp.Open();
            sp.ReadTimeout = 100;
            sp.Handshake = Handshake.None;
            serialThread = new Thread(recData);
            serialThread.Start();
            Debug.Log("Port Opened!");
        }
        catch (System.SystemException e)
        {
            Debug.Log("Error opening = " + e.Message);
        }
    }

    void openPort()
    {
        try
        {
            sp.Open();
            sp.ReadTimeout = 100;
            sp.Handshake = Handshake.None;
            Debug.Log("Port Opened!");
        }
        catch (System.SystemException e)
        {
            Debug.Log("Error opening = " + e.Message);
        }
    }

    void recData()
    {
        Debug.Log("recData is running...");
        Debug.Log(sp == null);

        if ((sp != null) && (sp.IsOpen))
        {
            byte tmp;
            string data = "";
            string avalues = "";

            tmp = (byte)sp.ReadByte();

            while (tmp != 255)
            {
                Debug.Log(data.Length);
                data += (char)(tmp);
                tmp = (byte)sp.ReadByte();
                if ((tmp == '\n') && (data.Length > 10))
                {
                    Debug.Log("parsing...");
                    avalues = data;
                    parseValues(data);
                    data = "";
                    Debug.Log("done parsing...");
                }
            }
        }
    }

    void Update()
    {
        gameObject.transform.rotation = qrigid;
        if ((sp != null) && (sp.IsOpen))
        {
            byte tmp;
            string data = "";
            string avalues = "";


            tmp = (byte)sp.ReadByte();
            while (tmp != 255 && ((tmp != '\n')))
            {
                // Debug.Log(data.Length);
                data += (char)(tmp);
                tmp = (byte)sp.ReadByte();
                if ((tmp == '\n') && (data.Length > 15))
                {
                    avalues = data;
                    // Debug.Log("done parsing...");
                }
            }
            parseValues(avalues);

            AppControl.control.inertialObservations = new double[] { qrigid.x, qrigid.y, qrigid.z, qrigid.w, accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z, magn.x, magn.y, magn.z };
            AppControl.control.writeData();

            data = "";
        }

        ++frames;
        float timeNow = Time.realtimeSinceStartup;
        if (timeNow > lastInterval + updateInterval)
        {
            fps = (float)(frames / (timeNow - lastInterval));
            frames = 0;
            lastInterval = timeNow;
        }
    }

    void OnDestroy()
    {
        sp.Close();
    }
}
