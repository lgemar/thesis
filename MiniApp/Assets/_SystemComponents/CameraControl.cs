using UnityEngine;
using System.Collections;
using System.IO.Ports;

public class CameraControl : MonoBehaviour
{

    // COM Location and serial port object
    public string comValue;
    public int BaudRate;
    ComPort DataSource;

    // Number of data elements per line
    public int N = 5;

    // Orientation variables
    public Vector3 SensorToBody = new Vector3(-90, 0, 90);
    public Quaternion qBodyToWorld = new Quaternion(0, 0, 0, 0);
    public Vector3 WorldToGraphic = new Vector3(0, -90, 0);

    // Sample rate counter
    float updateInterval = 0.5F;
    double lastInterval;
    int frames = 0;
    public float fps;

    // Data variables
    public string[] stringData;
    float[] floatData;

    void Start()
    {
//        DataSource = new ComPort(comValue, BaudRate);
        DataSource.Open();
        stringData = new string[N];
        floatData = new float[N];
    }

    void Update()
    {
//        DataSource.ReadData(ref stringData, ref floatData, N);

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

    void FixedUpdate()
    {
        qBodyToWorld = new Quaternion(floatData[2], floatData[4], floatData[3], floatData[1]); // body to world
        Quaternion qSensorToBody = Quaternion.Euler(SensorToBody[0], SensorToBody[1], SensorToBody[2]);
        Quaternion qWorldToGraphic = Quaternion.Euler(WorldToGraphic[0], WorldToGraphic[1], WorldToGraphic[2]); // world to graphic
        gameObject.transform.rotation = qSensorToBody * qBodyToWorld * qWorldToGraphic; // lhs rotation then rhs rotation
    }

    void OnDestroy()
    {
        DataSource.Close();
    }
}
