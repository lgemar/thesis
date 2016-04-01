using UnityEngine;
using System.Collections;
using System.IO.Ports;

public class BreadboardControl : MonoBehaviour
{

    // Data variables
    public string[] stringData;
    float[] floatData;

    // COM Location and serial port object
    public string comValue;
    public int BaudRate;
    ComPort DataSource;

    // Number of data elements per line
    public int N = 5;

    // sample rate counter
    float updateInterval = 0.5f;
    float lastInterval;
    int frames = 0;
    public float fps;

    // Orientation variables
    public Vector3 BodyToGraphic = new Vector3(-90, 0, 270);

    // Camera Reference
    GameObject MainCamera;

    // Self-reference
    BreadboardControl breadboard;

	// Use this for initialization
	void Awake () {

        // ensure that there is only one instance of this object
        if(breadboard == null)
        {
            DontDestroyOnLoad(gameObject);
            breadboard = this;
        }
        else if(breadboard != this)
        {
            Destroy(gameObject);
        }

	}

    void Start()
    {
//        DataSource = new ComPort(comValue, BaudRate);
        DataSource.Open();
        stringData = new string[N];
        floatData = new float[N];
        MainCamera = GameObject.FindWithTag("MainCamera");
    }

    void Update()
    {
         DataSource.ReadData(ref stringData, ref floatData, N);

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
        Quaternion qWorldToBody = new Quaternion(floatData[2], floatData[4], floatData[3], floatData[1]); // body to world
        Quaternion qBodyToGraphic = Quaternion.Euler(BodyToGraphic[0], BodyToGraphic[1], BodyToGraphic[2]); // world to graphic
        gameObject.transform.rotation = qWorldToBody * qBodyToGraphic; // lhs rotation then rhs rotation
    }

    void OnDestroy()
    {
        DataSource.Close();
    }
}
