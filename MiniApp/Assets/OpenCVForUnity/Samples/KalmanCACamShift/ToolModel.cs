using UnityEngine;
using System.Collections;
using Matrix = MathNet.Numerics.LinearAlgebra.Matrix<float>;
using Vector = MathNet.Numerics.LinearAlgebra.Vector<float>;

public class ToolModel : MonoBehaviour {

    public OpenCVForUnitySample.KalmanCACamShift putracker;

    // Object size in (mm)
    public float s = 30; // mm

    // Intrinsic matrix properties
    public float fx;
    public float fy;
    public float cx;
    public float cy;

    public float dtl;

    Vector pu; // position in undistorted image coordinates
    Vector pr; // position in camera coordinates
    Vector prpr; // position in display coordinates

    // State in the image
    public float pux;
    public float puy;
    public float pus;

    // Position of rear
    public float prx;
    public float pry;
    public float prz;

    GameObject mainCamera;

	// Use this for initialization
	void Start () {
        mainCamera = GameObject.FindWithTag("MainCamera");
	}

    // Update is called once per frame
    void Update() {
        ///////////////////////////////////////////////////////////////////////
        // Undistorted Image Measurements
        ///////////////////////////////////////////////////////////////////////
        pu = putracker.getState();
        pux = pu[0];
        puy = pu[3];
        pus = pu[6];

        ///////////////////////////////////////////////////////////////////////
        // World position measurements
        ///////////////////////////////////////////////////////////////////////
        prz = (s / pus) * 1 / Mathf.Sqrt(1 / Mathf.Pow(fx, 2) + 1 / Mathf.Pow(fy, 2));
        prx = (prz / fx) * (pux - cx);
        pry = (prz / fy) * (cy - puy);
        pr = Vector.Build.Dense(new float[] { prx, pry, prz });

        Matrix Ad = Matrix.Build.DenseOfArray(new float[,] { { -1, 0, 0 }, { 0, -1, 0 }, { 0, 0, 1 } });
        Vector pd = Vector.Build.DenseOfArray(new float[] { 0, 0, 1 });
        Vector prpr = pd - Ad * pr;

        ///////////////////////////////////////////////////////////////////////
        // Update Object's Virtual Position
        ///////////////////////////////////////////////////////////////////////
        Vector3 thisposition = new Vector3(prpr[0], prpr[1], prpr[2]);
        thisposition = 0.001f * thisposition;
        gameObject.transform.position = mainCamera.transform.position + thisposition;
	}
}
