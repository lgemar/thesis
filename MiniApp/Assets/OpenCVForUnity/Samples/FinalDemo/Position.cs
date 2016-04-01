using UnityEngine;
using System.Collections;
using Matrix = MathNet.Numerics.LinearAlgebra.Matrix<float>;
using Vector = MathNet.Numerics.LinearAlgebra.Vector<float>;

public class Position : MonoBehaviour {

    public OpenCVForUnitySample.CamShift puTracker;
    public Controller control;

    // Object size in (mm)
    public float ObjectSize = 30; // mm

    // Intrinsic matrix properties
    float fx;
    float fy;
    float cx;
    float cy;

    public Vector pu; // position in undistorted image coordinates
    public Vector pr; // position in camera coordinates

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

        fx = control.fx;
        fy = control.fy;
        cx = control.cx;
        cy = control.cy;
	}

    // Update is called once per frame
    void Update() {
        ///////////////////////////////////////////////////////////////////////
        // Undistorted Image Measurements
        ///////////////////////////////////////////////////////////////////////
        pu = puTracker.getState();
        pux = pu[0];
        puy = pu[3];
        pus = pu[6];

        ///////////////////////////////////////////////////////////////////////
        // World position measurements
        ///////////////////////////////////////////////////////////////////////
        prz = (ObjectSize / pus) * 1 / Mathf.Sqrt(1 / Mathf.Pow(fx, 2) + 1 / Mathf.Pow(fy, 2));
        prx = (prz / fx) * (pux - cx);
        pry = (prz / fy) * (puy - cy);
        pr = Vector.Build.Dense(new float[] { prx, pry, prz });

	}
}
