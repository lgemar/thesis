using UnityEngine;
using System.Collections;
using OpenCVForUnity; 
using Matrix = MathNet.Numerics.LinearAlgebra.Matrix<float>;
using Vector = MathNet.Numerics.LinearAlgebra.Vector<float>;

using System.Threading;


public class Orientation : MonoBehaviour {

    public Controller control;

    public float AlignYaw = 0;
    public float[] floatData;

    // Kalman filter variables
    int i = 0; // frame counter

    // State and covariance estimates
    Vector qte;
    Matrix Pte;

    // Noise covariances
    Matrix Q;
    Matrix R;

    // Predicted state and covariance variables
    Vector qtp;
    Matrix Ptp;

    // Gyro input
    Vector u;


    // Reference vectors
    Vector gW = Vector.Build.Dense(new float[] { 0f, 0f, 9.81f }); // gravity vector in world coordinate frame
    Vector bW; // b-field vector in world coordinate frame

    // Measurement vectors
    Vector aB; // accelerometer reading in body frame
    Vector mB; // magnetometer reading in body frame
    Vector gB; // magnetometer reading in body frame

    public Vector qt = Vector.Build.Dense(new float[] { 0f, 0f, 0f, 0f }); // body-referenced quaternion (rotation from body to world)

    // Sample rate counter
    float updateInterval = 0.5F;
    double lastInterval;
    int frames = 0;
    public float fps;

	// Use this for initialization
	void Start () {

        // Iniitialize the Kalman filter variables

        // Covariance matrix initialization
        Vector pdiag = Vector.Build.DenseOfArray(new float[] { Mathf.Pow(deg2rad(10), 2), Mathf.Pow(deg2rad(10), 2), Mathf.Pow(deg2rad(15), 2) });
        Pte = Matrix.Build.DenseOfDiagonalVector(pdiag);

        // Process noise initialization
        Vector qdiag = 20 * Vector.Build.DenseOfArray(new float[] { 2.8e-6f, 2.8e-6f, 2.8e-6f });
        Q = Matrix.Build.DenseOfDiagonalVector(qdiag);

        // Measurement noise initialization
        float rdiag_accel = 200 * 1.1e-6f; 
        float rdiag_magn = 0.0001f * 17; 
        Vector rdiag = Vector.Build.DenseOfArray(new float[] {rdiag_accel, rdiag_accel, rdiag_accel, rdiag_magn, rdiag_magn, rdiag_magn});
        R = Matrix.Build.DenseOfDiagonalVector((1/3f)*rdiag);


    }
    

	// Update is called once per frame
	void Update () {

        // Read the new data
        floatData = (float[])control.floatData.Clone();
        bool ret = control.ret;

        // Accelerometer, gyroscope and magnetometer measurements
        aB = Vector.Build.Dense(new float[] { floatData[5], floatData[6], floatData[7] });
        gB = Vector.Build.Dense(new float[] { floatData[8], floatData[9], floatData[10] });
        mB = Vector.Build.Dense(new float[] { floatData[11], floatData[12], floatData[13] });

        // Initialize the b-field vector
        if (ret)
        {
            if (i == 0)
            {
                bW = Vector.Build.Dense(new float[] { mB[0], mB[1], mB[2] } ); // assume that the device starts flat on table

                i += 1;

                Debug.Log("aB vector: " + aB);
                Debug.Log("gB vector: " + gB);
                Debug.Log("mB vector: " + mB);

                qt = triad(gW, aB, bW, mB);
                qte = qt;
            }
            else
            {
                ////////////////////////////////////////////////////////////////////////
                // KALMAN FILTER BEGIN
                ////////////////////////////////////////////////////////////////////////

                // Gyro input
                float alpha; float beta;
                u = Vector.Build.Dense(new float[] { deg2rad(gB[0]), deg2rad(gB[1]), deg2rad(gB[2]) });
                alpha = Mathf.Cos((float)u.Norm(2) / 2f);
                if (u.Norm(2) > 0)
                {
                    beta = Mathf.Sin((float)u.Norm(2) / 2f) / (float)u.Norm(2);
                }
                else
                {
                    beta = 0.5f; 
                }

                // Coavariance prediction
                Ptp = Pte + Q;

                // State prediction
                qtp = alpha * qte + 0.7f * XiMat(qte) * u;

                // Sensitivity matrix
                Vector rB1 = AMat(qtp) * gW; 
                Vector rB2 = AMat(qtp) * bW; 
                Matrix H = Matrix.Build.DenseOfMatrixArray( new Matrix[,] { {crossmat(rB1)} , { crossmat(rB2) } });

                // Kalman gain
                Matrix K = Ptp * H.Transpose() * (H * Ptp * H.Transpose() + R).Inverse();

                // Measurements
                aB = aB / (float)aB.Norm(2); mB = mB / (float)mB.Norm(2); 
                Vector zm = Vector.Build.DenseOfArray(new float[] { aB[0], aB[1], aB[2], mB[0], mB[1], mB[2] }); // measurements

                // Predicted measurements
                rB1 = rB1 / (float)rB1.Norm(2); rB2 = rB2 / (float)rB2.Norm(2); 
                Vector ze = Vector.Build.DenseOfArray(new float[] { rB1[0], rB1[1], rB1[2], rB2[0], rB2[1], rB2[2] }); // measurement estimates

                // Innovation 
                Vector nu = zm - ze;

                // Innovation gain
                Vector dqvec = K * nu;
                Vector dq = Vector.Build.DenseOfArray(new float[] { dqvec[0], dqvec[1], dqvec[2], 1 });

                // State update
                qte = qtp + XiMat(qtp) * dqvec;
                qte = qte / (float)qte.Norm(2);

                // Covariance update
                Pte = (Matrix.Build.DenseIdentity(3) - K * H) * Ptp;

                ////////////////////////////////////////////////////////////////////////
                // KALMAN FILTER END
                ////////////////////////////////////////////////////////////////////////

                qt = qte;
            }
        }

        if(gameObject.tag == "Stick")
        {

        }
        if(gameObject.tag == "MainCamera")
        {
        }

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

    public Vector qinv(Vector q)
    {
        return Vector.Build.DenseOfArray(new float[] { -q[0], -q[1], -q[2], q[3] });
    }

    // A(q1) * A(q2)
    public Vector qmultiply(Vector q1, Vector q2)
    {
        Vector qtprod;
        Matrix M = Matrix.Build.DenseOfMatrixArray(new Matrix[,] { { XiMat(q2), q2.ToColumnMatrix() } });
        qtprod = M * q1; 
        return qtprod;
    }

    public Matrix XiMat(Vector q)
    {
        Matrix M = Matrix.Build.DenseOfArray(new float[,] {
                {q[3], -q[2], q[1] },
                {q[2], q[3], -q[0] },
                {-q[1], q[0], q[3] },
                {-q[0], -q[1], -q[2] } });
        return M;
    }

    public Matrix PsiMat(Vector q)
    {
        Matrix M = Matrix.Build.DenseOfArray(new float[,] {
            {q[3], q[2], -q[1] },
            {-q[2], q[3], q[0] },
            {q[1], -q[0], q[3] },
            {-q[0], -q[1], -q[2] } });
        return M;
    }

    public Matrix AMat(Vector q)
    {
        Matrix A = XiMat(q).Transpose() * PsiMat(q);
        return A;
    }

    public Vector triad(Vector r1, Vector m1, Vector r2, Vector m2)
    {
        Vector q; // return value

        // Compute the reference TRIAD
        r1.Norm(2); r2.Norm(2);
        Vector v1 = r1.Clone();
        Vector v2 = crossmat(r1) * r2;
        Vector v3 = crossmat(v1) * v2; 

        // Compute the measurement TRIAD
        m1.Norm(2); m2.Norm(2);
        Vector b1 = m1.Clone();
        Vector b2 = crossmat(m1) * m2;
        Vector b3 = crossmat(b1) * b2; 

        // Construct reference matrix and sensor matrix
        Matrix Mr = Matrix.Build.DenseOfColumnVectors(v1, v2, v3); 
        Matrix Mb = Matrix.Build.DenseOfColumnVectors(b1, b2, b3);

        // Construct the rotation matrix
        Matrix R = Mr * Mb.Inverse();

        // Construct the quaternion from the rotation matrix
        q = rotm2quat(R);

        return q;
    }

    // Better solution
    public Vector rotm2quat(Matrix m)
    {
        Matrix4x4 mat4x4 = new Matrix4x4();
        mat4x4.SetColumn(0, new Vector4(m[0, 0], m[1,0], m[2,0], 0));
        mat4x4.SetColumn(1, new Vector4(m[0, 1], m[1,1], m[2,1], 0));
        mat4x4.SetColumn(2, new Vector4(m[0, 2], m[1,2], m[2,2], 0));
        mat4x4.SetColumn(3, new Vector4(0, 0, 0, 0));
        Quaternion qtemp = Quaternion.LookRotation(mat4x4.GetColumn(2), mat4x4.GetColumn(1));
        Vector q = Vector.Build.Dense(new float[] { qtemp.x, qtemp.y, qtemp.z, qtemp.w });
        return q;
    }

    public Matrix crossmat(Vector x)
    {
        Matrix A = Matrix.Build.DenseOfArray(new float[,] {
             {0,-x[2],x[1]},
             {x[2],0,-x[0]},
             {-x[1],x[0],0}});
        return A;
    }

    public Vector rotm2quat2(Matrix m)
    {
        // Adapted from: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
        float w; float x; float y; float z;
        w = Mathf.Sqrt(Mathf.Max(0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2;
        x = Mathf.Sqrt(Mathf.Max(0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2;
        y = Mathf.Sqrt(Mathf.Max(0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2;
        z = Mathf.Sqrt(Mathf.Max(0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2;
        x *= Mathf.Sign(x * (m[2, 1] - m[1, 2]));
        y *= Mathf.Sign(y * (m[0, 2] - m[2, 0]));
        z *= Mathf.Sign(z * (m[1, 0] - m[0, 1]));
        Vector q = Vector.Build.Dense(new float[] { x, y, z, w });
        return q;
    }

    public float deg2rad(float deg)
    {
        return Mathf.PI * deg / 180f; 
    }
}
