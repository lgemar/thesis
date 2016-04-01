using UnityEngine;
using System.Collections;
using System.Threading;
using Matrix = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using Vector = MathNet.Numerics.LinearAlgebra.Vector<double>;
 
 
 public class MathNetTest : MonoBehaviour {
    // *************************************************************************
    void Start() {
        Matrix A = Matrix.Build.DenseOfArray(new double[,] {
             {1,0,0},
             {0,2,0},
             {0,0,3}});

        Vector b = Vector.Build.Dense(new double[] { 1, 1, 1 });

        Vector x = A.Solve(b);

        double[,] test = new double[3,3];
        test = A.ToArray();
        Debug.Log("Test:" + test.ToString());

        Debug.Log("x = A^-1b: " + x);
        string s = A.ToMatrixString();
        string[] sarray = s.Split('\n');
     }
 }
