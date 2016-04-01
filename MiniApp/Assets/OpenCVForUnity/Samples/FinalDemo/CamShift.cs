using UnityEngine;
using System.Collections;

using OpenCVForUnity;

using System.Collections.Generic;

using Matrix = MathNet.Numerics.LinearAlgebra.Matrix<float>;
using Vector = MathNet.Numerics.LinearAlgebra.Vector<float>;

namespace OpenCVForUnitySample
{
    /// CamShift sample.
    /// referring to the http://www.computervisiononline.com/blog/tutorial-using-camshift-track-objects-video.
    [RequireComponent(typeof(WebCamTextureToMatHelper))]
    public class CamShift : MonoBehaviour
    {

        /// The colors.
        Color32[] colors;

        /// The texture.
        Texture2D texture;

        /// The roi point list.
        List<Point> roiPointList;

        /// The roi rect.
        OpenCVForUnity.Rect roiRect;
        OpenCVForUnity.Rect roiPred;
        OpenCVForUnity.Rect roiSearch;

        /// The hsv mat.
        Mat hsvMat;

        /// The roi hist mat.
        Mat roiHistMat;

        /// The termination.
        TermCriteria termination;

        /// The web cam texture to mat helper.
        WebCamTextureToMatHelper webCamTextureToMatHelper;

        Matrix Q; // Process noise covariance
        Matrix R; // Measurement noise covariance

        Matrix F; // state transition matrix
        Matrix H; // observation matrix

        Matrix Pp; // covariance prediction
        Matrix Pe; // covariance estimation

        Vector xp; // state prediction
        Vector xe; // state estimate;

        Matrix zero1x2 = 0 * Matrix.Build.DenseOfArray(new float[,] { { 0f, 0f } });
        Matrix zero1x3 = 0 * Matrix.Build.DenseOfArray(new float[,] { { 0f, 0f, 0f } });
        Matrix zero2x1 = 0 * Matrix.Build.DenseOfArray(new float[,] { { 0 }, { 0 } });
        Matrix zero3x1 = 0 * Matrix.Build.DenseOfArray(new float[,] { { 0 }, { 0 } , { 0 } });
        Matrix zero2x2 = 0 * Matrix.Build.DenseIdentity(2);
        Matrix zero3x3 = 0 * Matrix.Build.DenseIdentity(3);

        public bool ObjectFound;
        public float SpeedAtFailure = -1f;

        public float pn = 1;
        public float mn = 1;

        public float[] StateArray;
        public float[] CovarianceTrace;
        public float[] InnovationGain;

        // Sample rate counter
        float updateInterval = 0.5F;
        double lastInterval;
        int frames = 0;
        public float fps;
        public float dt;

        // Use this for initialization
        void Start()
        {
            // ROI variable initialization 
            roiPointList = new List<Point>();
            termination = new TermCriteria(TermCriteria.EPS | TermCriteria.COUNT, 20, 1);

            // Unity graphic object initialization
            webCamTextureToMatHelper = gameObject.GetComponent<WebCamTextureToMatHelper>();
            webCamTextureToMatHelper.Init(OnWebCamTextureToMatHelperInited, OnWebCamTextureToMatHelperDisposed);

            // Iniitialize the Kalman filter variables

            // Initialize roi objects
            roiRect = new OpenCVForUnity.Rect();
            roiPred = new OpenCVForUnity.Rect();
            roiSearch = new OpenCVForUnity.Rect();

            fps = 60f; // expected frame rate
            dt = 1 / fps; // expected sampling interval 


            // Observation matrix, H
            Matrix H2 = Matrix.Build.DenseOfArray(new float[,] { { 1, 0, 0 } });
            H = Matrix.Build.DenseOfMatrixArray(new Matrix[,]
            {
                {H2, zero1x3 , zero1x3},
                {zero1x3, H2 , zero1x3},
                {zero1x3, zero1x3 , H2}
            });

        }


        // Update is called once per frame
        void Update()
        {
            // Sample rate update
            ++frames;
            float timeNow = Time.realtimeSinceStartup;
            if (timeNow > lastInterval + updateInterval)
            {
                fps = (float)(frames / (timeNow - lastInterval));
                frames = 0;
                lastInterval = timeNow;
            }

            // Time since last update
            float dt = 1 / fps;

            if (webCamTextureToMatHelper.isPlaying() && webCamTextureToMatHelper.didUpdateThisFrame())
            {

                Mat rgbaMat = webCamTextureToMatHelper.GetMat();
                Imgproc.cvtColor(rgbaMat, hsvMat, Imgproc.COLOR_RGBA2RGB);
                Imgproc.cvtColor(hsvMat, hsvMat, Imgproc.COLOR_RGB2HSV);


                Point[] points = roiPointList.ToArray();

                if (roiPointList.Count == 4)
                {


                    using (Mat backProj = new Mat())
                    {
                        Imgproc.calcBackProject(new List<Mat>(new Mat[] { hsvMat }), new MatOfInt(0), roiHistMat, backProj, new MatOfFloat(0, 180), 1.0);


                        ///////////////////////////////////////////////////////
                        // Kalman Filter Start
                        ///////////////////////////////////////////////////////

                        // Process noise matrix, Q
                        Matrix Q2 = (Mathf.Pow(pn, 2) / dt) * Matrix.Build.DenseOfArray(new float[,]
                        {
                            { Mathf.Pow(dt,4)/4, Mathf.Pow(dt,3) / 2f, Mathf.Pow(dt,2) / 2f},
                            { Mathf.Pow(dt,3) / 2f, Mathf.Pow(dt,2) / 2f, dt},
                            { Mathf.Pow(dt,2) / 2f, dt, 1 }
                        });

                        Q = (Mathf.Pow(pn, 2) / dt) * Matrix.Build.DenseOfMatrixArray(new Matrix[,]
                        {
                            {Q2, zero3x3 , zero3x3},
                            {zero3x3, Q2 , zero3x3},
                            {zero3x3, zero3x3, Q2 }
                        });

                        // Measurement noise matrix, R
                        R = Mathf.Pow(mn, 2) * Matrix.Build.DenseIdentity(3);

                        // Build transition and control matrices
                        Matrix F2 = Matrix.Build.DenseOfArray(new float[,] { { 1, dt, Mathf.Pow(dt,2)/2f }, { 0, 1 , dt}, { 0, 0, 1 } });
                        Matrix F = Matrix.Build.DenseOfMatrixArray(new Matrix[,]
                        {
                            {F2, zero3x3 , zero3x3},
                            {zero3x3, F2 , zero3x3},
                            {zero3x3, zero3x3 , F2},
                        });

                        // Prediction
                        Pp = F * Pe * F.Transpose() + Q;
                        xp = F * xe;

                        roiPred = new OpenCVForUnity.Rect((int)(xp[0] - 0.5f * xp[6]), (int)(xp[3] - 0.5f * xp[6]), (int)xp[6], (int)xp[6]);
                        roiRect = new OpenCVForUnity.Rect((int)(xp[0] - 0.5f * roiSearch.width), (int)(xp[3] - 0.5f * roiSearch.height), roiSearch.width, roiSearch.height);
                        // roiRect = roiPred.clone();

                        RotatedRect r = Video.CamShift(backProj, roiRect, termination);
                        ObjectFound = (roiRect.height > 0 && roiRect.width > 0);

                        // Innovation
                        Vector nu;
                        if (ObjectFound)
                        {
                            // Innovation
                            Vector zk = Vector.Build.DenseOfArray(new float[]{ (int)(roiRect.x + 0.5f * roiRect.width),
                                                                        (int)(roiRect.y + 0.5f * roiRect.height),
                                                                        (int)(0.5 * (roiRect.width + roiRect.height))});
                            nu = zk - H * xp;

                            // Search window update
                            roiSearch = r.boundingRect().clone();

                            // Debug
                            SpeedAtFailure = -1f;
                        }
                        else
                        {

                            roiRect = roiPred.clone();

                            if (xp[0] < 0 || xp[0] < 0 || xp[0] > 640 || xp[3] > 480)
                            {
                                xp[0] = 320f; xp[1] = 0; xp[2] = 0;
                                xp[3] = 240f; xp[4] = 0; xp[5] = 0;
                                xp[6] = 40f; xp[7] = 0; xp[8] = 0;

                                roiRect.x = (int)(320 - 0.5f * 40);
                                roiRect.y = (int)(240 - 0.5f * 40);
                                roiRect.height = 40;
                                roiRect.width = 40;

                                roiPred = roiRect.clone();
                            }

                            // Innovation
                            Vector zk = Vector.Build.DenseOfArray(new float[]{ (float)(roiRect.x + 0.5f * roiRect.width),
                                                                        (float)(roiRect.y + 0.5f * roiRect.height),
                                                                        (float)(0.5 * (roiRect.width + roiRect.height))});

                            nu = zk - H * xp;
                            roiSearch = roiPred.clone();
                        }

                        // Kalman gain
                        Matrix K = Pp * H.Transpose() * R.Transpose();


                        // Innovation gain
                        Vector gain = K * nu;

                        // State update
                        xe = xp + gain;

                        // Covariance update
                        Pe = (Pp.Inverse() + H.Transpose() * R.Transpose() * H).Inverse();

                        // Display results to console
                        StateArray = xe.ToArray();
                        InnovationGain = gain.ToArray();
                        CovarianceTrace = Pe.Diagonal().ToArray();

                        ///////////////////////////////////////////////////////
                        // Kalman Filter End
                        ///////////////////////////////////////////////////////

                        r.points(points);
                    }

                    if (Input.GetMouseButtonUp(0))
                    {
                        roiPointList.Clear();
                    }
                }


                if (roiPointList.Count < 4)
                {
                    if (Input.GetMouseButtonUp(0))
                    {

                        roiPointList.Add(convertScreenPoint(new Point(Input.mousePosition.x, Input.mousePosition.y), gameObject, Camera.main));
                        //												Debug.Log ("mouse X " + Input.mousePosition.x);
                        //												Debug.Log ("mouse Y " + Input.mousePosition.y);

                        if (!(new OpenCVForUnity.Rect(0, 0, hsvMat.width(), hsvMat.height()).contains(roiPointList[roiPointList.Count - 1])))
                        {
                            roiPointList.RemoveAt(roiPointList.Count - 1);
                        }
                    }

                    if (roiPointList.Count == 4)
                    {

                        using (MatOfPoint roiPointMat = new MatOfPoint(roiPointList.ToArray()))
                        {
                            roiRect = Imgproc.boundingRect(roiPointMat);
                            roiPred = roiRect.clone();
                            roiSearch = roiRect.clone();
                        }

                        ///////////////////////////////////////////////////////
                        // Kalman Filter Initialize
                        ///////////////////////////////////////////////////////
                        Pe = Matrix.Build.DenseIdentity(9, 9);
                        Vector z1 = roi2center(roiRect);
                        xe = Vector.Build.DenseOfArray(new float[] { z1[0], 0, 0, z1[1], 0, 0, (roiRect.width + roiRect.height)/2, 0, 0});

                        ///////////////////////////////////////////////////////
                        // End Kalman Filter Initialize
                        ///////////////////////////////////////////////////////

                        if (roiHistMat != null)
                        {
                            roiHistMat.Dispose();
                            roiHistMat = null;
                        }
                        roiHistMat = new Mat();

                        using (Mat roiHSVMat = new Mat(hsvMat, roiRect))
                        using (Mat maskMat = new Mat())
                        {


                            Imgproc.calcHist(new List<Mat>(new Mat[] { roiHSVMat }), new MatOfInt(0), maskMat, roiHistMat, new MatOfInt(16), new MatOfFloat(0, 180));
                            Core.normalize(roiHistMat, roiHistMat, 0, 255, Core.NORM_MINMAX);

                            //														Debug.Log ("roiHist " + roiHistMat.ToString ());
                        }
                    }
                }

                if (points.Length < 4)
                {

                    for (int i = 0; i < points.Length; i++)
                    {
                        Core.circle(rgbaMat, points[i], 6, new Scalar(0, 0, 255, 255), 2);
                    }

                }
                else {
                    Core.rectangle(rgbaMat, roiRect.tl(), roiRect.br(), new Scalar(0, 255, 0, 255), 2);
                    Core.rectangle(rgbaMat, roiPred.tl(), roiPred.br(), new Scalar(0, 0, 255, 255), 2);
                    Core.rectangle(rgbaMat, roiSearch.tl(), roiSearch.br(), new Scalar(255, 0, 0, 255), 2);
                }

                Core.putText(rgbaMat, "PLEASE TOUCH 4 POINTS", new Point(5, rgbaMat.rows() - 10), Core.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(255, 255, 255, 255), 2, Core.LINE_AA, false);

                //				Imgproc.putText (rgbaMat, "W:" + rgbaMat.width () + " H:" + rgbaMat.height () + " SO:" + Screen.orientation, new Point (5, rgbaMat.rows () - 10), Core.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar (255, 255, 255, 255), 2, Imgproc.LINE_AA, false);

                Utils.matToTexture2D(rgbaMat, texture, colors);
            }

        }


        ///////////////////////////////////////////////////////////////////////
        // Helper functions
        ///////////////////////////////////////////////////////////////////////

        public Vector getState()
        {
            return xe;
        }


        Vector roi2center(OpenCVForUnity.Rect roi)
        {
            float xU = 0.5f * (roi.x + (roi.x + roi.width));
            float yU = 0.5f * (roi.y + (roi.y + roi.height));
            Vector meas = Vector.Build.DenseOfArray(new float[] { xU, yU }); // measurement vector
            return meas;
        }

        /// <summary>
        /// Raises the disable event.
        /// </summary>
        void OnDisable()
        {
            webCamTextureToMatHelper.Dispose();
        }

        /// <summary>
        /// Raises the back button event.
        /// </summary>
        public void OnBackButton()
        {
            Application.LoadLevel("OpenCVForUnitySample");
        }

        /// <summary>
        /// Raises the play button event.
        /// </summary>
        public void OnPlayButton()
        {
            webCamTextureToMatHelper.Play();
        }

        /// <summary>
        /// Raises the pause button event.
        /// </summary>
        public void OnPauseButton()
        {
            webCamTextureToMatHelper.Pause();
        }

        /// <summary>
        /// Raises the stop button event.
        /// </summary>
        public void OnStopButton()
        {
            webCamTextureToMatHelper.Stop();
        }

        /// <summary>
        /// Raises the change camera button event.
        /// </summary>
        public void OnChangeCameraButton()
        {
            webCamTextureToMatHelper.Init(null, webCamTextureToMatHelper.requestWidth, webCamTextureToMatHelper.requestHeight, !webCamTextureToMatHelper.requestIsFrontFacing, OnWebCamTextureToMatHelperInited, OnWebCamTextureToMatHelperDisposed);
        }

        /// <summary>
        /// Converts the screen point.
        /// </summary>
        /// <returns>The screen point.</returns>
        /// <param name="screenPoint">Screen point.</param>
        /// <param name="quad">Quad.</param>
        /// <param name="cam">Cam.</param>
        static Point convertScreenPoint(Point screenPoint, GameObject quad, Camera cam)
        {
            Vector2 tl;
            Vector2 tr;
            Vector2 br;
            Vector2 bl;


            tl = cam.WorldToScreenPoint(new Vector3(quad.transform.localPosition.x - quad.transform.localScale.x / 2, quad.transform.localPosition.y + quad.transform.localScale.y / 2, quad.transform.localPosition.z));
            tr = cam.WorldToScreenPoint(new Vector3(quad.transform.localPosition.x + quad.transform.localScale.x / 2, quad.transform.localPosition.y + quad.transform.localScale.y / 2, quad.transform.localPosition.z));
            br = cam.WorldToScreenPoint(new Vector3(quad.transform.localPosition.x + quad.transform.localScale.x / 2, quad.transform.localPosition.y - quad.transform.localScale.y / 2, quad.transform.localPosition.z));
            bl = cam.WorldToScreenPoint(new Vector3(quad.transform.localPosition.x - quad.transform.localScale.x / 2, quad.transform.localPosition.y - quad.transform.localScale.y / 2, quad.transform.localPosition.z));


            Mat srcRectMat = new Mat(4, 1, CvType.CV_32FC2);
            Mat dstRectMat = new Mat(4, 1, CvType.CV_32FC2);


            srcRectMat.put(0, 0, tl.x, tl.y, tr.x, tr.y, br.x, br.y, bl.x, bl.y);
            dstRectMat.put(0, 0, 0.0, 0.0, quad.transform.localScale.x, 0.0, quad.transform.localScale.x, quad.transform.localScale.y, 0.0, quad.transform.localScale.y);


            Mat perspectiveTransform = Imgproc.getPerspectiveTransform(srcRectMat, dstRectMat);

            //						Debug.Log ("srcRectMat " + srcRectMat.dump ());
            //						Debug.Log ("dstRectMat " + dstRectMat.dump ());
            //						Debug.Log ("perspectiveTransform " + perspectiveTransform.dump ());

            MatOfPoint2f srcPointMat = new MatOfPoint2f(screenPoint);
            MatOfPoint2f dstPointMat = new MatOfPoint2f();

            Core.perspectiveTransform(srcPointMat, dstPointMat, perspectiveTransform);

            //						Debug.Log ("srcPointMat " + srcPointMat.dump ());
            //						Debug.Log ("dstPointMat " + dstPointMat.dump ());

            return dstPointMat.toArray()[0];
        }

        /// Raises the web cam texture to mat helper inited event.
        public void OnWebCamTextureToMatHelperInited()
        {
            Debug.Log("OnWebCamTextureToMatHelperInited");

            Mat webCamTextureMat = webCamTextureToMatHelper.GetMat();

            colors = new Color32[webCamTextureMat.cols() * webCamTextureMat.rows()];
            texture = new Texture2D(webCamTextureMat.cols(), webCamTextureMat.rows(), TextureFormat.RGBA32, false);

            hsvMat = new Mat(webCamTextureMat.rows(), webCamTextureMat.cols(), CvType.CV_8UC3);

            gameObject.transform.localScale = new Vector3(webCamTextureMat.cols(), webCamTextureMat.rows(), 1);

            Debug.Log("Screen.width " + Screen.width + " Screen.height " + Screen.height + " Screen.orientation " + Screen.orientation);

            float width = 0;
            float height = 0;

            width = gameObject.transform.localScale.x;
            height = gameObject.transform.localScale.y;

            float widthScale = (float)Screen.width / width;
            float heightScale = (float)Screen.height / height;
            if (widthScale < heightScale)
            {
                Camera.main.orthographicSize = (width * (float)Screen.height / (float)Screen.width) / 2;
            }
            else {
                Camera.main.orthographicSize = height / 2;
            }

            gameObject.GetComponent<Renderer>().material.mainTexture = texture;

        }

        /// <summary>
        /// Raises the web cam texture to mat helper disposed event.
        /// </summary>
        public void OnWebCamTextureToMatHelperDisposed()
        {
            Debug.Log("OnWebCamTextureToMatHelperDisposed");

            hsvMat.Dispose();
            if (roiHistMat != null)
                roiHistMat.Dispose();
            roiPointList.Clear();
        }
    }
}