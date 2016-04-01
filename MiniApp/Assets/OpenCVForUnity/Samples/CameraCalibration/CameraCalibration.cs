using UnityEngine;
using System.Collections;

using OpenCVForUnity;

using System.Collections.Generic;

namespace OpenCVForUnitySample
{
    /// <summary>
    /// CamShift sample.
    /// referring to the http://www.computervisiononline.com/blog/tutorial-using-camshift-track-objects-video.
    /// </summary>
    [RequireComponent(typeof(WebCamTextureToMatHelper))]
    public class CameraCalibration : MonoBehaviour
    {

        /// <summary>
        /// The colors.
        /// </summary>
        Color32[] colors;

        /// <summary>
        /// The texture.
        /// </summary>
        Texture2D texture;

        /// <summary>
        /// The roi point list.
        /// </summary>
        List<Point> roiPointList;

        /// <summary>
        /// The roi rect.
        /// </summary>
        OpenCVForUnity.Rect roiRect;

        /// <summary>
        /// The gray mat.
        /// </summary>
        Mat grayMat;

        /// <summary>
        /// The hsv mat.
        /// </summary>
        Mat hsvMat;

        /// <summary>
        /// The roi hist mat.
        /// </summary>
        Mat roiHistMat;

        /// <summary>
        /// The termination.
        /// </summary>
        TermCriteria termination;

        /// <summary>
        /// The web cam texture to mat helper.
        /// </summary>
        WebCamTextureToMatHelper webCamTextureToMatHelper;

        /// <summary>
        ///  Timing variables 
        /// </summary>
        public float updateInterval = 1f;
        private double lastInterval;
        int numCalibImages = 0;
        public int numCalibrationTakes = 20;

        // Calibration variables
        Size patternsize; // (<param1>, <param2>) : new size with width = <param1> and height = <param2>
        float squaresize; 
        MatOfPoint2f pointbuf; //this will be filled by the detected corners of findChessboardCorners
        MatOfPoint3f objectpointbuf; //this will be filled by the detected corners of findChessboardCorners
        List<Mat> imagePoints;
        List<Mat> objectPoints;
        public bool calibrationComplete;

        Size imageSize;
        public Mat cameraMatrix;
        public Mat distCoeffs;
        List<Mat> rvecs;
        List<Mat> tvecs;

        // Use this for initialization
        void Start ()
        {
            // AppControl.control.Load();

            roiPointList = new List<Point> ();
            termination = new TermCriteria (TermCriteria.EPS | TermCriteria.COUNT, 10, 1);
            
            webCamTextureToMatHelper = gameObject.GetComponent<WebCamTextureToMatHelper> ();
            webCamTextureToMatHelper.Init (OnWebCamTextureToMatHelperInited, OnWebCamTextureToMatHelperDisposed);

            // Calibration variables
            patternsize = new Size(9, 6); // (<param1>, <param2>) : new size with width = <param1> and height = <param2>
            squaresize = 22f; 
            pointbuf = new MatOfPoint2f(); //this will be filled by the detected corners of findChessboardCorners
            objectpointbuf = new MatOfPoint3f(); //this will be filled by the detected corners of findChessboardCorners
            imagePoints = new List<Mat>();
            objectPoints = new List<Mat>();
            calibrationComplete = false;

            imageSize = new Size(480, 640);
            cameraMatrix = new Mat();
            distCoeffs = new Mat();
            rvecs = new List<Mat>();
            tvecs = new List<Mat>();

            // Set the "lastInterval" to the system time
            lastInterval = Time.realtimeSinceStartup;

        }

				/// <summary>
				/// Raises the web cam texture to mat helper inited event.
				/// </summary>
				public void OnWebCamTextureToMatHelperInited ()
				{
						Debug.Log ("OnWebCamTextureToMatHelperInited");
			
						Mat webCamTextureMat = webCamTextureToMatHelper.GetMat ();
			
						colors = new Color32[webCamTextureMat.cols () * webCamTextureMat.rows ()];
						texture = new Texture2D (webCamTextureMat.cols (), webCamTextureMat.rows (), TextureFormat.RGBA32, false);

                        // Initialize object instances
						hsvMat = new Mat (webCamTextureMat.rows (), webCamTextureMat.cols (), CvType.CV_8UC3);
						grayMat = new Mat (webCamTextureMat.rows (), webCamTextureMat.cols (), CvType.CV_16U);
			
						gameObject.transform.localScale = new Vector3 (webCamTextureMat.cols (), webCamTextureMat.rows (), 1);
			
						Debug.Log ("Screen.width " + Screen.width + " Screen.height " + Screen.height + " Screen.orientation " + Screen.orientation);
			
						float width = 0;
						float height = 0;
			
						width = gameObject.transform.localScale.x;
						height = gameObject.transform.localScale.y;
			
						float widthScale = (float)Screen.width / width;
						float heightScale = (float)Screen.height / height;
						if (widthScale < heightScale) {
								Camera.main.orthographicSize = (width * (float)Screen.height / (float)Screen.width) / 2;
						} else {
								Camera.main.orthographicSize = height / 2;
						}
			
						gameObject.GetComponent<Renderer> ().material.mainTexture = texture;

				}

				/// <summary>
				/// Raises the web cam texture to mat helper disposed event.
				/// </summary>
				public void OnWebCamTextureToMatHelperDisposed ()
				{
						Debug.Log ("OnWebCamTextureToMatHelperDisposed");

						hsvMat.Dispose ();
						if (roiHistMat != null)
								roiHistMat.Dispose ();
						roiPointList.Clear ();
				}

        // Update is called once per frame
        void Update ()
        {

            if (webCamTextureToMatHelper.isPlaying () && webCamTextureToMatHelper.didUpdateThisFrame ()) {
        
                Mat rgbaMat = webCamTextureToMatHelper.GetMat ();

                if(AppControl.control.calibrationComplete)
                {
                    Mat rgbaMatUndistorted = rgbaMat.clone();
                    Size SizeCm = AppControl.control.cameraMatrix.size();
                    Size SizeDc = AppControl.control.distCoeffs.size();
                    Imgproc.undistort(rgbaMat, rgbaMatUndistorted, AppControl.control.cameraMatrix, AppControl.control.distCoeffs);
                    rgbaMat = rgbaMatUndistorted;
                }

                Imgproc.cvtColor (rgbaMat, hsvMat, Imgproc.COLOR_RGBA2RGB);
                Imgproc.cvtColor (hsvMat, hsvMat, Imgproc.COLOR_RGB2HSV);

                // IMAGE PROCESSING STARTS (rgbaMat is the captured image)

                // Convert the current rgba frame (rgbaMat) to a gray image (grayMat)
                Imgproc.cvtColor(rgbaMat, grayMat, Imgproc.COLOR_RGBA2GRAY);


                //CALIB_CB_FAST_CHECK saves a lot of time on images
                //that do not contain any chessboard corners
                bool patternfound = Calib3d.findChessboardCorners(grayMat, patternsize, pointbuf,
                        Calib3d.CALIB_CB_ADAPTIVE_THRESH + Calib3d.CALIB_CB_NORMALIZE_IMAGE
                        + Calib3d.CALIB_CB_FAST_CHECK);

                if (patternfound)
                {
                    Imgproc.cornerSubPix(grayMat, pointbuf, new Size(11, 11), new Size(-1, -1), new TermCriteria(TermCriteria.EPS + TermCriteria.MAX_ITER, 30, 0.1));

                    float timeNow = Time.realtimeSinceStartup;
                    if ((timeNow > lastInterval + updateInterval))
                    {

                        imagePoints.Add( pointbuf.clone() );
                        calcChessboardCorners(patternsize, squaresize, objectpointbuf);
                        objectPoints.Add( objectpointbuf.clone() );

                        ++numCalibImages;
                        lastInterval = timeNow;
                    }

                }


                if( numCalibImages > numCalibrationTakes && !calibrationComplete )
                {
                    // Do calibration

                    int flags = 0;
                    calibrationComplete = true;
                    Calib3d.calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, flags);

                    // Calibration is complete
                    List<float> perViewErrors = new List<float>();
                    float error;
                    error = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix, distCoeffs, perViewErrors);

                    // Debug statements
                    Debug.Log("Calibration complete!");
                    Debug.Log("Camera Matrix:");
                    Debug.Log(cameraMatrix.dump());
                    Debug.Log("Distortion Coefficients:");
                    Debug.Log(distCoeffs.dump()); // Debug.Log("fx: " + cameraMatrix.get(0,0)[0]);
                    Debug.Log("Reprojection error average: " + error); // Error should be in units of pixels
                    Debug.Log(perViewErrors.ToString());

                    // Save the calibration variables in AppControl
                    AppControl.control.calibrationComplete = calibrationComplete;
                    AppControl.control.cameraMatrix = cameraMatrix;
                    AppControl.control.distCoeffs = distCoeffs;
                    AppControl.control.reprojectionError = error;
                    AppControl.control.fx = (float)cameraMatrix.get(0, 0)[0];
                    AppControl.control.fy = (float)cameraMatrix.get(1, 1)[0];
                    AppControl.control.cx = (float)cameraMatrix.get(0, 2)[0];
                    AppControl.control.cy = (float)cameraMatrix.get(1, 2)[0];

                    // Store the calibration parameters in player preferences text file
                    PlayerPrefs.SetInt("Calibrated", 1);
                    PlayerPrefs.SetFloat("Error", error);
                    float k1 = (float)distCoeffs.get(0, 1)[0];
                    float k2 = (float)distCoeffs.get(0, 1)[0];
                    PlayerPrefs.SetFloat("k1", (float)distCoeffs.get(0,0)[0]);
                    PlayerPrefs.SetFloat("k2", (float)distCoeffs.get(0,1)[0]);
                    PlayerPrefs.SetFloat("p1", (float)distCoeffs.get(0,2)[0]);
                    PlayerPrefs.SetFloat("p2", (float)distCoeffs.get(0,3)[0]);
                    PlayerPrefs.SetFloat("k3", (float)distCoeffs.get(0,4)[0]);
                    PlayerPrefs.SetFloat("fx",(float)cameraMatrix.get(0, 0)[0]);
                    PlayerPrefs.SetFloat("fy", (float)cameraMatrix.get(1, 1)[0]);
                    PlayerPrefs.SetFloat("cx",(float)cameraMatrix.get(0, 2)[0]);
                    PlayerPrefs.SetFloat("cy",(float)cameraMatrix.get(1, 2)[0]);
                }

                     
                // SHOW IMAGE ON THE DISPLAY

                // Draw the chessboard corners so it's obvious that the app is working
                Calib3d.drawChessboardCorners(rgbaMat, patternsize, pointbuf, patternfound);

                // Notify the user when the calibration is finished
                if( !calibrationComplete )
                {
                    Core.putText (rgbaMat, "Number of images collected: " +  numCalibImages, new Point (5, rgbaMat.rows () - 10), Core.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar (255, 255, 255, 255), 2, Core.LINE_AA, false);
                }
                else
                {
                    // Mat rgbaMatUndistorted = new Mat();
                    Core.putText (rgbaMat, "Calibration complete", new Point (5, rgbaMat.rows () - 10), Core.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar (255, 255, 255, 255), 2, Core.LINE_AA, false);
                }


                Utils.matToTexture2D(rgbaMat, texture); // Default display
                //  Utils.matToTexture2D (rgbaMat, texture, colors)
            }

        }

        // modified from https://github.com/Itseez/opencv/blob/master/samples/cpp/calibration.cpp
        void calcChessboardCorners(Size boardSize, float squareSize, MatOfPoint3f corners)
        {
            List<Point3> lp = new List<Point3>();
            for (int i = 0; i < boardSize.height; i++) {
                for (int j = 0; j < boardSize.width; j++)
                    lp.Add(new Point3((j * squareSize), (i * squareSize), 0));
            }

            corners.fromList(lp);
        }

        // Taken from http://docs.opencv.org/3.0-beta/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
        float computeReprojectionErrors( List<Mat> objectPoints, List<Mat> imagePoints, List<Mat> rvecs, List<Mat> tvecs, Mat cameraMatrix , Mat distCoeffs, List<float> perViewErrors) {
            MatOfPoint2f imagePoints2 = new MatOfPoint2f();
            int i, totalPoints = 0;
            float totalErr = 0, err;

            int numItems = objectPoints.Count;
            for( i = 0; i < numItems; ++i )
            {
                MatOfPoint3f objectPointsi = new MatOfPoint3f(objectPoints[i]);
                MatOfPoint2f imagePointsi = new MatOfPoint2f(imagePoints[i]);

                MatOfDouble distCoeffsv2 = new MatOfDouble(distCoeffs);
                Calib3d.projectPoints(objectPointsi, rvecs[i], tvecs[i], cameraMatrix, distCoeffsv2, imagePoints2);

                err = norml2(imagePointsi, imagePoints2);              // difference

                Size temp = objectPoints[i].size();
                int n = (int)temp.height;
                perViewErrors.Add(Mathf.Sqrt(err*err / (n)));                        // save for this view
                totalErr += err* err;                                             // sum it up
                totalPoints += (int)n;
            }

            return Mathf.Sqrt(totalErr/totalPoints);              // calculate the arithmetical mean
        }

        float norml2(MatOfPoint2f m1, MatOfPoint2f m2)
        {
            float norm = 0f;

            // Type conversions
            List<Point> m1list = m1.toList();
            List<Point> m2list = m2.toList();

            // Array sizes
            int dimM1 = m1list.Count;
            int dimM2 = m2list.Count;
            
            // Ensure that the arrays are the same size
            if(dimM1 != dimM2)
            {
                return -1f;
            }

            for(int i = 0; i < dimM1; i++)
            {
                norm += Mathf.Sqrt(Mathf.Pow((float)(m2list[i].x - m1list[i].x), 2) + Mathf.Pow((float)(m2list[i].y - m1list[i].y), 2));
            }
            return norm;
        }

        /// <summary>
        /// Raises the disable event.
        /// </summary>
        void OnDisable () {
            webCamTextureToMatHelper.Dispose ();
        }

        public void OnRecalibrate () {

            // Clear containers
            objectPoints.Clear();
            imagePoints.Clear();
            numCalibImages = 0; 

            // Set the calibrationComplete parameter to false
            calibrationComplete = false;
            AppControl.control.calibrationComplete = false;
        }

        /// <summary>
        /// Raises the back button event.
        /// </summary>
        public void OnBackButton ()
        {
                Application.LoadLevel ("OpenCVForUnitySample");
        }

        /// <summary>
        /// Raises the play button event.
        /// </summary>
        public void OnPlayButton ()
        {
                webCamTextureToMatHelper.Play ();
        }

        /// <summary>
        /// Raises the pause button event.
        /// </summary>
        public void OnPauseButton ()
        {
                webCamTextureToMatHelper.Pause ();
        }

        /// <summary>
        /// Raises the stop button event.
        /// </summary>
        public void OnStopButton ()
        {
                webCamTextureToMatHelper.Stop ();
        }

        /// <summary>
        /// Raises the change camera button event.
        /// </summary>
        public void OnChangeCameraButton ()
        {
                webCamTextureToMatHelper.Init (null, webCamTextureToMatHelper.requestWidth, webCamTextureToMatHelper.requestHeight, !webCamTextureToMatHelper.requestIsFrontFacing, OnWebCamTextureToMatHelperInited, OnWebCamTextureToMatHelperDisposed);
        }

        /// <summary>
        /// Converts the screen point.
        /// </summary>
        /// <returns>The screen point.</returns>
        /// <param name="screenPoint">Screen point.</param>
        /// <param name="quad">Quad.</param>
        /// <param name="cam">Cam.</param>
        static Point convertScreenPoint (Point screenPoint, GameObject quad, Camera cam)
        {
                Vector2 tl;
                Vector2 tr;
                Vector2 br;
                Vector2 bl;
            

                tl = cam.WorldToScreenPoint (new Vector3 (quad.transform.localPosition.x - quad.transform.localScale.x / 2, quad.transform.localPosition.y + quad.transform.localScale.y / 2, quad.transform.localPosition.z));
                tr = cam.WorldToScreenPoint (new Vector3 (quad.transform.localPosition.x + quad.transform.localScale.x / 2, quad.transform.localPosition.y + quad.transform.localScale.y / 2, quad.transform.localPosition.z));
                br = cam.WorldToScreenPoint (new Vector3 (quad.transform.localPosition.x + quad.transform.localScale.x / 2, quad.transform.localPosition.y - quad.transform.localScale.y / 2, quad.transform.localPosition.z));
                bl = cam.WorldToScreenPoint (new Vector3 (quad.transform.localPosition.x - quad.transform.localScale.x / 2, quad.transform.localPosition.y - quad.transform.localScale.y / 2, quad.transform.localPosition.z));


                Mat srcRectMat = new Mat (4, 1, CvType.CV_32FC2);
                Mat dstRectMat = new Mat (4, 1, CvType.CV_32FC2);

                
                srcRectMat.put (0, 0, tl.x, tl.y, tr.x, tr.y, br.x, br.y, bl.x, bl.y);
                dstRectMat.put (0, 0, 0.0, 0.0, quad.transform.localScale.x, 0.0, quad.transform.localScale.x, quad.transform.localScale.y, 0.0, quad.transform.localScale.y);

                
                Mat perspectiveTransform = Imgproc.getPerspectiveTransform (srcRectMat, dstRectMat);

//						Debug.Log ("srcRectMat " + srcRectMat.dump ());
//						Debug.Log ("dstRectMat " + dstRectMat.dump ());
//						Debug.Log ("perspectiveTransform " + perspectiveTransform.dump ());

                MatOfPoint2f srcPointMat = new MatOfPoint2f (screenPoint);
                MatOfPoint2f dstPointMat = new MatOfPoint2f ();

                Core.perspectiveTransform (srcPointMat, dstPointMat, perspectiveTransform);

//						Debug.Log ("srcPointMat " + srcPointMat.dump ());
//						Debug.Log ("dstPointMat " + dstPointMat.dump ());

                return dstPointMat.toArray () [0];
        }
    }
}