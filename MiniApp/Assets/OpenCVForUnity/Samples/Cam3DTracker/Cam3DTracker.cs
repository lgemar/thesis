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
    public class Cam3DTracker : MonoBehaviour
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

        // 3D Tracking Variables
        public bool objectFound = false;
        public float objectSize = 35f; // Size of the object in mm (world coordinates)
        public Vector3 objectPosition; // Position of the object in camera coordinates
        GameObject target;
        GameObject stick;
        GameObject mainCamera;

        // Use this for initialization
        void Start ()
        {
            roiPointList = new List<Point> ();
            termination = new TermCriteria (TermCriteria.EPS | TermCriteria.COUNT, 10, 1);
            target = GameObject.FindWithTag("Target");
            stick = GameObject.FindWithTag("Stick");
            mainCamera = GameObject.FindWithTag("MainCamera");

            // Initialize the observation array
            AppControl.control.camObservations = new double[] { 0, 0, 0 }; 
            
            webCamTextureToMatHelper = gameObject.GetComponent<WebCamTextureToMatHelper> ();
            webCamTextureToMatHelper.Init (OnWebCamTextureToMatHelperInited, OnWebCamTextureToMatHelperDisposed);
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

            hsvMat = new Mat (webCamTextureMat.rows (), webCamTextureMat.cols (), CvType.CV_8UC3);

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

            /*
            hsvMat.Dispose ();
            if (roiHistMat != null)
                roiHistMat.Dispose ();
            roiPointList.Clear ();
            */
        }

        // Update is called once per frame
        void Update ()
        {

            // Debug.Log("Did update this frame: " + webCamTextureToMatHelper.didUpdateThisFrame());
            // Debug.Log("WebCam Texture is playing: " + webCamTextureToMatHelper.isPlaying());
            if (webCamTextureToMatHelper.isPlaying () && webCamTextureToMatHelper.didUpdateThisFrame ()) {
    
                ///////////////////////////////////////////////////////////////
                // Acquire the next frame from the camera and undistort if necessary
                ///////////////////////////////////////////////////////////////
                Mat rgbaMat = webCamTextureToMatHelper.GetMat ();

                if(AppControl.control.calibrationComplete && false)
                {
                    Mat rgbaMatUndistorted = rgbaMat.clone();
                    Imgproc.undistort(rgbaMat, rgbaMatUndistorted, AppControl.control.cameraMatrix , AppControl.control.distCoeffs);
                    rgbaMat = rgbaMatUndistorted;
                }

                Imgproc.cvtColor (rgbaMat, hsvMat, Imgproc.COLOR_RGBA2RGB);
                Imgproc.cvtColor (hsvMat, hsvMat, Imgproc.COLOR_RGB2HSV);

                ///////////////////////////////////////////////////////////////
                // If the object color spectrum is initialized, find the object
                ///////////////////////////////////////////////////////////////
                // Debug.Log("Number of ROI points: " + roiPointList.Count);
                Point[] points = roiPointList.ToArray ();
                if (roiPointList.Count == 4) {
                                        

                    using (Mat backProj = new Mat ()) {
                        Imgproc.calcBackProject (new List<Mat> (new Mat[]{hsvMat}), new MatOfInt (0), roiHistMat, backProj, new MatOfFloat (0, 180), 1.0);
                        // Video.meanShift(backProj, roiRect, termination);
                        RotatedRect r = Video.CamShift (backProj, roiRect, termination);
                        r.points (points);
                        if(roiRect.height > 0 && roiRect.width > 0)
                        {
                            objectFound = true;
                            AppControl.control.TargetBox = roiRect;
                            // Debug.Log("Object found: " + objectFound);
                        }
                        else
                        {
                            objectFound = false;
                        }
                    }

                    // Estimate the 3D position of the object
                    if(objectFound && AppControl.control.calibrationComplete)
                    {
                        float fx = AppControl.control.fx; // focal length x-axis
                        float fy = AppControl.control.fy; // focal length y-axis
                        float cx = AppControl.control.cx; // focal length x-axis
                        float cy = AppControl.control.cy; // focal length y-axis
                        float xu = (roiRect.x) + 0.5f * roiRect.width - cx; // undistorted object center along x-axis in pixels
                        float yu = - ( (roiRect.y) + 0.5f * roiRect.height - cy ); // undistorted object center along y-axis in pixels
                        float du = (roiRect.height + roiRect.width) / 2f; // Size of the object in pixels
                        float pz = (objectSize / du) * (1 / Mathf.Sqrt( 1f / Mathf.Pow(fx,2) + 1f / Mathf.Pow(fy, 2))) ;
                        float px = xu * pz / fx; 
                        float py = yu * pz / fy; 
                        objectPosition = new Vector3(px, py, pz); // in units of mm
                        target = GameObject.FindWithTag("Target");
                        stick = GameObject.FindWithTag("Breadboard");
                        if(target != null && stick != null)
                        {
                            target.transform.position = mainCamera.transform.position + 0.001f * objectPosition; // in units of meters
                            AppControl.control.camObservations = new double[] { xu, yu, du};
                            AppControl.control.writeData();
                            // stick.transform.rotation = mainCamera.transform.rotation * stick.transform.rotation * Quaternion.Inverse(mainCamera.transform.rotation);
                            // stick.transform.position = stick.transform.position

                        }
                    }
                    else
                    {
                        objectPosition = new Vector3(0, 0, 0);
                    }


                    // not necessary to reset the object tracking by clicking
                    /*
                    #if ((UNITY_ANDROID || UNITY_IOS) && !UNITY_EDITOR)
                        //Touch
                        int touchCount = Input.touchCount;
                        if (touchCount == 1)
                        {
                            if(Input.GetTouch(0).phase == TouchPhase.Ended){
                                roiPointList.Clear ();
                            }
                        }
                    #else
                    if (Input.GetMouseButtonUp (0)) {
                            roiPointList.Clear (); 
                    }
                    #endif
                    */
                }


                ///////////////////////////////////////////////////////////////
                // Capture the ROI from user input; compute the HSV histogram 
                // of the ROI
                ///////////////////////////////////////////////////////////////
                if (roiPointList.Count < 4) {

                    #if ((UNITY_ANDROID || UNITY_IOS) && !UNITY_EDITOR)
                        //Touch
                        int touchCount = Input.touchCount;
                        if (touchCount == 1)
                        {
                            Touch t = Input.GetTouch(0);
                            if(t.phase == TouchPhase.Ended){
                                roiPointList.Add (convertScreenPoint (new Point (t.position.x, t.position.y), gameObject, Camera.main));
//									Debug.Log ("touch X " + t.position.x);
//									Debug.Log ("touch Y " + t.position.y);

                                if (!(new OpenCVForUnity.Rect (0, 0, hsvMat.width (), hsvMat.height ()).contains (roiPointList [roiPointList.Count - 1]))) {
                                    roiPointList.RemoveAt (roiPointList.Count - 1);
                                }
                            }
                            
                        }
                    #else
                    // Capture mouse input events and add the points to the ROI List
                    if (Input.GetMouseButtonUp (0)) {
                                            
                            roiPointList.Add (convertScreenPoint (new Point (Input.mousePosition.x, Input.mousePosition.y), gameObject, Camera.main));
                            //												Debug.Log ("mouse X " + Input.mousePosition.x);
                            //												Debug.Log ("mouse Y " + Input.mousePosition.y);

                            if (!(new OpenCVForUnity.Rect (0, 0, hsvMat.width (), hsvMat.height ()).contains (roiPointList [roiPointList.Count - 1]))) {
                                    roiPointList.RemoveAt (roiPointList.Count - 1);
                            }
                    }
                    #endif

                    // If the user has selected four points, lock in the ROI; compute
                    // the HSV histogram of the ROI
                    if (roiPointList.Count == 4) {

                        using (MatOfPoint roiPointMat = new MatOfPoint (roiPointList.ToArray ())) {
                            roiRect = Imgproc.boundingRect (roiPointMat);
                        }

                        if (roiHistMat != null) {
                            roiHistMat.Dispose ();
                            roiHistMat = null;
                        }
                        roiHistMat = new Mat ();

                        using (Mat roiHSVMat = new Mat(hsvMat, roiRect))
                        using (Mat maskMat = new Mat ()) {
                            Imgproc.calcHist (new List<Mat> (new Mat[]{roiHSVMat}), new MatOfInt (0), maskMat, roiHistMat, new MatOfInt (16), new MatOfFloat (0, 180)); 
                            Core.normalize (roiHistMat, roiHistMat, 0, 255, Core.NORM_MINMAX);
                            //														Debug.Log ("roiHist " + roiHistMat.ToString ());
                        }
                    }
                }


                ///////////////////////////////////////////////////////////////
                // Draw the an image that displays where the user has pressed
                ///////////////////////////////////////////////////////////////
                if (points.Length < 4) {
                    for (int i = 0; i < points.Length; i++) {
                        Core.circle (rgbaMat, points [i], 6, new Scalar (0, 0, 255, 255), 2);
                    }
                } else {
                    for (int i = 0; i < 4; i++) {
                        Core.line (rgbaMat, points [i], points [(i + 1) % 4], new Scalar (255, 0, 0, 255), 2);
                    }
                    Core.rectangle (rgbaMat, roiRect.tl (), roiRect.br (), new Scalar (0, 255, 0, 255), 2);
                }

                Core.putText (rgbaMat, "PLEASE TOUCH 4 POINTS", new Point (5, rgbaMat.rows () - 10), Core.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar (255, 255, 255, 255), 2, Core.LINE_AA, false);
//				Imgproc.putText (rgbaMat, "W:" + rgbaMat.width () + " H:" + rgbaMat.height () + " SO:" + Screen.orientation, new Point (5, rgbaMat.rows () - 10), Core.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar (255, 255, 255, 255), 2, Imgproc.LINE_AA, false);
                Utils.matToTexture2D (rgbaMat, texture, colors);
            }

        }

        /// <summary>
        /// Raises the disable event.
        /// </summary>
        void OnDisable ()
        {
            webCamTextureToMatHelper.Dispose ();
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