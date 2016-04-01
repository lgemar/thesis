using UnityEngine;
using System.Collections;
using UnityEngine.SceneManagement;

namespace OpenCVForUnitySample
{
    public class OpenCVForUnitySample : MonoBehaviour
    {

        // Use this for initialization
        void Start ()
        {

        }

        // Update is called once per frame
        void Update ()
        {

        }

        public void OnCameraCalibration ()
        {
            SceneManager.LoadScene ("CameraCalibration");
        }

        public void OnCam3DTracker ()
        {
            SceneManager.LoadScene ("Cam3DTracker");
        }

        public void OnOrientationTest ()
        {
            SceneManager.LoadScene ("OrientationTest");
        }

        public void OnShowLicenseButton ()
        {
            SceneManager.LoadScene ("ShowLicense");
        }

        public void OnTexture2DToMatSample ()
        {
            SceneManager.LoadScene ("Texture2DToMatSample");
        }

        public void OnThresholdSample ()
        {
            SceneManager.LoadScene ("ThresholdSample");
        }

        public void OnDrawingSample ()
        {
            SceneManager.LoadScene ("DrawingSample");
        }

        public void OnConvexHullSample ()
        {
            SceneManager.LoadScene ("ConvexHullSample");
        }

        public void OnHoughLinesPSample ()
        {
            SceneManager.LoadScene ("HoughLinesPSample");
        }

        public void OnFeature2DSample ()
        {
            SceneManager.LoadScene ("Feature2DSample");
        }

        public void OnWrapPerspectiveSample ()
        {
            SceneManager.LoadScene ("WrapPerspectiveSample");
        }

        public void OnFaceRecognizerSample ()
        {
            SceneManager.LoadScene ("FaceRecognizerSample");
        }

        public void OnDetectFaceSample ()
        {
            SceneManager.LoadScene ("DetectFaceSample");
        }
        
        public void OnWebCamTextureToMatSample ()
        {
            SceneManager.LoadScene ("WebCamTextureToMatSample");
        }
        
        public void OnWebCamTextureDetectFaceSample ()
        {
            SceneManager.LoadScene ("WebCamTextureDetectFaceSample");
        }

        public void OnWebCamTextureAsyncDetectFaceSample ()
        {
            SceneManager.LoadScene ("WebCamTextureAsyncDetectFaceSample");
        }

        public void OnOpticalFlowSample ()
        {
            SceneManager.LoadScene ("OpticalFlowSample");
        }

        public void OnComicFilterSample ()
        {
            SceneManager.LoadScene ("ComicFilterSample");
        }

        public void OnCamShiftSample ()
        {
            SceneManager.LoadScene ("CamShiftSample");
        }

        public void OnHandPoseEstimationSample ()
        {
            SceneManager.LoadScene ("HandPoseEstimationSample");
        }

        public void OnMultiObjectTrackingBasedOnColorSample ()
        {
            SceneManager.LoadScene ("MultiObjectTrackingBasedOnColorSample");
        }
    }
}
		