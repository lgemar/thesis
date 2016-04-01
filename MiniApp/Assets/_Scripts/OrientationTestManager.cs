using UnityEngine;
using System.Collections;

public class OrientationTestManager : MonoBehaviour {

    int buttonWidth = 200;
    int buttonHeight = 30;
    int padding = 5;

    void Start()
    {
        //
    }

    void OnGUI()
    {
        if(GUI.Button(new Rect(0 + padding, buttonHeight + padding, buttonWidth, buttonHeight), "Back")) {
            UnityEngine.SceneManagement.SceneManager.LoadScene("OpenCVForUnitySample");
        }
    }
}
