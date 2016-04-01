using UnityEngine;
using System.Collections;

public class Cam3DManager : MonoBehaviour {

    int buttonWidth = 200;
    int buttonHeight = 30;
    int padding = 5;

    GameObject[] allObjects;
    Camera mainCamera;
    GameObject Quad;
   

    void Start()
    {
        // Code from http://answers.unity3d.com/questions/329395/how-to-get-all-gameobjects-in-scene.html
        allObjects = FindObjectsOfType<GameObject>();
        mainCamera = FindObjectsOfType<Camera>()[0];
        Quad = GameObject.FindWithTag("Quad");
        deactivateObjects("VirtualEnvironment");
        deactivateObjects("Target");
        // deactivateObjects("Stick");
    }

    void OnGUI()
    {
        if(GUI.Button(new Rect(Screen.width - buttonWidth - padding, buttonHeight + padding, buttonWidth, buttonHeight), "Enter Virtual Environment")) {
            AppControl.control.virtualEnvironmentEnabled = true;
            turnOnVE();
        }
        if(GUI.Button(new Rect(Screen.width - buttonWidth - padding, 2 * buttonHeight + 2 * padding, buttonWidth, buttonHeight), "Exit Virtual Environment")) {
            AppControl.control.virtualEnvironmentEnabled = false;
            turnOffVE();
        }
        if(GUI.Button(new Rect(Screen.width - buttonWidth - padding, 3 * buttonHeight + 3 * padding, buttonWidth, buttonHeight), "Start Test")) {
            AppControl.control.beginTest();
            AppControl.control.acquiring = true;
        }
        if(GUI.Button(new Rect(Screen.width - buttonWidth - padding, 4 * buttonHeight + 4 * padding, buttonWidth, buttonHeight), "End Test")) {
            AppControl.control.acquiring = false;
            AppControl.control.endTest();
        }
    }

    void turnOnVE()
    {
        Quad.transform.position = new Vector3(-800, 300, 0);
        mainCamera.transform.position = new Vector3(0, 1, -800);
        mainCamera.fieldOfView = 60;
        deactivateObjects("ImageDisplay");
        activateObjects("VirtualEnvironment");
        activateObjects("Target");
        // activateObjects("Stick");
    }

    void turnOffVE()
    {
        Quad.transform.position = new Vector3(0, 0, 0);
        mainCamera.transform.position = new Vector3(0, 1, -400);
        mainCamera.fieldOfView = 60;
        activateObjects("ImageDisplay");
        deactivateObjects("VirtualEnvironment");
        deactivateObjects("Target");
        // deactivateObjects("Stick");
    }

    void activateObjects(string tag)
    {
        for(int i = 0; i < allObjects.Length; i++)
        {
            if(allObjects[i] != null)
            {
                if(allObjects[i].CompareTag(tag))
                {
                    allObjects[i].SetActive(true);
                }
            }
        }
    }

    void deactivateObjects(string tag)
    {
        for(int i = 0; i < allObjects.Length; i++)
        {
            if(allObjects[i] != null)
            {
                if(allObjects[i].CompareTag(tag))
                {
                    allObjects[i].SetActive(false);
                }
            }
        }

    }
}
