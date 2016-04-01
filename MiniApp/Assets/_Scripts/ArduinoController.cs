using UnityEngine;
using System.Collections;
using System.IO.Ports;

public class ArduinoController : MonoBehaviour {

    public SerialPort serial = new SerialPort("COM5", 9600);
    private bool lightState = false;

    public void OnMouseDown()
    {
        if(serial.IsOpen == false)
        {
            serial.Open();
        }
        if(lightState == false)
        {
            serial.Write("A");
            lightState = true;
        }
        else {
            serial.Write("B");
            lightState = false;
        }
    }

	// Use this for initialization
	void Start () {
	}
	
	// Update is called once per frame
	void Update () {

        if(lightState == false)
        {
            gameObject.GetComponent<Renderer>().material.color = Color.red;
        }
        else {
            gameObject.GetComponent<Renderer>().material.color = Color.green;
        }
	
	}
}
