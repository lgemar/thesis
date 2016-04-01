using UnityEngine;
using System.Collections;
using System.IO.Ports;

public class ComPort : MonoBehaviour
{

    SerialPort sp;
    

    public ComPort(string comPort, int BaudRate)
    {
        sp = new SerialPort(comPort, 9600);
    }

    public void Open()
    {
        try
        {
            sp.Open();
            sp.ReadTimeout = 25;
            sp.Handshake = Handshake.None;
            Debug.Log("Port Opened!");
        }
        catch (System.SystemException e)
        {
            Debug.Log("Error opening = " + e.Message);
        }
    }

    // Returns true if it read in N elements from the data stream
        // Returns true if it read in N elements from the data stream
    public bool ReadData(ref string[] stringData, ref float[] floatData, int N)
    {
        int numBytes = 0;
        if ((sp != null) && (sp.IsOpen))
        {
            byte tmp;
            string data = "";
            tmp = (byte)sp.ReadByte();
            numBytes += 1; 
            while (tmp != 255 && tmp != '\n')
            {
                // Debug.Log(data.Length);
                data += (char)(tmp);
                tmp = (byte)sp.ReadByte();
                numBytes += 1; 
            }
            string[] dataVector = data.Split(',');
            if (dataVector.Length == N)
            {
                for (int i = 0; i < N; i++)
                {
                    stringData[i] = dataVector[i];
                }
                for (int i = 0; i < N; i++)
                {
                    floatData[i] = float.Parse(stringData[i]);
                }
                return true;
            }
            // Debug.Log("Read in " + numBytes + " bytes.");
        }
        return false;
    }


    public void Close()
    {
        sp.Close();
    }
}
