using UnityEngine;
using System.Collections;
using System.IO.Ports;

using System.Net.Sockets;
using System.Net;


public class Master : MonoBehaviour {

    TcpClient x = new TcpClient();
    NetworkStream xStream;

	// Use this for initialization
	void Start () {
        x.Connect("localhost", 30000);
	}

	
	// Update is called once per frame
	void Update () {

        if( x.Connected )
        {
            xStream = x.GetStream();
            byte[] myWriteBuffer = System.Text.Encoding.ASCII.GetBytes("Are you receiving this message?");
            // xStream.Write(myWriteBuffer, 0, myWriteBuffer.Length);

            double[] floatArray1 = new double[] { 123.45, 123, 45, 1.2, 34.5 };
            string str = "";
            for(int i = 0; i < floatArray1.Length; i++)
            {
                if(i == 0)
                {
                    str += floatArray1[0].ToString();
                    i++;
                }
                if( i < floatArray1.Length)
                {
                    str += "\t";
                    str += floatArray1[i].ToString();
                }
            }
            str += "\n";
            // create a byte array and copy the floats into it...
            Debug.Log(str);
            byte[] myWriteBuffer2 = System.Text.Encoding.ASCII.GetBytes(str);
            xStream.Write(myWriteBuffer2, 0, myWriteBuffer2.Length);
            xStream.Flush();
        }
	}

    static byte[] GetBytes(string str)
    {
        byte[] bytes = new byte[str.Length * sizeof(char)];
        System.Buffer.BlockCopy(str.ToCharArray(), 0, bytes, 0, bytes.Length);
        return bytes;
    }

    static string GetString(byte[] bytes)
    {
        char[] chars = new char[bytes.Length / sizeof(char)];
        System.Buffer.BlockCopy(bytes, 0, chars, 0, bytes.Length);
        return new string(chars);
    }

    void OnDestroy()
    {
        // Nothing for now
    }
}


