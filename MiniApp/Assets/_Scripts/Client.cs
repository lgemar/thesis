using UnityEngine;
using System.Collections;

using System.IO.Ports;

using System.Net.Sockets;
using System.Net;



public class Client2 : MonoBehaviour
{

    public string m_IPAdress = "127.0.0.1";
    public const int kPort = 30000;


    private static Client2 singleton;


    private Socket m_Socket;
    void Awake()
    {
        m_Socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);

        // System.Net.PHostEntry ipHostInfo = Dns.Resolve("host.contoso.com");
        // System.Net.IPAddress remoteIPAddress = ipHostInfo.AddressList[0];
        System.Net.IPAddress remoteIPAddress = System.Net.IPAddress.Parse(m_IPAdress);

        System.Net.IPEndPoint remoteEndPoint = new System.Net.IPEndPoint(remoteIPAddress, kPort);

        singleton = this;
        m_Socket.Connect(remoteEndPoint);
    }

    void OnApplicationQuit()
    {
        m_Socket.Close();
        m_Socket = null;
    }


    static public void Send(string msgData)
    {
        if (singleton.m_Socket == null)
            return;

        System.Text.UTF8Encoding encoding = new System.Text.UTF8Encoding();
        byte[] sendData = encoding.GetBytes(msgData);
        byte[] prefix = new byte[1];
        prefix[0] = (byte)sendData.Length;
        singleton.m_Socket.Send(prefix);
        singleton.m_Socket.Send(sendData);
    }

}


