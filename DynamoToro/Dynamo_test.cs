using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Net;
using System.Net.Sockets;
using ABB.Robotics;
using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.Discovery;
using ABB.Robotics.Controllers.RapidDomain;
using Autodesk.DesignScript.Geometry;
using Autodesk.DesignScript.Runtime;




namespace Dynamo_TORO
{
    internal class Dynamo_test
    {
        /*
        [STAThread]
        public static void robotListener(string[] controllerData, int port)
        {
            IPEndPoint endPoint = new IPEndPoint(IPAddress.Parse(controllerData[1]), port);
            TcpListener listener = new TcpListener(endPoint);
            Socket socket = null;

            listener.Start();
            Console.WriteLine("Local end point: {0}", listener.LocalEndpoint);
            socket = listener.AcceptSocket();
            Console.WriteLine("Connection accepted from " + socket.RemoteEndPoint);

            byte[] data = new byte[256];
            int len = socket.Receive(data);
            Console.WriteLine("Received...");




            TcpClient client = listener.AcceptTcpClient();
            Console.WriteLine("Connection accepted");

            NetworkStream stream = client.GetStream();
            String response = "Connection has been accepted";
            Byte[] sendTheseBytes = Encoding.ASCII.GetBytes(response);

            stream.Write(sendTheseBytes, 0, sendTheseBytes.Length);


            client.Close();
            listener.Stop();
        }


        public static RobTarget RobTargetAtPoint(Point point, double q1, double q2, double q3, double q4)
        {
            var target = new RobTarget();
            if (point != null)
            {
                target.FillFromString2(
                    string.Format(
                        "[[{0},{1},{2}],[{3},{4},{5},{6}],[0,0,0,0],[9.999999999E09,9.999999999E09,9.999999999E09,9.999999999E09,9.999999999E09,9.999999999E09]];",
                        point.X, point.Y, point.Z, q1, q2, q3, q4));
            }
            return target;
        }
        */



        public static ToolData toolData(float x, float y, float z, float q1, float q2, float q3, float q4, float load, float cog_x, float cog_y, float cog_z)
        {
            var t = new ToolData();
            t.Tframe.Trans.X = x;
            t.Tframe.Trans.Y = y;
            t.Tframe.Trans.Z = z;
            t.Tframe.Rot.Q1 = q1;
            t.Tframe.Rot.Q2 = q2;
            t.Tframe.Rot.Q3 = q3;
            t.Tframe.Rot.Q4 = q4;
            t.Tload.Mass = load;
            t.Tload.Cog.X = cog_x;
            t.Tload.Cog.Y = cog_y;
            t.Tload.Cog.Z = cog_z;
            return t;
        }


        public static List<string> programDataFormatter(List<object[]> programData)
        {
            List<string> dataOut = new List<string>();
            foreach (object[] group in programData)
            {
                string type = group[0].ToString();
                switch (type)
                {
                    case "RobTarget":
                        string result = string.Format("data = {0}", group[1]);
                        dataOut.Add(result);
                        break;
                    case "JointTarget":
                        result = string.Format("data = {0}", group[1]);
                        dataOut.Add(result);
                        break;
                    case "ToolData":
                        result = string.Format("data = {0}", group[1]);
                        dataOut.Add(result);
                        break;
                    case "WobjData":
                        result = string.Format("data = {0}", group[1]);
                        dataOut.Add(result);
                        break;
                }
            }
            return dataOut;
        }

        

        







    }
}
