using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text;
using ABB.Robotics;
using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.Discovery;
using ABB.Robotics.Controllers.RapidDomain;
using Autodesk.DesignScript.Geometry;
using Autodesk.DesignScript.Runtime;



//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////


//implemented CreateRapid based on logic from the
//Dynamo_ABB utility function with contributions from Matt Jezyk and Mike Dewberry
//converted to DynamoToRobot utility function at Autodesk, Inc. Waltham

//implemented PlaneToQuaternian based on logic from the 
//Design Robotics Group @ Harvard Gsd with contributions from Sola Grantham, Anthony Kane, Nathan King, Jonathan Grinham, and others. 
//converted to Dynamo_ABB utility function at the Virginia Tech Robot Summit


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////



namespace Dynamo_toRo
{


    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////


    public class DataTypes
    {


        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////

        /// <summary>
        /// Create robot target from coordinate and quaternion values.
        /// </summary>
        /// <param name="ptX">Coordinate</param>
        /// <param name="ptY">Coordinate</param>
        /// <param name="ptZ">Coordinate</param>
        /// <param name="q1">Quaternion</param>
        /// <param name="q2">Quaternion</param>
        /// <param name="q3">Quaternion</param>
        /// <param name="q4">Quaternion</param>
        /// <returns></returns>
        public static RobTarget RobTargetAtVals(double ptX, double ptY, double ptZ, double q1, double q2, double q3, double q4)
        {
            var target = new RobTarget();
            {
                target.FillFromString2(
                    string.Format(
                        "[[{0},{1},{2}],[{3},{4},{5},{6}],[0,0,0,0],[9.999999999E09,9.999999999E09,9.999999999E09,9.999999999E09,9.999999999E09,9.999999999E09]];",
                        ptX, ptY, ptZ, q1, q2, q3, q4));
            }
            return target;
        }

        /// <summary>
        /// Create robot target from point and quaternion values.
        /// </summary>
        /// <param name="point">Point</param>
        /// <param name="q1">Quaternion</param>
        /// <param name="q2">Quaternion</param>
        /// <param name="q3">Quaternion</param>
        /// <param name="q4">Quaternion</param>
        /// <returns></returns>
        public static RobTarget RobTargetAtPoint0(Point point, double q1, double q2, double q3, double q4)
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

        /// <summary>
        /// Create robot target from point and list of quaternion values.
        /// </summary>
        /// <param name="point">Point</param>
        /// <param name="quatList">List of quaternions</param>
        /// <returns></returns>
        public static RobTarget RobTargetAtPoint1(Point point, List<double> quatList)
        {
            var target = new RobTarget();
            if (point != null)
            {
                target.FillFromString2(
                    string.Format(
                        "[[{0},{1},{2}],[{3},{4},{5},{6}],[0,0,0,0],[9.999999999E09,9.999999999E09,9.999999999E09,9.999999999E09,9.999999999E09,9.999999999E09]];",
                        point.X, point.Y, point.Z, quatList[0], quatList[1], quatList[2], quatList[3]));
            }
            return target;
        }

        /// <summary>
        /// Create robot target from plane.
        /// </summary>
        /// <param name="plane">Plane</param>
        /// <returns></returns>
        public static RobTarget RobTargetAtPlane(Plane plane)
        {
            var target = new RobTarget();
            if (plane != null)
            {
                List<double> quatDoubles = RobotUtils.PlaneToQuaternian(plane);
                target.FillFromString2(
                    string.Format(
                        "[[{0},{1},{2}],[{3},{4},{5},{6}],[0,0,0,0],[9.999999999E09,9.999999999E09,9.999999999E09,9.999999999E09,9.999999999E09,9.999999999E09]];",
                        plane.Origin.X, plane.Origin.Y, plane.Origin.Z, quatDoubles[0], quatDoubles[1], quatDoubles[2], quatDoubles[3]));
            }
            return target;
        }

        /// <summary>
        /// Create joint target from rotational values per axis.
        /// </summary>
        /// <param name="j1">Degrees</param>
        /// <param name="j2">Degrees</param>
        /// <param name="j3">Degrees</param>
        /// <param name="j4">Degrees</param>
        /// <param name="j5">Degrees</param>
        /// <param name="j6">Degrees</param>
        /// <returns></returns>
        public static JointTarget JointTargetAtVals(double j1, double j2, double j3, double j4, double j5, double j6)
        {
            var target = new JointTarget();
            target.FillFromString2(
                string.Format("[[{0},{1},{2},{3},{4},{5}],[9.999999999E09,9.999999999E09,9.999999999E09,9.999999999E09,9.999999999E09,9.999999999E09]];",
                j1, j2, j3, j4, j5, j6));
            return target;
        }

        /// <summary>
        /// Create joint target from list of rotational values.
        /// </summary>
        /// <param name="joints">List of degrees</param>
        /// <returns></returns>
        public static JointTarget JointTargetAtList(List<double> joints)
        {
            var target = new JointTarget();
            if (joints != null)
            {
                target.FillFromString2(
                    string.Format("[[{0},{1},{2},{3},{4},{5}],[9.999999999E09,9.999999999E09,9.999999999E09,9.999999999E09,9.999999999E09,9.999999999E09]];",
                    joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]));
            }
            return target;
        }

        /// <summary>
        /// Create robot Pose from coordinate and quaternion values.
        /// </summary>
        /// <param name="ptX">Coordinate</param>
        /// <param name="ptY">Coordinate</param>
        /// <param name="ptZ">Coordinate</param>
        /// <param name="q1">Quaternion</param>
        /// <param name="q2">Quaternion</param>
        /// <param name="q3">Quaternion</param>
        /// <param name="q4">Quaternion</param>
        /// <returns></returns>
        public static Pose RobotPoseAtVals(double ptX, double ptY, double ptZ, double q1, double q2, double q3, double q4)
        {
            var r = new Pose();
            r.Trans.X = (float)ptX;
            r.Trans.Y = (float)ptY;
            r.Trans.Z = (float)ptZ;
            r.Rot.Q1 = (float)q1;
            r.Rot.Q2 = (float)q2;
            r.Rot.Q3 = (float)q3;
            r.Rot.Q4 = (float)q4;
            return r;
        }

        /// <summary>
        /// Create robot Pose from point and quaternion values.
        /// </summary>
        /// <param name="translation">Point</param>
        /// <param name="q1">Quaternion</param>
        /// <param name="q2">Quaternion</param>
        /// <param name="q3">Quaternion</param>
        /// <param name="q4">Quaternion</param>
        /// <returns></returns>
        public static Pose RobotPoseAtPoint0(Point translation, double q1, double q2, double q3, double q4)
        {
            var r = new Pose();
            r.Trans.X = (float)translation.X;
            r.Trans.Y = (float)translation.Y;
            r.Trans.Z = (float)translation.Z;
            r.Rot.Q1 = (float)q1;
            r.Rot.Q2 = (float)q2;
            r.Rot.Q3 = (float)q3;
            r.Rot.Q4 = (float)q4;
            return r;
        }

        /// <summary>
        /// Create robot Pose from point and list of quaternion values.
        /// </summary>
        /// <param name="translation">Point</param>
        /// <param name="quatDoubles">List of quaternions</param>
        /// <returns></returns>
        public static Pose RobotPoseAtPoint1(Point translation, List<double> quatDoubles)
        {
            var r = new Pose();
            r.Trans.X = (float)translation.X;
            r.Trans.Y = (float)translation.Y;
            r.Trans.Z = (float)translation.Z;
            r.Rot.Q1 = (float)quatDoubles[0];
            r.Rot.Q2 = (float)quatDoubles[1];
            r.Rot.Q3 = (float)quatDoubles[2];
            r.Rot.Q4 = (float)quatDoubles[3];
            return r;
        }

        /// <summary>
        /// Create robot Pose from plane.
        /// </summary>
        /// <param name="plane">Plane</param>
        /// <returns></returns>
        public static Pose RobotPoseAtPlane(Plane plane)
        {
            var r = new Pose();
            r.Trans.X = (float)plane.Origin.X;
            r.Trans.Y = (float)plane.Origin.Y;
            r.Trans.Z = (float)plane.Origin.Z;
            List<double> quatDoubles = RobotUtils.PlaneToQuaternian(plane);
            r.Rot.Q1 = (float)quatDoubles[0];
            r.Rot.Q2 = (float)quatDoubles[1];
            r.Rot.Q3 = (float)quatDoubles[2];
            r.Rot.Q4 = (float)quatDoubles[3];
            return r;
        }

        /// <summary>
        /// Define speeddata.
        /// </summary>
        /// <param name="varName">Name of speeddata variable</param>
        /// <param name="v_tcp">Speed at tool center point in mm/s</param>
        /// <param name="v_ori">Reorientation speed of the TCP in deg</param>
        /// <param name="v_leax">Linear speed of external axes in mm/s</param>
        /// <param name="v_reax">Rotational speed of external axes in deg</param>
        /// <returns></returns>
        public static string Speeddata(string varName, double v_tcp, double v_ori, double v_leax, double v_reax)
        {
            string speed = string.Format("\n\tVAR speeddata {0}:=[{1},{2},{3},{4}];", varName, v_tcp, v_ori, v_leax, v_reax);
            return speed;
        }

        /// <summary>
        /// Define zonedata.
        /// </summary>
        /// <param name="varName">Name of zonedata variable</param>
        /// <param name="pfine">T: stop-point | F: fly-by-point</param>
        /// <param name="pzone_tcp">Radius for TCP</param>
        /// <param name="pzone_ori">Radius for reorientation</param>
        /// <param name="pzone_eax">Radius for external axes</param>
        /// <param name="zone_ori">Radius for tool reorientation in deg</param>
        /// <param name="zone_leax">Radius for linear external axes in mm</param>
        /// <param name="zone_reax">Radius for rotating external axes in deg</param>
        /// <returns></returns>
        public static string Zonedata(string varName, bool pfine, double pzone_tcp, double pzone_ori, double pzone_eax, double zone_ori, double zone_leax, double zone_reax)
        {
            string zone = string.Format("\n\tVAR zonedata {0}:=[{1},{2},{3},{4},{5},{6},{7}];", varName, pfine, pzone_tcp, pzone_ori, pzone_eax, zone_ori, zone_leax, zone_reax);
            return zone;
        }

        /// <summary>
        /// Define loaddata.
        /// </summary>
        /// <param name="varName">Name of loaddata variable</param>
        /// <param name="cog_x">Center of gravity coordinate</param>
        /// <param name="cog_y">Center of gravity coordinate</param>
        /// <param name="cog_z">Center of gravity coordinate</param>
        /// <param name="aom_q1">Axes of moment quaternion</param>
        /// <param name="aom_q2">Axes of moment quaternion</param>
        /// <param name="aom_q3">Axes of moment quaternion</param>
        /// <param name="aom_q4">Axes of moment quaternion</param>
        /// <param name="ix">Inertia in kgm^2</param>
        /// <param name="iy">Inertia in kgm^2</param>
        /// <param name="iz">Inertia in kgm^2</param>
        /// <returns></returns>
        public static string Loaddata(string varName, double load, double cog_x, double cog_y, double cog_z, double aom_q1, double aom_q2, double aom_q3, double aom_q4, double ix, double iy, double iz)
        {
            string loadData = string.Format("\n\tPERS loaddata {0}:=[{1},[{2},{3},{4}],[{5},{6},{7},{8}],{9},{10},{11}];", varName, load, cog_x, cog_y, cog_z, aom_q1, aom_q2, aom_q3, aom_q4, ix, iy, iz);
            return loadData;
        }

        /// <summary>
        /// Define confdata.
        /// </summary>
        /// <param name="varName">Name of confdata variable</param>
        /// <param name="cf1">Current quadrant or meter interval of axis 1</param>
        /// <param name="cf4">Current quadrant or meter interval of axis 4</param>
        /// <param name="cf6">Current quadrant or meter interval of axis 6</param>
        /// <param name="cfx">Current quadrant or meter interval of axis 2 | X</param>
        /// <returns></returns>
        public static string Confdata(string varName, double cf1, double cf4, double cf6, double cfx)
        {
            string conf = string.Format("\n\tPERS confdata {0}:=[{1},{2},{3},{4}];", varName, cf1, cf4, cf6, cfx);
            return conf;
        }

        /// <summary>
        /// Define motion set data.
        /// </summary>
        /// <param name="varName">Name of motsetdata variable</param>
        /// <param name="vel_oride">Velocity as a percentage of programmed velocity.</param>
        /// <param name="vel_max">Maximum velocity in mm/s.</param>
        /// <param name="acc_acc">Acceleration and deceleration as a percentage of the normal values.</param>
        /// <param name="acc_ramp">The rate by which acceleration and deceleration increases as a percentage of the normal values. </param>
        /// <param name="sing_wrist">The orientation of the tool is allowed to deviate somewhat in order to prevent wrist singularity. </param>
        /// <param name="sing_arm">The orientation of the tool is allowed to deviate somewhat in order to prevent arm singularity (not implemented).</param>
        /// <param name="sing_base">The orientation of the tool is not allowed to deviate. </param>
        /// <param name="conf_jsup">Supervision of joint configuration is active during joint movement. </param>
        /// <param name="conf_lsup">Supervision of joint configuration is active during linear and circular movement. </param>
        /// <param name="conf_ax1">Maximum permitted deviation in degrees for axis 1 (not used in this version). </param>
        /// <param name="conf_ax4">Maximum permitted deviation in degrees for axis 4 (not used in this version). </param>
        /// <param name="conf_ax6">Maximum permitted deviation in degrees for axis 6 (not used in this version). </param>
        /// <param name="pathresol">Current override in percentage of the configured path resolution.</param>
        /// <param name="motionsup">Mirror RAPID status (TRUE = On and FALSE = Off) of motion supervision function.</param>
        /// <param name="tunevalue">Current RAPID override as a percentage of the configured tunevalue for the motion supervision function.</param>
        /// <param name="acclim">Limitation of tool acceleration along the path. (TRUE = limitation and FALSE = no limitation).</param>
        /// <param name="accmax">TCP acceleration limitation in m/s2. If acclim is FALSE, the value is always set to -1.</param>
        /// <param name="decellim">Limitation of tool deceleration along the path. (TRUE = limitation and FALSE = no limitation).</param>
        /// <param name="decelmax">TCP deceleration limitation in m/s2. If decellim is FALSE, the value is always set to -1.</param>
        /// <param name="cirpathreori">Tool reorientation during circle path: 0 = Interpolation in path frame; 1 = Interpolation in object frame; 2 = Programmed tool orientation in CirPoint</param>
        /// <param name="worldacclim">Limitation of acceleration in world coordinate system. (TRUE = limitation and FALSE = no limitation).</param>
        /// <param name="worldaccmax">Limitation of acceleration in world coordinate system in m/s2. If worldacclim is FALSE, the value is always set to -1.</param>
        /// <param name="evtbufferact">Event buffer active or not active. (TRUE = event buffer active and FALSE = event buffer not active).</param>
        /// <returns></returns>
        public static string Motsetdata(string varName, double vel_oride, double vel_max, double acc_acc, double acc_ramp, bool sing_wrist, bool sing_arm, bool sing_base, bool conf_jsup, bool conf_lsup, double conf_ax1, double conf_ax4, double conf_ax6, double pathresol, bool motionsup, double tunevalue, bool acclim, double accmax, bool decellim, double decelmax, int cirpathreori, bool worldacclim, double worldaccmax, bool evtbufferact)
        {
            string motset = string.Format("\n\tVAR motsetdata {0}:=" + "[{1},{2}],\n" +
                                                                        "[{3},{4}],\n" +
                                                                        "[{5},{6},{7}],\n" +
                                                                        "[{8},{9},{10},{11},{12},{13}],\n" +
                                                                        "{14},\n" +
                                                                        "{15},\n" +
                                                                        "{16},\n" +
                                                                        "{17},\n" +
                                                                        "{18},\n" +
                                                                        "{19},\n" +
                                                                        "{20},\n" +
                                                                        "{21},\n" +
                                                                        "{22},\n" +
                                                                        "{23},\n" +
                                                                        "{24},\n",
                                                                        varName,
                                                                        vel_oride, vel_max,
                                                                        acc_acc, acc_ramp,
                                                                        sing_wrist, sing_arm, sing_base,
                                                                        conf_jsup, conf_lsup, conf_ax1, conf_ax4, conf_ax6,
                                                                        pathresol,
                                                                        motionsup,
                                                                        tunevalue,
                                                                        acclim,
                                                                        accmax,
                                                                        decellim,
                                                                        decelmax,
                                                                        cirpathreori,
                                                                        worldacclim,
                                                                        worldaccmax,
                                                                        evtbufferact);
            return motset;
        }

        /// <summary>
        /// Define stoppointdata
        /// </summary>
        /// <param name="varName">Name of stoppointdata variable</param>
        /// <param name="progsynch">Sychronization with RAPID program execution</param>
        /// <param name="type">1 = inpos; 2 = stoptime; 3 = followtime</param>
        /// <param name="inpos_position">Position condition for TCP</param>
        /// <param name="inpos_speed">Speed condition for TCP</param>
        /// <param name="inpos_mintime">Minimum wait time</param>
        /// <param name="inpos_maxtime">Maximum wait time</param>
        /// <param name="stoptime">Time stopped</param>
        /// <param name="followtime">Follow time</param>
        /// <returns></returns>
        public static string Stoppointdata(string varName, bool progsynch, string type, int inpos_position, int inpos_speed, double inpos_mintime, double inpos_maxtime, double stoptime, double followtime)
        {
            string stop = string.Format("\n\tVAR stoppointdata {0}:= [{1},{2},[{3},{4},{5},{6}],{7},{8},'',0,0];", varName);
            return stop;
        }

        /// <summary>
        /// Define shapedata and create work-zone instruction for box.
        /// </summary>
        /// <param name="varName">Name of shapedata variable</param>
        /// <param name="Inside_Outside">Define as volume "Inside" | "Outside"</param>
        /// <param name="lo_x">Low point coordinate</param>
        /// <param name="lo_y">Low point coordinate</param>
        /// <param name="lo_z">Low point coordinate</param>
        /// <param name="hi_x">High point coordinate</param>
        /// <param name="hi_y">High point coordinate</param>
        /// <param name="hi_z">High point coordinate</param>
        /// <returns></returns>
        [MultiReturn(new[] { "cnst", "inst" })]
        public static Dictionary<string, string> WZBoxDef(string varName, string Inside_Outside, double lo_x, double lo_y, double lo_z, double hi_x, double hi_y, double hi_z)
        {
            string cnst = string.Format("\n\tVAR shapedata {0};" +
                                        "\n\tCONST pos lo{0}:=[{1},{2},{3}];" +
                                        "\n\tCONST pos hi{0}:=[{4},{5},{6}];",
                                        varName, lo_x, lo_y, lo_z, hi_x, hi_y, hi_z);
            string inst = string.Format("WZBoxDef \\{0},{1},l{1},h{1};",
                                        Inside_Outside, varName);

            return new Dictionary<string, string>
            {
                {"cnst", cnst},
                {"inst", inst},
             };
        }

        /// <summary>
        /// Define shapedata and create work-zone instruction for cylinder.
        /// </summary>
        /// <param name="varName">Name of shapedata variable</param>
        /// <param name="Inside_Outside">Define as volume "Inside" | "Outside"</param>
        /// <param name="center_x">Coordinate</param>
        /// <param name="center_y">Coordinate</param>
        /// <param name="center_z">Coordinate</param>
        /// <param name="radius">Radius</param>
        /// <param name="height">Height</param>
        /// <returns></returns>
        [MultiReturn(new[] { "cnst", "inst" })]
        public static Dictionary<string, string> WZCylDef(string varName, string Inside_Outside, double center_x, double center_y, double center_z, double radius, double height)
        {
            string cnst = string.Format("\n\tVAR shapedata {0};" +
                                        "\n\tCONST pos c{0}:=[{1},{2},{3}];" +
                                        "\n\tCONST num r{0}:={4};" +
                                        "\n\tCONST num h{0}:={5};",
                                        varName, center_x, center_y, center_z, radius, height);
            string inst = string.Format("WZBoxDef \\{0},{1},c{1},r{1},h{1};",
                                        Inside_Outside, varName);

            return new Dictionary<string, string>
            {
                {"cnst", cnst},
                {"inst", inst},
             };
        }

        /// <summary>
        /// Define shapedata and create work-zone instruction for sphere.
        /// </summary>
        /// <param name="varName">Name of shapedata variable</param>
        /// <param name="Inside_Outside">Define as volume "Inside" | "Outside"</param>
        /// <param name="center_x">Coordinate</param>
        /// <param name="center_y">Coordinate</param>
        /// <param name="center_z">Coordinate</param>
        /// <param name="radius">Radius</param>
        /// <returns></returns>
        [MultiReturn(new[] { "cnst", "inst" })]
        public static Dictionary<string, string> WZSphDef(string varName, string Inside_Outside, double center_x, double center_y, double center_z, double radius)
        {
            string cnst = string.Format("\n\tVAR shapedata {0};" +
                                        "\n\tCONST pos c{0}:=[{1},{2},{3}];" +
                                        "\n\tCONST num r{0}:={4};",
                                        varName, center_x, center_y, center_z, radius);
            string inst = string.Format("WZBoxDef \\{0},{1},c{1},r{1};",
                                        Inside_Outside, varName);

            return new Dictionary<string, string>
            {
                {"cnst", cnst},
                {"inst", inst},
             };
        }

        /// <summary>
        /// Define joint-targets for joint limits.
        /// </summary>
        /// <param name="varName">Name of variables</param>
        /// <param name="Inside_Outside">Define as volume "Inside" | "Outside"</param>
        /// <param name="loJointVal">JointTarget</param>
        /// <param name="hiJointVal">JointTarget</param>
        /// <returns></returns>
        [MultiReturn(new[] { "cnst", "inst" })]
        public static Dictionary<string, string> WZLimJointDef(string varName, string Inside_Outside, JointTarget loJointVal, JointTarget hiJointVal)
        {
            string cnst = string.Format("\n\tVAR wzstationary wl{0};" +
                                        "\n\tVAR shapedata js{0};" +
                                        "\n\tCONST jointtarget lo{0}:={1};" +
                                        "\n\tCONST jointtarget hi{0}:={2};",
                                        varName, loJointVal, hiJointVal);
            string inst = string.Format("WZLimJointDef \\{0},js{1},lo{1},hi{1};" +
                                        "WzLimSup \\Stat, wl{1},js{1};",
                                        Inside_Outside, varName);

            return new Dictionary<string, string>
            {
                {"cnst", cnst},
                {"inst", inst},
             };
        }

        /// <summary>
        /// Define tooldata from coordinate, quaternion, and load values.
        /// </summary>
        /// <param name="x">Coordinate</param>
        /// <param name="y">Coordinate</param>
        /// <param name="z">Coordinate</param>
        /// <param name="q1">Quaternion</param>
        /// <param name="q2">Quaternion</param>
        /// <param name="q3">Quaternion</param>
        /// <param name="q4">Quaternion</param>
        /// <param name="load">Load in kg</param>
        /// <param name="name">Name of tooldata variable</param>
        /// <returns></returns>
        public static List<string> ToolAtVals(double x, double y, double z, double q1, double q2, double q3, double q4, object load, string name)
        {
            List<string> toolData = new List<string>();
            string tool = string.Format("\tPERS tooldata {0}:=[TRUE,[[{1},{2},{3}],[{4},{5},{6},{7}]],[{8},[0,0,0.001],[1,0,0,0],0,0,0]];\n", name, x, y, z, q1, q2, q3, q4, load);
            toolData.Add(tool);
            return toolData;
        }

        /// <summary>
        /// Define tooldata from point, quaternion, and load values.
        /// </summary>
        /// <param name="pt">Point</param>
        /// <param name="q1">Quaternion</param>
        /// <param name="q2">Quaternion</param>
        /// <param name="q3">Quaternion</param>
        /// <param name="q4">Quaternion</param>
        /// <param name="load">Load in kg</param>
        /// <param name="name">Name of tooldata variable</param>
        /// <returns></returns>
        public static List<string> ToolAtPoint(Point pt, double q1, double q2, double q3, double q4, object load, string name)
        {
            List<string> toolData = new List<string>();
            string tool = string.Format("\tPERS tooldata {0}:=[TRUE,[[{1},{2},{3}],[{4},{5},{6},{7}]],[{8},[0,0,0.001],[1,0,0,0],0,0,0]];\n", name, pt.X, pt.Y, pt.Z, q1, q2, q3, q4, load);
            toolData.Add(tool);
            return toolData;
        }

        /// <summary>
        /// Define tooldata from plane and load value.
        /// </summary>
        /// <param name="pl">Plane</param>
        /// <param name="load">Load in kg</param>
        /// <param name="name">Name of tooldata variable</param>
        /// <returns></returns>
        public static List<string> ToolAtPlane(Plane pl, object load, string name)
        {
            List<string> toolData = new List<string>();
            List<double> quats = RobotUtils.PlaneToQuaternian(pl);
            string tool = string.Format("\tPERS tooldata {0}:=[TRUE,[[{1},{2},{3}],[{4},{5},{6},{7}]],[{8},[0,0,0.001],[1,0,0,0],0,0,0]];\n", name, pl.Origin.X, pl.Origin.Y, pl.Origin.Z, quats[0], quats[1], quats[2], quats[3], load);
            toolData.Add(tool);
            return toolData;
        }

        /// <summary>
        /// Define wobjdata from coordinate and quaternion values.
        /// </summary>
        /// <param name="x">Coordinate</param>
        /// <param name="y">Coordinate</param>
        /// <param name="z">Coordinate</param>
        /// <param name="q1">Quaternion</param>
        /// <param name="q2">Quaternion</param>
        /// <param name="q3">Quaternion</param>
        /// <param name="q4">Quaternion</param>
        /// <param name="name">Name of wobjdata variable</param>
        /// <returns></returns>
        public static List<string> WobjAtVals(double x, double y, double z, double q1, double q2, double q3, double q4, string name)
        {
            List<string> wobjData = new List<string>();
            string wobj = string.Format("\tTASK PERS wobjdata {0}:=[FALSE,TRUE," + @"""""" + ",[[{1},{2},{3}],[{4},{5},{6},{7}]],[[0,0,0],[1,0,0,0]]];\n", name, x, y, z, q1, q2, q3, q4);
            wobjData.Add(wobj);
            return wobjData;
        }

        /// <summary>
        /// Define wobjdata from point and quaternion values.
        /// </summary>
        /// <param name="pt">Point</param>
        /// <param name="q1">Quaternion</param>
        /// <param name="q2">Quaternion</param>
        /// <param name="q3">Quaternion</param>
        /// <param name="q4">Quaternion</param>
        /// <param name="name">Name of wobjdata variable</param>
        /// <returns></returns>
        public static List<string> WobjAtPoint(Point pt, double q1, double q2, double q3, double q4, string name)
        {
            List<string> wobjData = new List<string>();
            string wobj = string.Format("\tTASK PERS wobjdata {0}:=[FALSE,TRUE," + @"""""" + ",[[{1},{2},{3}],[{4},{5},{6},{7}]],[[0,0,0],[1,0,0,0]]];\n", name, pt.X, pt.Y, pt.Z, q1, q2, q3, q4);
            wobjData.Add(wobj);
            return wobjData;
        }

        /// <summary>
        /// Define wobjdata from plane.
        /// </summary>
        /// <param name="pl">Plane</param>
        /// <param name="name">Name of wobjdata variable</param>
        /// <returns></returns>
        public static List<string> WobjAtPlane(Plane pl, string name)
        {
            List<string> wobjData = new List<string>();
            List<double> quats = RobotUtils.PlaneToQuaternian(pl);
            string wobj = string.Format("\tTASK PERS wobjdata {0}:=[FALSE,TRUE," + @"""""" + ",[[{1},{2},{3}],[{4},{5},{6},{7}]],[[0,0,0],[1,0,0,0]]];\n", name, pl.Origin.X, pl.Origin.Y, pl.Origin.Z, quats[0], quats[1], quats[2], quats[3]);
            wobjData.Add(wobj);
            return wobjData;
        }


    }


    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////


    public class Utilities
    {


        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////

        /// <summary>
        /// Get individual quaternions from a plane.
        /// </summary>
        /// <param name="plane">The plane</param>
        /// <returns></returns>
        [MultiReturn(new[] { "q1", "q2", "q3", "q4" })]
        public static Dictionary<string, double> QuatAtPlane(Plane plane)
        {
            List<double> quats = new List<double>();

            if (plane != null)
            {
                List<double> quatDoubles = RobotUtils.PlaneToQuaternian(plane);
                quats.Add(quatDoubles[0]);
                quats.Add(quatDoubles[1]);
                quats.Add(quatDoubles[2]);
                quats.Add(quatDoubles[3]);
            }
            return new Dictionary<string, double>
        {
            {"q1", quats[0]},
            {"q2", quats[1]},
            {"q3", quats[2]},
            {"q4", quats[3]}
            };
        }

        /// <summary>
        /// Get list of quaternions from a plane.
        /// </summary>
        /// <param name="plane">The plane</param>
        /// <returns></returns>
        public static List<double> QuatListAtPlane(Plane plane)
        {
            List<double> quats = new List<double>();

            if (plane != null)
            {
                List<double> quatDoubles = RobotUtils.PlaneToQuaternian(plane);
                quats.Add(quatDoubles[0]);
                quats.Add(quatDoubles[1]);
                quats.Add(quatDoubles[2]);
                quats.Add(quatDoubles[3]);
            }
            return quats;
        }

        /// <summary>
        /// Insert an item into a list at specified index.
        /// </summary>
        /// <param name="list">Initial list</param>
        /// <param name="item">Item to insert</param>
        /// <param name="index">Index at which to insert</param>
        /// <returns></returns>
        public static List<object> InsertAtIndex(List<object> list, List<object> item, List<int> index)
        {
            int cnt = 0;
            foreach (var dex in index)
            {
                if (item.Count == cnt) { item.Add(item[0]); }
                if (dex <= list.Count + 1) { list.Insert(dex + cnt, item[cnt]); }
                if (dex > list.Count + 1) { list.Add(item[cnt]); }
                cnt++;
            }
            return list;
        }

        /// <summary>
        /// Until specified length, zero-pad a value at left.
        /// </summary>
        /// <param name="val">Initial value</param>
        /// <param name="numDigits">Total number of digits</param>
        /// <returns></returns>
        public static string ZeroPadLeft(double val, int numDigits)
        {
            string valStr = val.ToString().PadLeft(numDigits, '0');
            return valStr;
        }

        /// <summary>
        /// Until specified length, zero-pad a value at right.
        /// </summary>
        /// <param name="val">Initial value</param>
        /// <param name="numDigits">Total number of digits</param>
        /// <returns></returns>
        public static string ZeroPadRight(double val, int numDigits)
        {
            string valStr = val.ToString().PadRight(numDigits, '0');
            return valStr;
        }

        /// <summary>
        /// Create file at destination from data in Dynamo.
        /// </summary>
        /// <param name="filePath">Write to destination</param>
        /// <param name="robData">Data to write</param>
        /// <returns></returns>
        public static string DataToFile(string filePath, List<string> robData)
        {
            var dataBuilder = new StringBuilder();
            foreach (string data in robData) { dataBuilder.Append(data); }

            using (var lines = new StreamWriter(filePath, false))
            {
                lines.Write(dataBuilder.ToString());
                lines.Flush();
            }
            return filePath;
        }

        /// <summary>
        /// Read data in Dynamo from file at destination.
        /// </summary>
        /// <param name="filePath">Read from destination</param>
        /// <returns></returns>
        public static string FileToData(string filePath)
        {
            string robData;
            using (var data = new StreamReader(filePath, false))
            {
                robData = data.ReadToEnd();
            }
            return robData;
        }

    }


    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////


    public class Instructions
    {


        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////

        /// <summary>
        /// Create a linear movement instruction.
        /// </summary>
        /// <param name="targets">Robot target</param>
        /// <param name="speed">Speed data (rounds to default in RobotStudio)</param>
        /// <param name="zone">Zone data (rounds to default in RobotStudio)</param>
        /// <param name="setName">Unique name for this instruction</param>
        /// <param name="toolName">Active tool</param>
        /// <param name="wobjName">Active work-object</param>
        /// <returns></returns>
        [MultiReturn(new[] { "cnstList", "instList" })]
        public static Dictionary<string, List<string>> MoveL(List<RobTarget> targets, List<object> speed, List<object> zone, string setName, string toolName, string wobjName)
        {
            /// setup
            List<string> cnstList = new List<string>();
            List<string> instList = new List<string>();
            int cnt;

            /// target instructions
            cnt = 0;
            foreach (var target in targets)
            {
                if (cnt < targets.Count)
                {
                    if (cnt == speed.Count) { speed.Add(speed[0]); }
                    if (cnt == zone.Count) { zone.Add(zone[0]); }
                }
                if (speed[cnt] is int || speed[cnt] is double)
                {
                    speed[cnt] = RobotUtils.closestSpeed(Convert.ToDouble(speed[cnt]));
                    speed[cnt] = string.Format("v{0}", speed[cnt]);
                }
                if (zone[cnt] is int || zone[cnt] is double)
                {
                    zone[cnt] = RobotUtils.closestZone(Convert.ToDouble(zone[cnt]));
                    zone[cnt] = string.Format("z{0}", zone[cnt]);
                }

                cnstList.Add(string.Format("\n\tCONST robtarget {0}{1}:={2};", setName, cnt, target));
                instList.Add(string.Format("\n\t\tMoveL {0}{1},{2},{3},{4}\\WObj:={5};", setName, cnt, speed[cnt], zone[cnt], toolName, wobjName));
                cnt++;
            }

            ///end step
            return new Dictionary<string, List<string>>
            {
                {"cnstList", cnstList},
                {"instList", instList},
                };
        }

        /// <summary>
        /// Create a joint movement instruction.
        /// </summary>
        /// <param name="targets">Robot target</param>
        /// <param name="speed">Speed data (rounds to default in RobotStudio)</param>
        /// <param name="zone">Zone data (rounds to default in RobotStudio)</param>
        /// <param name="setName">Unique name of this instruction</param>
        /// <param name="toolName">Active tool</param>
        /// <param name="wobjName">Active work-object</param>
        /// <returns></returns>
        [MultiReturn(new[] { "cnstList", "instList" })]
        public static Dictionary<string, List<string>> MoveJ(List<RobTarget> targets, List<object> speed, List<object> zone, string setName, string toolName, string wobjName)
        {
            /// setup
            List<string> cnstList = new List<string>();
            List<string> instList = new List<string>();
            int cnt;

            /// target instructions
            cnt = 0;
            foreach (var target in targets)
            {
                if (cnt < targets.Count)
                {
                    if (cnt == speed.Count) { speed.Add(speed[0]); }
                    if (cnt == zone.Count) { zone.Add(zone[0]); }
                }
                if (speed[cnt] is int || speed[cnt] is double)
                {
                    speed[cnt] = RobotUtils.closestSpeed(Convert.ToDouble(speed[cnt]));
                    speed[cnt] = string.Format("v{0}", speed[cnt]);
                }
                if (zone[cnt] is int || zone[cnt] is double)
                {
                    zone[cnt] = RobotUtils.closestZone(Convert.ToDouble(zone[cnt]));
                    zone[cnt] = string.Format("z{0}", zone[cnt]);
                }

                cnstList.Add(string.Format("\n\tCONST robtarget {0}{1}:={2};", setName, cnt, target));
                instList.Add(string.Format("\n\t\tMoveJ {0}{1},{2},{3},{4}\\WObj:={5};", setName, cnt, speed[cnt], zone[cnt], toolName, wobjName));

                cnt++;
            }

            ///end step
            return new Dictionary<string, List<string>>
            {
                {"cnstList", cnstList},
                {"instList", instList},
                };
        }

        /// <summary>
        /// Create an absolute joint movement instruction.
        /// </summary>
        /// <param name="targets">Joint target</param>
        /// <param name="speed">Speed data (rounds to default in RobotStudio)</param>
        /// <param name="zone">Zone data (rounds to default in RobotStudio)</param>
        /// <param name="setName">Unique name for this instruction</param>
        /// <param name="toolName">Active tool</param>
        /// <param name="wobjName">Active work-object</param>
        /// <returns></returns>
        [MultiReturn(new[] { "cnstList", "instList" })]
        public static Dictionary<string, List<string>> MoveAbsJ(List<JointTarget> targets, List<object> speed, List<object> zone, string setName, string toolName, string wobjName)
        {
            /// setup
            List<string> cnstList = new List<string>();
            List<string> instList = new List<string>();
            int cnt;

            /// target instructions
            cnt = 0;
            foreach (var target in targets)
            {
                if (cnt < targets.Count)
                {
                    if (cnt == speed.Count) { speed.Add(speed[0]); }
                    if (cnt == zone.Count) { zone.Add(zone[0]); }
                }
                if (speed[cnt] is int || speed[cnt] is double)
                {
                    speed[cnt] = RobotUtils.closestSpeed(Convert.ToDouble(speed[cnt]));
                    speed[cnt] = string.Format("v{0}", speed[cnt]);
                }
                if (zone[cnt] is int || zone[cnt] is double)
                {
                    zone[cnt] = RobotUtils.closestZone(Convert.ToDouble(zone[cnt]));
                    zone[cnt] = string.Format("z{0}", zone[cnt]);
                }

                if (speed[cnt] is int)
                { speed[cnt] = string.Format("v{0}", speed[cnt]); }
                if (zone[cnt] is int)
                { zone[cnt] = string.Format("z{0}", zone[cnt]); }

                cnstList.Add(string.Format("\n\tCONST jointtarget {0}{1}:={2};", setName, cnt, target));
                instList.Add(string.Format("\n\t\tMoveAbsJ {0}{1},{2},{3},{4}\\WObj:={5};", setName, cnt, speed[cnt], zone[cnt], toolName, wobjName));

                cnt++;
            }

            ///end step
            return new Dictionary<string, List<string>>
            {
                {"cnstList", cnstList},
                {"instList", instList},
                };
        }

        /// <summary>
        /// Create a circular movement instruction.
        /// </summary>
        /// <param name="cirTarget">Robot target (through point)</param>
        /// <param name="toTarget">Robot target (destination)</param>
        /// <param name="speed">Speed data (rounds to default in RobotStudio)</param>
        /// <param name="zone">Zone data (rounds to default in RobotStudio)</param>
        /// <param name="setName">Unique name for this instruction</param>
        /// <param name="toolName">Active tool</param>
        /// <param name="wobjName">Active work-object</param>
        /// <returns></returns>
        [MultiReturn(new[] { "cnstList", "instList" })]
        public static Dictionary<string, List<string>> MoveC(List<RobTarget> cirTarget, List<RobTarget> toTarget, List<object> speed, List<object> zone, string setName, string toolName, string wobjName)
        {
            /// setup
            List<string> cnstList = new List<string>();
            List<string> instList = new List<string>();
            int cnt;

            /// target instructions
            cnt = 0;
            foreach (var target in toTarget)
            {
                if (cnt < toTarget.Count)
                {
                    if (cnt == speed.Count) { speed.Add(speed[0]); }
                    if (cnt == zone.Count) { zone.Add(zone[0]); }
                }
                if (speed[cnt] is int || speed[cnt] is double)
                {
                    speed[cnt] = RobotUtils.closestSpeed(Convert.ToDouble(speed[cnt]));
                    speed[cnt] = string.Format("v{0}", speed[cnt]);
                }
                if (zone[cnt] is int || zone[cnt] is double)
                {
                    zone[cnt] = RobotUtils.closestZone(Convert.ToDouble(zone[cnt]));
                    zone[cnt] = string.Format("z{0}", zone[cnt]);
                }

                cnstList.Add(string.Format("\n\tCONST robtarget cir{0}{1}:={2};" +
                                            "\n\tCONST robtarget to{0}{1}:={3};"
                                            , setName, cnt, cirTarget[cnt], target));
                instList.Add(string.Format("\n\t\tMoveC cir{0}{1}, to{0}{1}, {2},{3},{4}\\WObj:={5};", setName, cnt, speed[cnt], zone[cnt], toolName, wobjName));

                cnt++;
            }

            ///end step
            return new Dictionary<string, List<string>>
            {
                {"cnstList", cnstList},
                {"instList", instList},
                };
        }

        /// <summary>
        /// Create independent joint movement.
        /// </summary>
        /// <param name="mecUnit">Active mechanical unit</param>
        /// <param name="axisNum">Select axis (1-6)</param>
        /// <param name="axisPos">Amount rotation (deg)</param>
        /// <param name="speed">Speed data (rounds to default in RobotStudio)</param>
        /// <param name="setName">Unique name for this instruction</param>
        /// <returns></returns>
        [MultiReturn(new[] { "cnstList", "instList" })]
        public static Dictionary<string, List<string>> IndAMove(string mecUnit, List<int> axisNum, List<double> axisPos, List<object> speed, string setName)
        {
            /// setup
            List<string> cnstList = new List<string>();
            List<string> instList = new List<string>();
            int cnt;

            /// target instructions
            cnt = 0;
            foreach (var position in axisPos)
            {
                if (cnt < axisPos.Count)
                {
                    if (cnt == speed.Count) { speed.Add(speed[0]); }
                    if (cnt == axisNum.Count) { axisNum.Add(axisNum[0]); }
                }
                if (speed[cnt] is int || speed[cnt] is double)
                {
                    speed[cnt] = RobotUtils.closestSpeed(Convert.ToDouble(speed[cnt]));
                    speed[cnt] = string.Format("v{0}", speed[cnt]);
                }

                cnstList.Add(string.Format("\n\tVAR num {0}{1}:={2};", setName, cnt, position));
                instList.Add(string.Format("\n\t\tIndAMove {0},{1}\\ToAbsNum:={2}{3},{4}\\WObj:={5};", mecUnit, axisNum[cnt], setName, cnt, speed[cnt]));

                cnt++;
            }

            ///end step
            return new Dictionary<string, List<string>>
            {
                {"cnstList", cnstList},
                {"instList", instList},
                };
        }

        /// <summary>
        /// Reset independent joints.
        /// </summary>
        /// <param name="mecUnit">Active mechanical unit</param>
        /// <param name="axisNum">Select axis (1-6)</param>
        /// <param name="axisRefPos">Reference position</param>
        /// <returns></returns>
        public static List<string> IndReset(string mecUnit, List<int> axisNum, List<double> axisRefPos)
        {
            /// setup
            List<string> instList = new List<string>();
            int cnt;

            /// target instructions
            cnt = 0;
            foreach (var axis in axisNum)
            {
                if (cnt < axisNum.Count)
                {
                    if (cnt == axisRefPos.Count) { axisRefPos.Add(axisRefPos[0]); }
                }

                instList.Add(string.Format("\n\t\tIndAMove {0},{1} \\RefNum:={2} \\Old;", mecUnit, axis, axisRefPos[cnt]));

                cnt++;
            }

            return instList;
        }

        /// <summary>
        /// Create custom instruction from string.
        /// </summary>
        /// <param name="instructions"></param>
        /// <returns></returns>
        public static List<string> customInstruction(List<string> instructions)
        {
            List<string> instList = new List<string>();
            foreach (string inst in instructions)
            {
                instList.Add(string.Format("\n\t\t{0};", inst));
            }
            return instList;
        }

        /// <summary>
        /// Insert instructions into list at specified index.
        /// </summary>
        /// <param name="instList">Initial list of instructions</param>
        /// <param name="instructions">List of instructions to insert</param>
        /// <param name="index">List of indices at which to insert</param>
        /// <returns></returns>
        public static List<string> insertInstAtIndex(List<string> instList, List<string> instructions, List<int> index)
        {
            int cnt = 0;
            foreach (var dex in index)
            {
                if (cnt == instructions.Count) { instructions.Add(instructions[0]); }
                instList.Insert(dex + cnt, string.Format("\n\t\t{0};", instructions[cnt]));
                cnt++;
            }
            return instList;
        }

    }


    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////


    public class WriteProgram
    {


        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////


        /// <summary>
        /// Send RAPID to controller.
        /// </summary>
        /// <param name="filePath">File path</param>
        /// <returns></returns>
        public static bool sendRapidToController(string filePath)
        {
            if (!File.Exists(filePath))
            {
                throw new Exception("File could not be found.");
            }

            var scanner = new NetworkScanner();
            scanner.Scan();

            ControllerInfoCollection controllers = scanner.Controllers;
            using (var controller = ControllerFactory.CreateFrom(controllers[0]))
            {
                controller.Logon(UserInfo.DefaultUser);
                using (Mastership.Request(controller.Rapid))
                {
                    if (controller.OperatingMode == ControllerOperatingMode.Auto)
                    {
                        var fileName = Path.GetFileName(filePath);

                        try
                        {
                            var remoteDir = controller.FileSystem.RemoteDirectory.Replace('/', '\\');
                            var localDir = Path.GetDirectoryName(filePath);
                            controller.FileSystem.LocalDirectory = localDir;

                            //write the mod file
                            controller.FileSystem.PutFile(fileName, true);

                            //write the prf file
                            controller.FileSystem.PutFile("Dynamo.prg", true);

                        }
                        catch (Exception ex)
                        {
                            Debug.WriteLine(ex.Message);
                            return false;
                        }

                        using (var task = controller.Rapid.GetTask("T_ROB1")) // note the task name needs to be set up like this in Robot Studio
                        {
                            // read data from file
                            task.LoadProgramFromFile("Dynamo.prg", RapidLoadMode.Replace);
                            task.LoadModuleFromFile(fileName, RapidLoadMode.Add);
                            try
                            {
                                task.ResetProgramPointer();//seems to get hung up here.
                            }
                            catch (Exception ex)
                            {
                                Debug.WriteLine(ex.Message.ToString());
                                throw;
                            }

                            var result = task.Start(true);
                            Debug.WriteLine(result.ToString());
                        }
                    }
                    else
                    {
                        Debug.WriteLine("Automatic mode is required to start execution from a remote client.");
                    }
                }
                controller.Logoff();
            }

            return true;
        }

        /// <summary>
        /// Send robot target to controller.
        /// </summary>
        /// <param name="moduleName">Name of module</param>
        /// <param name="targetName">Name of target</param>
        /// <param name="targetValue">New value for target</param>
        public static void sendRobTargetToController(string moduleName, string targetName, string targetValue)
        {
            try
            {
                var scanner = new NetworkScanner();
                scanner.Scan();

                ControllerInfoCollection controllers = scanner.Controllers;
                using (var controller = ControllerFactory.CreateFrom(controllers[0]))
                {
                    controller.Logon(UserInfo.DefaultUser);
                    using (Mastership.Request(controller.Rapid))
                    {
                        if (controller.OperatingMode == ControllerOperatingMode.Auto)
                        {
                            using (var task = controller.Rapid.GetTask("T_ROB1"))
                            {
                                var target = new RobTarget();
                                using (var rapidData = task.GetRapidData(moduleName, targetName))
                                {
                                    if (rapidData.Value is RobTarget)
                                    {
                                        target.FillFromString2(targetValue);
                                        rapidData.Value = target;
                                    }
                                }

                                var result = task.Start(true);
                                Debug.WriteLine(result.ToString());

                                task.ResetProgramPointer();
                            }
                        }
                        else
                        {
                            Debug.WriteLine("Automatic mode is required to start execution from a remote client.");
                        }
                    }
                    controller.Logoff();
                }

            }
            catch (Exception ex)
            {
                Debug.WriteLine(ex.Message);
                Debug.WriteLine(ex.StackTrace);
            }
        }

        /// <summary>
        /// Merge and write data to a destination.
        /// </summary>
        /// <param name="filePath">"C:\Users\YourName\Desktop\myPath.prg"</param>
        /// <param name="cnstList">List of constants</param>
        /// <param name="instList">List of instructions</param>
        /// <param name="toolList">List of tooldata</param>
        /// <param name="wobjList">List of work-object data</param>
        /// <returns></returns>
        public static string createRapid(string filePath, List<string> cnstList, List<string> instList, List<string> toolList, List<string> wobjList)
        {
            /// setup
            var cnstBuilder = new StringBuilder();
            var moveBuilder = new StringBuilder();
            var toolBuilder = new StringBuilder();
            var wobjBuilder = new StringBuilder();
            foreach (string cnst in cnstList) { cnstBuilder.Append(cnst); }
            foreach (string inst in instList) { moveBuilder.Append(inst); }
            foreach (string tool in toolList) { toolBuilder.Append(tool); }
            foreach (string wobj in wobjList) { wobjBuilder.Append(wobj); }

            /// create rapid
            using (var tw = new StreamWriter(filePath, false))
            {
                var rapid = string.Format("MODULE MainModule\n" +
                                            "\t! Program data\n" +
                                            "{0}\n" +
                                            "{1}\n" +
                                            "\n\t! Target data" +
                                            "{2}\n" +
                                            "\n" +
                                            "\t! Routine\n" +
                                            "\tPROC main()\n" +
                                            "\t\tConfL\\Off;\n" +
                                            "\t\tSingArea\\Wrist;\n" +
                                            "\t\trStart;\n" +
                                            "\t\tRETURN;\n" +
                                            "\tENDPROC\n" +
                                            "\n" +
                                            "\tPROC rStart()\n" +
                                            "\n\t\t! instructions" +
                                            "{3}\n" +
                                            "\t\tRETURN;\n" +
                                            "\tENDPROC\n" +
                                            "\n" +
                                            "ENDMODULE\n",
                toolBuilder.ToString(), wobjBuilder.ToString(), cnstBuilder.ToString(), moveBuilder.ToString());

                tw.Write(rapid);
                tw.Flush();
            }

            /// end step
            return filePath;
        }

        
    }


    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////

}
















