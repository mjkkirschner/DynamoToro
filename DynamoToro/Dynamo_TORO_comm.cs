using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text;
using ABB.Robotics;
using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.Discovery;
using ABB.Robotics.Controllers.RapidDomain;
using ABB.Robotics.Controllers.IOSystemDomain;
using Autodesk.DesignScript.Geometry;
using Autodesk.DesignScript.Runtime;




/// 
/// finds, looks-for, searches-for data either on controller or network
/// sends, updates, replaces data on controller or network
/// 



namespace Dynamo_toRo
{



    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////



    public class RobComm
    {



        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////




        /// <summary>
        /// Scans network for controllers and returns SystemName, SystemID, Version, and IPAddress.
        /// </summary>
        /// <param name="run">True to run</param>
        /// <returns></returns>
        [MultiReturn(new[] { "robotController", "virtualController" })]
        public static Dictionary<string, List<string[]>> findControllers(bool run)
        {
            List<string[]> roboCtrl = new List<string[]> { };
            List<string[]> virtCtrl = new List<string[]> { };

            if (run == true)
            {
                NetworkScanner scanner = new NetworkScanner();
                scanner.Scan();

                ControllerInfoCollection controllers = scanner.Controllers;
                foreach (ControllerInfo controller in controllers)
                {
                    if (controller.IsVirtual == false)
                    {
                        string[] eachController1 = new string[5]
                        {
                                controller.SystemName.ToString(),
                                controller.SystemId.ToString(),
                                controller.Availability.ToString(),
                                controller.Version.ToString(),
                                controller.IPAddress.ToString()
                        };
                        roboCtrl.Add(eachController1);
                    }

                    else
                    {
                        string[] eachController2 = new string[5]
                        {
                                controller.SystemName.ToString(),
                                controller.SystemId.ToString(),
                                controller.Availability.ToString(),
                                controller.Version.ToString(),
                                controller.IPAddress.ToString()
                        };
                        virtCtrl.Add(eachController2);
                    }
                }
            }
            return new Dictionary<string, List<string[]>>
            {
                {"robotController", roboCtrl},
                {"virtualController", virtCtrl}
            };
        }

        /// <summary>
        /// Send a .PRG file to controller.
        /// </summary>
        /// <param name="run">True to run</param>
        /// <param name="controllerData">Controller data</param>
        /// <param name="filePath">File to send</param>
        public static void sendProgramToController(bool run, string[] controllerData, string filePath)
        {
            if (run == true)
            {
                Guid systemId = new Guid(controllerData[1]);
                Controller controller = new Controller(systemId);
                controller.Logon(UserInfo.DefaultUser);

                Task newTask = controller.Rapid.GetTask("T_ROB1");
                using (Mastership.Request(controller.Rapid))
                {
                    newTask.LoadProgramFromFile(filePath, RapidLoadMode.Replace);
                }

                controller.Logoff();
            }
        }

        /// <summary>
        /// Read RobTarget and JointTarget from current position.
        /// </summary>
        /// <param name="run">True to run</param>
        /// <param name="controllerData">Controller data</param>
        /// <returns></returns>
        [MultiReturn(new[] { "robotTarget", "jointTarget" })]
        public static Dictionary<string, string> getCurrentPosition(bool run, string[] controllerData)
        {
            string robotTarget = "";
            string jointTarget = "";

            if (run == true)
            {
                Guid systemId = new Guid(controllerData[1]);
                Controller controller = new Controller(systemId);
                controller.Logon(UserInfo.DefaultUser);

                Task newTask = controller.Rapid.GetTask("T_ROB1");
                using (Mastership.Request(controller.Rapid))
                {
                    robotTarget = newTask.GetRobTarget().ToString();
                    jointTarget = newTask.GetJointTarget().ToString();
                }
            }

            return new Dictionary<string, string>
            {
                {"robotTarget", robotTarget },
                {"jointTarget", jointTarget }
            };
        }

        /// <summary>
        /// Read RobTargets and JointTargets for MainModule on controller.
        /// </summary>
        /// <param name="run">True to run</param>
        /// <param name="controllerData">Controller data</param>
        /// <returns></returns>
        [MultiReturn(new[] { "robTargets", "jointTargets" })]
        public static Dictionary<string, List<string>> readTargetData(bool run, string[] controllerData)
        {
            List<string> rTargets = new List<string>();
            List<string> jTargets = new List<string>();

            if (run == true)
            {
                Guid systemId = new Guid(controllerData[1]);
                Controller controller = new Controller(systemId);
                controller.Logon(UserInfo.DefaultUser);

                Task newTask = controller.Rapid.GetTask("T_ROB1");
                using (Mastership.Request(controller.Rapid))
                {
                    RapidSymbolSearchProperties sProp = RapidSymbolSearchProperties.CreateDefault();
                    sProp.Types = SymbolTypes.Data;
                    RapidSymbol[] datas = newTask.GetModule("MainModule").SearchRapidSymbol(sProp);
                    foreach (RapidSymbol rs in datas)
                    {
                        RapidData rd = controller.Rapid.GetTask("T_ROB1").GetModule("MainModule").GetRapidData(rs);
                        if (rd.Value is RobTarget)
                        { rTargets.Add(rd.Value.ToString()); }
                        if (rd.Value is JointTarget)
                        { jTargets.Add(rd.Value.ToString()); }
                    }
                }
            }

            return new Dictionary<string, List<string>>
            {
                {"robTargets", rTargets },
                {"jointTargets", jTargets }
            };
        }

        /// <summary>
        /// Read tooldata and wobjdata for MainModule on controller.
        /// </summary>
        /// <param name="run">True to run</param>
        /// <param name="controllerData">Controller data</param>
        /// <returns></returns>
        [MultiReturn(new[] { "programData" /*, "currentData"*/ })]
        public static Dictionary<string, List<string[]>> readProgramData(bool run, string[] controllerData)
        {
            List<string[]> progData = new List<string[]> { };
            ///List<string[]> currData = new List<string[]> { };

            if (run == true)
            {
                Guid systemId = new Guid(controllerData[1]);
                Controller controller = new Controller(systemId);
                controller.Logon(UserInfo.DefaultUser);

                Task newTask = controller.Rapid.GetTask("T_ROB1");
                using (Mastership.Request(controller.Rapid))
                {
                    RapidSymbolSearchProperties sProp = RapidSymbolSearchProperties.CreateDefault();

                    sProp.Types = SymbolTypes.Data;
                    RapidSymbol[] progDatas = newTask.GetModule("MainModule").SearchRapidSymbol(sProp);
                    foreach (RapidSymbol rs in progDatas)
                    {
                        RapidData rd = controller.Rapid.GetTask("T_ROB1").GetModule("MainModule").GetRapidData(rs);
                        if ((rd.Value is ToolData) | (rd.Value is WobjData))
                        {
                            string[] eachProg = new string[3]
                            {
                                rd.RapidType,
                                rd.Name,
                                rd.Value.ToString()
                            };
                            progData.Add(eachProg);
                        }
                    }

                    /*
                    RapidSymbol[] currDatas = newTask.GetModule("MainModule").SearchRapidSymbol(sProp);
                    foreach (RapidSymbol rs in currDatas)
                    {
                        RapidData rd = controller.Rapid.GetTask("T_ROB1").GetModule("MainModule").GetRapidData(rs);
                        if ((rd.Value is ToolData) | (rd.Value is WobjData))
                        {
                            string[] eachCurr = new string[3]
                            {
                                rd.RapidType,
                                rd.Name,
                                rd.Value.ToString()
                            };
                            currData.Add(eachCurr);
                        }
                    }
                    */

                }
            }

            return new Dictionary<string, List<string[]>>
            {
                {"programData", progData } /*,{"currentData", currData}*/
            };
        }

        /// <summary>
        /// Set current program pointer on controller.
        /// </summary>
        /// <param name="run">True to run</param>
        /// <param name="controllerData">Controller data</param>
        /// <param name="value"></param>
        public static void setProgramPointer(bool run, string[] controllerData, int value)
        {
            if (run == true)
            {
                Guid systemId = new Guid(controllerData[1]);
                Controller controller = new Controller(systemId);
                controller.Logon(UserInfo.DefaultUser);

                Task newTask = controller.Rapid.GetTask("T_ROB1");
                using (Mastership.Request(controller.Rapid))
                {
                    newTask.SetProgramPointer("MainModule", value);
                }
            }
        }














        ///// The following is incomplete, untested, or otherwise buggy.
        ///// The following is incomplete, untested, or otherwise buggy.
        ///// The following is incomplete, untested, or otherwise buggy.
        ///// The following is incomplete, untested, or otherwise buggy.
        ///// The following is incomplete, untested, or otherwise buggy.
        ///// The following is incomplete, untested, or otherwise buggy.




        private static List<object> progList = new List<object>();

        public static List<object> listenToRobot(bool run, string[] controllerData)
        {
            if (run == true)
            {
                Guid systemId = new Guid(controllerData[1]);
                Controller controller = new Controller(systemId);
                controller.Logon(UserInfo.DefaultUser);

                Task newTask = controller.Rapid.GetTask("T_ROB1");
                int initRow = newTask.ProgramPointer.Range.Begin.Row;
                newTask.ProgramPointerChanged += NewTask_ProgramPointerChanged;
            }
            return progList;
        }

        private static void NewTask_ProgramPointerChanged(object sender, ProgramPositionEventArgs e)
        {
            int row = e.Position.Range.Begin.Row;
            progList.Add(row);
        }






        public static string setProgramPointer(bool run, string[] controllerData, int row)
        {
            if (run == true)
            {
                Guid systemId = new Guid(controllerData[1]);
                Controller controller = new Controller(systemId);
                controller.Logon(UserInfo.DefaultUser);

                Task newTask = controller.Rapid.GetTask("T_ROB1");
                using (Mastership.Request(controller.Rapid))
                {
                    newTask.SetProgramPointer("MainModule", row);
                }
                return "Success!";
            }
            else
            {
                return "True to run";
            }
        }


        public static List<int> getExecutionStatus2(bool run, string[] controllerData)
        {
            List<int> posList = new List<int>();

            if (run == true)
            {
                Guid systemId = new Guid(controllerData[1]);
                Controller controller = new Controller(systemId);
                controller.Logon(UserInfo.DefaultUser);
                Task newTask = controller.Rapid.GetTask("T_ROB1");

                newTask.MotionPointerChanged += MPChanged;
                int pPos = newTask.ProgramPointer.Range.Begin.Row;
                posList.Add(pPos);
            }
            return posList;
        }

        private static void MPChanged(object sender, ProgramPositionEventArgs e)
        {
            int row = e.Position.Range.Begin.Row;
            progList.Add(row);
        }




        /*
        


        public static List<string> getExecutionStatus(bool run, string[] controllerData, string[] programData)
        {
            List<string> variables = new List<string>();
            if (run == true)
            {
                Guid systemId = new Guid(controllerData[1]);
                Controller controller = new Controller(systemId);
                controller.Logon(UserInfo.DefaultUser);

                Task newTask = controller.Rapid.GetTask("T_ROB1");
                Module newModule = newTask.GetModule("MainModule");
                using (Mastership.Request(controller.Rapid))
                {
                    newTask.SetProgramPointer("MainModule", 50);

                    //Module ppModule = newTask.GetModule(newTask.ProgramPointer.Module);
                    //Module mpModule = newTask.GetModule(newTask.MotionPointer.Module);
                    //ppList.Add(ppModule.ToString());
                    //mpList.Add(ppModule.ToString());

                    ProgramPosition pPos = newTask.ProgramPointer;
                    ProgramPosition mPos = newTask.MotionPointer;

                    variables.Add(pPos.ToString());
                    variables.Add(mPos.ToString());
                }
                //newTask.MotionPointerChanged += new EventHandler<ProgramPositionEventArgs>(mpChanged);
            }
            return variables;
        }




        
        public static List<int> getExecutionStatus2(bool run, string[] controllerData)
        {
            List<int> posList = new List<int>();

            if (run == true)
            {
                Guid systemId = new Guid(controllerData[1]);
                Controller controller = new Controller(systemId);
                controller.Logon(UserInfo.DefaultUser);
                Task newTask = controller.Rapid.GetTask("T_ROB1");

                //ProgramPosition pPos = newTask.ProgramPointer;
                //int p0 = pPos.Range.Begin.Row;
                //int p1 = pPos.Range.End.Row;
                //posList.Add(p0);
                //posList.Add(p1);
                //newTask.ProgramPointerChanged += new EventHandler<ProgramPositionEventArgs>(dataRecorder);

                newTask.MotionPointerChanged += new EventHandler<ProgramPositionEventArgs>(DataRecorder);
                //newTask.MotionPointerChanged += DataRecorder;
                posList.Add(m0);
            }
            return posList;
        }

        private static void DataRecorder(object sender, ProgramPositionEventArgs e)
        {
            int m0 = e.Position.Range.Begin.Row;

            //throw new NotImplementedException(m0.ToString());
        }






        ///This node will pass a cycle (of some length) of targets to the controller.
        ///Listens for whenever (a) the program pointer and (b) the mechanical unit have passed a target. 
        ///Cycles targets OUT : whenever mechanical unit has passed a target after two step buff.
        ///Cycles targets IN  : whenever the program pointer has passed a target before two step buff.
        public static void sendTargets(bool run, string[] controllerData, string[] programData)
        {
            if (run == true)
            {
                Guid systemId = new Guid(controllerData[1]);
                Controller controller = new Controller(systemId);
                controller.Logon(UserInfo.DefaultUser);

                Task newTask = controller.Rapid.GetTask("T_ROB1");
                using (Mastership.Request(controller.Rapid))
                {
                    RapidSymbolSearchProperties sProp = RapidSymbolSearchProperties.CreateDefault();

                    sProp.Types = SymbolTypes.Data;
                    RapidSymbol[] targDatas = newTask.GetModule("MainModule").SearchRapidSymbol(sProp);
                    foreach (RapidSymbol rs in targDatas)
                    {
                        RapidData rd = controller.Rapid.GetTask("T_ROB1").GetModule("MainModule").GetRapidData(rs);
                        if (rd.RapidType.ToString() == "RobTarget" || rd.RapidType.ToString() == "JointTarget")
                        {
                            rd.Value.FillFromString(programData[1]);
                        }
                    }
                }
            }
        }



        public static void sendProgramData(bool run, string[] controllerData, string[] programData)
        {
            if (run == true)
            {
                Guid systemId = new Guid(controllerData[1]);
                Controller controller = new Controller(systemId);
                controller.Logon(UserInfo.DefaultUser);

                Task newTask = controller.Rapid.GetTask("T_ROB1");
                using (Mastership.Request(controller.Rapid))
                {
                    RapidSymbolSearchProperties sProp = RapidSymbolSearchProperties.CreateDefault();

                    sProp.Types = SymbolTypes.Data;
                    RapidSymbol[] progDatas = newTask.GetModule("MainModule").SearchRapidSymbol(sProp);
                    foreach (RapidSymbol rs in progDatas)
                    {
                        RapidData rd = controller.Rapid.GetTask("T_ROB1").GetModule("MainModule").GetRapidData(rs);
                        if (rd.RapidType.ToString() == programData[0] && rd.Name == programData[1])
                        {
                            /// this will replace data if it has the same name as programData
                            /// perhaps improve by adding to current programdata list
                            rd.Value.FillFromString(programData[1]);
                        }
                    }
                }
            }


        */







        /*


        public static void sendProgramData(bool run, string[] controllerData, string[] programData)
        {
            if (run == true)
            {
                Guid systemId = new Guid(controllerData[1]);
                Controller controller = new Controller(systemId);
                controller.Logon(UserInfo.DefaultUser);

                Task newTask = controller.Rapid.GetTask("T_ROB1");
                using (Mastership.Request(controller.Rapid))
                {
                    if (programData[1] == "ToolData")
                    { string doSomething = "Something"; }
                }
            }
        }

        
        public static void sendRapidDataToController(bool run, string[] controllerData, string oldVarName, string newVar)
        {
            if (run == true)
            {
                Guid systemId = new Guid(controllerData[1]);
                Controller controller = new Controller(systemId);
                controller.Logon(UserInfo.DefaultUser);

                Task newTask = controller.Rapid.GetTask("T_ROB1");
                using (Mastership.Request(controller.Rapid))
                {
                    using (var task = controller.Rapid.GetTask("T_ROB1"))
                    {
                        using (var rapidData = task.GetRapidData("MainModule", oldVarName))
                        {
                            if (rapidData.Name.ToString() == oldVarName)
                            {
                                rapidData.Value = newVar;
                            }
                        }
                    }
                }
            }
        }


            public static void sendDataToController(bool run, string[] controllerData, string dataName, IRapidData dataToSend)
            {
                if (run == true)
                {
                    Guid systemId = new Guid(controllerData[1]);
                    Controller controller = new Controller(systemId);
                    controller.Logon(UserInfo.DefaultUser);

                    using (Mastership.Request(controller.Rapid))
                    {
                        using (var task = controller.Rapid.GetTask("T_ROB1"))
                        {
                            using (var rapidData = task.GetRapidData("T_ROB1", dataName))
                            {
                                rapidData.Value = dataToSend;
                            }
                        }
                    }
                }
            }


            public static void sendToolToController(bool run, string[] controllerName, string toolName, ToolData toolData)
            {
                if (run == true)
                {
                    NetworkScanner scanner = new NetworkScanner();
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
                                    using (var rapidData = task.GetRapidData("T_ROB1", toolName))
                                    {
                                        if (rapidData.Value is ToolData)
                                        {
                                            rapidData.Value = toolData;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }


            */





    }
}
