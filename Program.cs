using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using RobotRaconteur;
using Staubli.Robotics.Soap.Proxies.ServerV0;
using Staubli.Robotics.Soap.Proxies.ServerV1;
using Staubli.Robotics.Soap.Proxies.ServerV3;
using System.ServiceModel;
using System.Globalization;
using StaubliSoapClient;
using RobotRaconteur.InfoParser;
using Mono.Options;

namespace StaubliTX40Server
{
    class Program
    {
        //static StaubliRobotRaconteurDriver.StaubliRobot robot;
        static int Main(string[] args)
        {
            /*
            RobotRaconteurNode.s.NodeName = "StaubliTX40Server";
            RobotRaconteurNode.s.NodeID = new NodeID("241c:2283:cc37:764f:b784:999f:a7d7:e3e8");

            RobotRaconteurNode.s.RegisterServiceType(new StaubliTX40_interface.StaubliTX40_interfaceFactory());
            TcpChannel t = new RobotRaconteur.TcpChannel();
            t.StartServer(4445);

            RobotRaconteurNode.s.RegisterChannel(t);
            robot = new StaubliTX40();
            */
            //robot.Connect(/*"127.0.0.1"*/ "10.10.90.1", 5653);

            //robot.Connect("127.0.0.1", 5653);

            //server.Connect(...)
            /*
            RobotRaconteurNode.s.RegisterService("StaubliTX40", "StaubliTX40_interface", robot);

            Console.WriteLine("Server started");
            Console.ReadLine();

            RobotRaconteurNode.s.Shutdown();
            */

            bool shouldShowHelp = false;
            string robot_info_file = null;

            var options = new OptionSet {
                { "robot-info-file=", n => robot_info_file = n },
                { "h|help", "show this message and exit", h => shouldShowHelp = h != null }
            };

            List<string> extra;
            try
            {
                // parse the command line
                extra = options.Parse(args);
            }
            catch (OptionException e)
            {
                // output some error message
                Console.Write("ABBRobotRaconteurDriver: ");
                Console.WriteLine(e.Message);
                Console.WriteLine("Try `ABBRobotRaconteurDriver --help' for more information.");
                return 1;
            }

            if (shouldShowHelp)
            {
                Console.WriteLine("Usage: ABBRobotRaconteurDriver [Options+]");
                Console.WriteLine();
                Console.WriteLine("Options:");
                options.WriteOptionDescriptions(Console.Out);
                return 0;
            }

            if (robot_info_file == null)
            {
                Console.WriteLine("error: robot-info-file must be specified");
                return 1;
            }


            var robot_info = RobotInfoParser.LoadRobotInfoYamlWithIdentifierLocks(robot_info_file);
            using (robot_info.Item2)
            {



                using (var robot = new StaubliRobotRaconteurDriver.StaubliRobot(robot_info.Item1))
                {
                    robot.address = "10.10.90.1";
                    robot.port = 5653;
                    robot._start_robot();
                    using (var node_setup = new ServerNodeSetup("ABB_robot", 58651, args))
                    {


                        RobotRaconteurNode.s.RegisterService("abb_robot", "com.robotraconteur.robotics.robot", robot);

                        Console.WriteLine("Press enter to exit");
                        Console.ReadKey();

                        RobotRaconteurNode.s.Shutdown();
                    }
                }
            }

            return 0;
        }


    }





    

}