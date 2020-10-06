// Copyright 2020 Rensselaer Polytechnic Institute
//                Wason Technology, LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Collections.Generic;
using System.Text;
using RobotRaconteur;
using com.robotraconteur.robotics.robot;
using GeneralRoboticsToolbox;
using System.IO;
using System.Linq;
using com.robotraconteur.robotics.joints;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
using com.robotraconteur.geometry;
using com.robotraconteur.action;
using com.robotraconteur.robotics.trajectory;
using RobotRaconteur.Companion.Robot;
using Staubli.Robotics.Soap.Proxies.ServerV0;
using Staubli.Robotics.Soap.Proxies.ServerV1;
using Staubli.Robotics.Soap.Proxies.ServerV3;
using StaubliSoapClient;
using System.Globalization;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;



namespace StaubliRobotRaconteurDriver
{
    public class StaubliRobot : AbstractRobot
    {

        private string m_address;
        private Staubli.Robotics.Soap.Proxies.ServerV0.Robot m_currentRobot = null;
        private int m_sessionId = -1;
        private CS8ServerV0PortTypeClient m_cs8ServerV0;
        private CS8ServerV1PortTypeClient m_cs8ServerV1;
        private CS8ServerV3PortTypeClient m_cs8ServerV3;
        private double[] m_jointPosition;
        private const string m_applicationName = "soapSample.pjx";
        public int port;
        public string address;
        protected bool m_Connected = false;
        public StaubliRobot(com.robotraconteur.robotics.robot.RobotInfo robot_info) : base(robot_info, -1)
        {
            _uses_homing = false;
            _has_position_command = true;
            _has_velocity_command = false;
            _update_period = 4;
            robot_info.robot_capabilities &= (uint)(RobotCapabilities.jog_command & RobotCapabilities.position_command);
            
        }
        private Frame getAsIdentity()
        {
            Frame l_ret = new Frame();
            Tools.setRxRyRzCoord(0, 0, 0, out l_ret);
            l_ret.px = 0;
            l_ret.py = 0;
            l_ret.pz = 0;
            return l_ret;
        }
        public override void _start_robot()
        {
            m_address = "http://" + address + ":" + port;

            this.m_cs8ServerV0 = null;


            //CS8ServerV0(m_txtBoxIpAdress.Text, m_txtBoxPortNumber.Text);
            m_cs8ServerV0 = new CS8ServerV0PortTypeClient();
            m_cs8ServerV0.Endpoint.Address = new System.ServiceModel.EndpointAddress(m_address);

            m_cs8ServerV3 = new CS8ServerV3PortTypeClient();
            m_cs8ServerV3.Endpoint.Address = new System.ServiceModel.EndpointAddress(this.m_address + "/CS8ServerV3");

            // If soap server is created, enable others tests
            if (m_cs8ServerV0 == null) throw new Exception("Could not connect to robot");
            // will generate an error if cannot connect to server
            SoapServerVersion l_soapVersion = m_cs8ServerV0.getSoapServerVersion("me", "0");
            double l_version = Convert.ToDouble(l_soapVersion.version, CultureInfo.GetCultureInfo("en-US").NumberFormat);

            m_cs8ServerV0.login("default", "", out m_sessionId);



            m_currentRobot = m_cs8ServerV0.getRobots(m_sessionId)[0];



            m_cs8ServerV3.setSchedulingMode(m_sessionId, SchedulingMode.SCHEDULINGINTERNAL);

            m_Connected = true;
        }

        protected override Task _send_disable()
        {
            throw new NotImplementedException();
        }

        protected override Task _send_enable()
        {
            throw new NotImplementedException();
        }

        protected override Task _send_reset_errors()
        {
            throw new NotImplementedException();
        }
        private double[] m_des = new double[6];
        protected override void _send_robot_command(long now, double[] joint_pos_cmd, double[] joint_vel_cmd)
        {
            if (joint_pos_cmd != null)
            {
                if (joint_pos_cmd.Length != 6) throw new InvalidOperationException("Joint position vector must have size of 6");

                MotionDesc l_mdesc = new MotionDesc();
                l_mdesc.acc = 2.25;
                l_mdesc.dec = 2.25;
                l_mdesc.vel = 1.50;
                l_mdesc.transVel = 99999;
                l_mdesc.rotVel = 99999;
                l_mdesc.freq = 1.0;
                l_mdesc.absRel = MoveType.ABSOLUTEMOVE;
                l_mdesc.config = new Config();
                l_mdesc.config.Item = new AnthroConfig();
                ((AnthroConfig)l_mdesc.config.Item).shoulder = ShoulderConfig.SSAME;
                ((AnthroConfig)l_mdesc.config.Item).elbow = PositiveNegativeConfig.PNSAME;
                ((AnthroConfig)l_mdesc.config.Item).wrist = PositiveNegativeConfig.PNSAME;
                l_mdesc.frame = getAsIdentity();
                l_mdesc.tool = getAsIdentity();
                //l_mdesc.blendType = BlendType.BLENDOFF;
                l_mdesc.blendType = BlendType.BLENDJOINT;
                l_mdesc.distBlendPrev = (1 / 1000.0);		// in meter
                l_mdesc.distBlendNext = (1 / 1000.0);		// in meter

                MotionReturnCode l_rc;
                m_cs8ServerV3.resetMotion(m_sessionId);
                m_cs8ServerV3.moveJJ(m_sessionId, 0, joint_pos_cmd, l_mdesc, out l_rc);
                m_des = joint_pos_cmd;
            }
        }

        protected override void _run_timestep(long now)
        {

            _joint_position= m_cs8ServerV0.getRobotJointPos(m_sessionId, 0);
            _ready = true;
            _enabled = true;
            Config config = new Config();
            Frame temp = new Frame();
            temp = m_cs8ServerV3.forwardKin(m_sessionId, 0, _joint_position, out config);
            com.robotraconteur.geometry.Pose tcp_pose = default;
            
            Matrix<double> R = Matrix<double>.Build.Dense(3, 3);
            R[0, 0] = temp.nx;
            R[0, 1] = temp.ox;
            R[0, 2] = temp.ax;
            R[1, 0] = temp.ny;
            R[1, 1] = temp.oy;
            R[1, 2] = temp.ay;
            R[2, 0] = temp.nz;
            R[2, 1] = temp.oz;
            R[2, 2] = temp.az;
            Vector<double> q = GeneralRoboticsToolbox.Functions.R2Q(R);
            tcp_pose.orientation.w = q[0];
            tcp_pose.orientation.x = q[1];
            tcp_pose.orientation.y = q[2];
            tcp_pose.orientation.z = q[3];
            tcp_pose.position.x = temp.px;
            tcp_pose.position.y = temp.py;
            tcp_pose.position.z = temp.pz;
            _endpoint_pose = new Pose[] { tcp_pose };
            base._run_timestep(now);

            /*
             egm_client.GetState(out var egm_last_recv, out bool egm_enabled, out bool egm_ready, out var egm_joint_pos, out var egm_tcp_pos);

            _last_joint_state = egm_last_recv;
            _last_endpoint_state = egm_last_recv;
            _last_robot_state = egm_last_recv;
            _enabled = egm_enabled;
            _ready = egm_ready;
            if (egm_joint_pos != null)
            {
                _joint_position = egm_joint_pos;
            }
            else
            {
                _joint_position = new double[0];
            }
            _endpoint_pose = new Pose[] { egm_tcp_pos };

            

            if (_command_mode == RobotCommandMode.halt || _command_mode == RobotCommandMode.invalid_state)
            {
                egm_client.StopMotion();
            }
            */
        }

        public override void Dispose()
        {
            if (!m_Connected) return;
            m_cs8ServerV0.Close();
            base.Dispose();
        }
    }
}
