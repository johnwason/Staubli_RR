using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Staubli.Robotics.Soap.Proxies.ServerV0;
using Staubli.Robotics.Soap.Proxies.ServerV1;
using Staubli.Robotics.Soap.Proxies.ServerV3;

namespace StaubliSoapClient
{
    class Tools
    {
        public const double METER_TO_MM = 1000.0;
        public const double MM_TO_METER = 0.001;

        public const double RAD_TO_DEG = (180.0 / Math.PI);
        public const double DEG_TO_RAD = (Math.PI / 180.0);

        public static readonly double SMALL_FLOAT = Math.Pow(1, -5);
        public static readonly double VERY_SMALL_FLOAT = Math.Pow(1, -10);
        public static readonly double BIG_FLOAT = Math.Pow(1, 10);
        public static readonly double VERY_BIG_FLOAT = Math.Pow(1, 32);

        public static void getRxRyRzCoord(Frame x_fr, out double x_Rx, out double x_Ry, out double x_Rz)
        {
            double l_sinRy;

            l_sinRy = x_fr.ax;
            // ATTENTION : it may be possible that sinRy > 1.0 or < -1.0 (numerical pbm)
            if (l_sinRy < (-1.0 + SMALL_FLOAT * SMALL_FLOAT / 2.0))
            {
                x_Rx = 0;
                x_Ry = -Math.PI / 2.0;
                x_Rz = Math.Atan2(x_fr.ny, x_fr.nz);
            }
            else if (l_sinRy > (1.0 - SMALL_FLOAT * SMALL_FLOAT / 2.0))
            {
                x_Rx = 0;
                x_Ry = Math.PI / 2.0;
                x_Rz = Math.Atan2(x_fr.ny, -x_fr.nz);
            }
            else
            {
                double l_sign = 1.0;
                x_Ry = Math.Asin(l_sinRy);
                if ((x_fr.az < 0.0) && (x_fr.nx < 0.0)
                   && ((Math.Abs(l_sinRy) > VERY_SMALL_FLOAT) || (Math.Abs(x_fr.ay) > VERY_SMALL_FLOAT)))
                {	// if cosRy > 0, and both cosRx and cosRz are < 0, choose Ry so that
                    // cosRx and cosRz are positive
                    if (x_Ry >= 0.0)
                        x_Ry = Math.PI - x_Ry;
                    else
                        x_Ry = -Math.PI - x_Ry;
                    l_sign = -1.0;
                }
                x_Rx = Math.Atan2(-l_sign * x_fr.ay, l_sign * x_fr.az);
                x_Rz = Math.Atan2(-l_sign * x_fr.ox, l_sign * x_fr.nx);
            }
        }

        static void getSinCos(double x_angle, out double x_sin, out double x_cos)
        {
            double l_t, l_t2;

            // Too big angle involve numerical noise
            if (Math.Abs(x_angle) > BIG_FLOAT)
            {
                x_angle = x_angle % (2.0 * Math.PI);
            }
            l_t = Math.Tan(x_angle / 2.0);
            l_t2 = l_t * l_t;
            x_sin = 2.0 * l_t / (1.0 + l_t2);
            x_cos = (1.0 - l_t2) / (1.0 + l_t2);
        }

        public static void setRxRyRzCoord(double x_Rx, double x_Ry, double x_Rz, out Frame x_fr)
        {
            double l_sinRx, l_sinRy, l_sinRz;
            double l_cosRx, l_cosRy, l_cosRz;

            getSinCos(x_Rx, out l_sinRx, out l_cosRx);
            getSinCos(x_Ry, out l_sinRy, out l_cosRy);
            getSinCos(x_Rz, out l_sinRz, out l_cosRz);

            x_fr = new Frame();
            x_fr.nx = l_cosRz * l_cosRy;
            x_fr.ny = l_cosRz * l_sinRy * l_sinRx + l_sinRz * l_cosRx;
            x_fr.nz = -l_cosRz * l_sinRy * l_cosRx + l_sinRz * l_sinRx;

            x_fr.ox = -l_sinRz * l_cosRy;
            x_fr.oy = l_cosRz * l_cosRx - l_sinRz * l_sinRy * l_sinRx;
            x_fr.oz = l_cosRz * l_sinRx + l_sinRz * l_sinRy * l_cosRx;

            x_fr.ax = l_sinRy;
            x_fr.ay = -l_cosRy * l_sinRx;
            x_fr.az = l_cosRy * l_cosRx;
        }
    }

}
