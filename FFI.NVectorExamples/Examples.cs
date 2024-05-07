//   This file is available from www.navlab.net/nvector
//
//   The content of this file is based on the following publication:
//
//   Gade, K. (2010). A Nonsingular Horizontal Position Representation, The Journal 
//   of Navigation, Volume 63, Issue 03, pp 395-417, July 2010. 
//   (www.navlab.net/Publications/A_Nonsingular_Horizontal_Position_Representation.pdf)
//
//   This paper should be cited in publications using this file.
//
//   Copyright (c) 2016, Norwegian Defence Research Establishment (FFI)
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without 
//   modification, are permitted provided that the following conditions are met:
//
//   1. Redistributions of source code must retain the above publication 
//   information, copyright notice, this list of conditions and the following disclaimer.
//
//   2. Redistributions in binary form must reproduce the above publication 
//   information, copyright notice, this list of conditions and the following disclaimer 
//   in the documentation and/or other materials provided with the distribution.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED 
//   TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
//   PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS 
//   BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
//   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
//   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
//   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
//   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
//   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
//   THE POSSIBILITY OF SUCH DAMAGE.
//   
//   Originated: 2016 Jørn Inge Vestgården, FFI
//   Based on original Matlab code by Kenneth Gade, FFI

using FFI.NVector;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace FFI.NVectorExamples
{
    public class Examples
    {
        private NVMath _NV;
        public Examples()
        {
            _NV = new NVMath();
        }

        // Example 1: "A and B to delta"
        public void Example1()
        { 
            // Positions A and B are given in (decimal) degrees and depths:
            // Position A:
            var lat_EA_deg=1;
            var long_EA_deg=2;
            var z_EA=3;

            // Position B:
            var lat_EB_deg=4;
            var long_EB_deg=5;
            var z_EB=6;

            // Find the exact vector between the two positions, given in meters north,
            // east, and down, i.e. find p_AB_N.

            // SOLUTION:

            // Step1: Convert to n-vectors (rad() converts to radians):
            var n_EA_E=_NV.lat_long2n_E(_NV.rad(lat_EA_deg),_NV.rad(long_EA_deg));
            var n_EB_E=_NV.lat_long2n_E(_NV.rad(lat_EB_deg),_NV.rad(long_EB_deg));

            // Step2: Find p_AB_E (delta decomposed in E). WGS-84 ellipsoid is default:
            var p_AB_E = _NV.n_EA_E_and_n_EB_E2p_AB_E(n_EA_E,n_EB_E,z_EA,z_EB);

            // Step3: Find R_EN for position A:
            var R_EN=_NV.n_E2R_EN(n_EA_E);

            // Step4: Find p_AB_N

            var p_AB_N = Utilities.MatMul(Utilities.Transpose(R_EN), p_AB_E);
            // (Note the transpose of R_EN: The "closest-rule" says that when
            // decomposing, the frame in the subscript of the rotation matrix that is
            // closest to the vector, should equal the frame where the vector is
            // decomposed. Thus the calculation R_NE*p_AB_E is correct, since the vector
            // is decomposed in E, and E is closest to the vector. In the above example
            // we only had R_EN, and thus we must transpose it: R_EN'=R_NE)

            // Step5: Also find the direction (azimuth) to B, relative to north:
            var azimuth= Math.Atan2(p_AB_N[1],p_AB_N[0]); // positive angle about down-axis

            Console.Out.WriteLine("Ex1, delta north, east, down = " + p_AB_N[0] + ", " + p_AB_N[1] + ", " + p_AB_N[2] + " m");
            Console.Out.WriteLine("Ex1, azimuth = " + _NV.deg(azimuth) + " deg");
            Console.Out.WriteLine();
        }

        // Example 2: "B and delta to C"
        public void Example2() 
        {
            // delta vector from B to C, decomposed in B is given:
            var p_BC_B = new double[] { 3000, 2000, 100 };

            // Position and orientation of B is given:
            var n_EB_E=_NV.unit(new double[] {1, 2, 3}); // unit to get unit length of vector
            var z_EB=-400;
            var R_NB=_NV.zyx2R(_NV.rad(10),_NV.rad(20),_NV.rad(30)); // the three angles are yaw, pitch, and roll

            // A custom reference ellipsoid is given (replacing WGS-84):
            var a=6378135; 
            var f=1/298.26; // (WGS-72)

            // Find the position of C.
            
            // SOLUTION:

            // Step1: Find R_EN:
            var R_EN=_NV.n_E2R_EN(n_EB_E);

            // Step2: Find R_EB, from R_EN and R_NB:
            var R_EB= Utilities.MatMul(R_EN,R_NB); // Note: closest frames cancel

            // Step3: Decompose the delta vector in E:
            var p_BC_E= Utilities.MatMul(R_EB,p_BC_B); // no transpose of R_EB, since the vector is in B

            // Step4: Find the position of C, using the functions that goes from one
            // position and a delta, to a new position:
            var n_EC_Eandz_EC = _NV.n_EA_E_and_p_AB_E2n_EB_E(n_EB_E,p_BC_E,z_EB,a,f);
            var n_EC_E = n_EC_Eandz_EC.Item1;
            var z_EC =  n_EC_Eandz_EC.Item2;
            
            // When displaying the resulting position for humans, it is more convenient
            // to see lat, long:
            var lat_long_EC = _NV.n_E2lat_long(n_EC_E);
            var lat_EC = lat_long_EC[0];
            var long_EC = lat_long_EC[1];
            // Here we also assume that the user wants the output to be height (= - depth):
            Console.Out.WriteLine("Ex2, Pos C: lat, long = "+ _NV.deg(lat_EC) + ", " + _NV.deg(long_EC) + " deg, height = " + (-z_EC) + " m");
            Console.Out.WriteLine();
        }

        // Example 3: ECEF-vector to geodetic latitude
        public void Example3() 
        {
            // Position B is given as p_EB_E ("ECEF-vector")

            var p_EB_E=  Utilities.VecMul(6371e3, new double[] {0.71, -0.72, 0.1}); // m

            // Find position B as geodetic latitude, longitude and height

            // SOLUTION:

            // Find n-vector from the p-vector:
            var n_EB_E_and_z_EB = _NV.p_EB_E2n_EB_E(p_EB_E);
            var n_EB_E = n_EB_E_and_z_EB.Item1;
            var z_EB = n_EB_E_and_z_EB.Item2;

            // Convert to lat, long and height:

            var latlon_EB = _NV.n_E2lat_long(n_EB_E);
            var lat_EB = latlon_EB[0];
            var long_EB = latlon_EB[1];
            var h_EB = -z_EB;

            Console.Out.WriteLine("Ex3, Pos B: lat, long = "+ _NV.deg(lat_EB) + ", " + _NV.deg(long_EB) + " deg, height = " + h_EB + " m");
            Console.Out.WriteLine();
        }

        // Example 4: geodetic latitude to ECEF-vector
        public void Example4() 
        {
            // Position B is given with lat, long and height:
            var lat_EB_deg = 1;
            var long_EB_deg = 2;
            var h_EB = 3;

            // Find the vector p_EB_E ("ECEF-vector")

            // SOLUTION:

            // Step1: Convert to n-vector:
            var n_EB_E = _NV.lat_long2n_E(_NV.rad(lat_EB_deg), _NV.rad(long_EB_deg));

            // Step2: Find the ECEF-vector p_EB_E:
            var p_EB_E = _NV.n_EB_E2p_EB_E(n_EB_E, -h_EB);

            Console.Out.WriteLine("Ex4: p_EB_E = [" + p_EB_E[0] + ", " + p_EB_E[1] + ", " + p_EB_E[2] + "] m");
            Console.WriteLine();
        }

        // Example 5: Surface distance
        public void Example5() 
        { 
            // Position A and B are given as n_EA_E and n_EB_E:
            // Enter elements directly:
            // n_EA_E=unit([1 0 -2]');
            // n_EB_E=unit([-1 -2 0]');

            // or input as lat/long in deg:
            var n_EA_E= _NV.lat_long2n_E(_NV.rad(88), _NV.rad(0));
            var n_EB_E= _NV.lat_long2n_E(_NV.rad(89), _NV.rad(-170));

            var r_Earth=6371e3; // m, mean Earth radius

            // SOLUTION:

            // The great circle distance is given by equation (16) in Gade (2010):
            // Well conditioned for all angles:
            var s_AB= Math.Atan2(Utilities.Norm(Utilities.Cross(n_EA_E,n_EB_E)),Utilities.Dot(n_EA_E,n_EB_E))*r_Earth;

            // % ill conditioned for small angels:
            // s_AB_version1=acos(dot(n_EA_E,n_EB_E))*r_Earth;
            //
            // % ill-conditioned for angles near pi/2 (and not valid above pi/2)
            // s_AB_version2=asin(norm(cross(n_EA_E,n_EB_E)))*r_Earth;

            // The Euclidean distance is given by:
            var d_AB = Utilities.Norm(Utilities.VecSubtract(n_EB_E,n_EA_E) )*r_Earth;

            Console.Out.WriteLine("Ex5, Great circle distance = "+s_AB/1000 + " km, Euclidean distance = " + d_AB/1000+ " km");
            Console.Out.WriteLine();       
        }

        // Example 6: Interpolated position
        public void Example6()
        {
            // Position B is given at time t0 as n_EB_E_t0 and at time t1 as n_EB_E_t1:
            // Enter elements directly:
            // n_EB_E_t0=unit([1 0 -2]');
            // n_EB_E_t1=unit([-1 -2 0]');

            // or input as lat/long in deg:
            var n_EB_E_t0 = _NV.lat_long2n_E(_NV.rad(89.9), _NV.rad(-150));
            var n_EB_E_t1 = _NV.lat_long2n_E(_NV.rad(89.9), _NV.rad(150));

            // The times are given as:
            var t0 = 10.0;
            var t1 = 20.0;
            var ti = 16.0; // time of interpolation

            // Find the interpolated position at time ti, n_EB_E_ti

            // SOLUTION:

            // Using standard interpolation:            
            var n_EB_E_ti = _NV.unit(Utilities.VecAdd(n_EB_E_t0, Utilities.VecMul((ti-t0)/(t1-t0), Utilities.VecSubtract(n_EB_E_t1, n_EB_E_t0))));

            // When displaying the resulting position for humans, it is more convenient
            // to see lat, long:
            var lat_long_EB_ti =_NV.n_E2lat_long(n_EB_E_ti);
            var lat_EB_ti = lat_long_EB_ti[0];
            var long_EB_ti = lat_long_EB_ti[1];
            Console.Out.WriteLine("Ex6, Interpolated position: lat, long = "+ _NV.deg(lat_EB_ti) + ", "+_NV.deg(long_EB_ti)+ " deg");
            Console.Out.WriteLine();
        }

        public void Example7()
        {
            Console.Out.WriteLine("Ex7, Not implemented." );
            Console.Out.WriteLine();
        }

         // Example 8: Position A and azimuth&distance to B
        public void Example8()
        { 
            // Position A is given as n_EA_E:
            // Enter elements directly:
            // n_EA_E=unit([1 0 -2]');

            // or input as lat/long in deg:
            var n_EA_E = _NV.lat_long2n_E(_NV.rad(80), _NV.rad(-90));

            // The initial azimuth and great circle distance (s_AB), and Earth radius
            // (r_Earth) are also given:
            var azimuth = _NV.rad(200);
            var s_AB=1000.0; // m
            var r_Earth=6371e3; // m, mean Earth radius

            // Find the destination point B, as n_EB_E ("The direct/first geodetic
            // problem" for a sphere)

            // SOLUTION:

            // Step1: Find unit vectors for north and east (see equations (9) and (10)
            // in Gade (2010):
            var k_east_E = _NV.unit(Utilities.Cross(Utilities.MatMul(Utilities.Transpose(_NV.R_Ee), new double[3] {1, 0, 0}) ,n_EA_E));
            var k_north_E= Utilities.Cross(n_EA_E,k_east_E);

            // Step2: Find the initial direction vector d_E:
            var d_E = Utilities.VecAdd(Utilities.VecMul(Math.Cos(azimuth), k_north_E), Utilities.VecMul(Math.Sin(azimuth), k_east_E));

            // Step3: Find n_EB_E:
            var n_EB_E = Utilities.VecAdd(Utilities.VecMul(Math.Cos(s_AB/r_Earth), n_EA_E), Utilities.VecMul(Math.Sin(s_AB/r_Earth),d_E));

            // When displaying the resulting position for humans, it is more convenient
            // to see lat, long:
            var lat_long_EB = _NV.n_E2lat_long(n_EB_E);
            var lat_EB = lat_long_EB[0];
            var long_EB = lat_long_EB[1];
            Console.Out.WriteLine("Ex8, Destination: lat, long = "+ _NV.deg(lat_EB) + ", " + _NV.deg(long_EB) + " deg");
            Console.Out.WriteLine();
        }

        // Example 9: Intersection
        public void Example9()
        {         
            // Two paths A and B are given by two pairs of positions:
            // Enter elements directly:
            // n_EA1_E=unit([0 0 1]');
            // n_EA2_E=unit([-1 0 1]');
            // n_EB1_E=unit([-2 -2 4]');
            // n_EB2_E=unit([-2 2 2]');

            // or input as lat/long in deg:
            var n_EA1_E = _NV.lat_long2n_E(_NV.rad(50),_NV.rad(180));
            var n_EA2_E = _NV.lat_long2n_E(_NV.rad(90),_NV.rad(180));
            var n_EB1_E = _NV.lat_long2n_E(_NV.rad(60),_NV.rad(160));
            var n_EB2_E = _NV.lat_long2n_E(_NV.rad(80),_NV.rad(-140));

            // Find the intersection between the two paths, n_EC_E:
            var n_EC_E_tmp = _NV.unit(Utilities.Cross(Utilities.Cross(n_EA1_E,n_EA2_E),Utilities.Cross(n_EB1_E,n_EB2_E)));

            // n_EC_E_tmp is one of two solutions, the other is -n_EC_E_tmp. Select the
            // one that is closest to n_EA1_E, by selecting sign from the dot product
            // between n_EC_E_tmp and n_EA1_E:
            var n_EC_E = Utilities.VecMul(Math.Sign(Utilities.Dot(n_EC_E_tmp,n_EA1_E)), n_EC_E_tmp);

            // When displaying the resulting position for humans, it is more convenient
            // to see lat, long:
            var lat_EC = _NV.n_E2lat(n_EC_E);
            var long_EC = _NV.n_E2long(n_EC_E);
            Console.Out.WriteLine("Ex9, Intersection: lat, long = "+_NV.deg(lat_EC)+ ", "+_NV.deg(long_EC)+" deg");
            Console.Out.WriteLine();
        }

        // Example 10: Cross track distance
        public void Example10()
        {             

            // Position A1 and A2 and B are given as n_EA1_E, n_EA2_E, and n_EB_E:
            // Enter elements directly:
            // n_EA1_E=unit([1 0 -2]');
            // n_EA2_E=unit([-1 -2 0]');
            // n_EB_E=unit([0 -2 3]');

            // or input as lat/long in deg:
            var n_EA1_E = _NV.lat_long2n_E(_NV.rad(0),_NV.rad(0));
            var n_EA2_E = _NV.lat_long2n_E(_NV.rad(10),_NV.rad(0));
            var n_EB_E = _NV.lat_long2n_E(_NV.rad(1),_NV.rad(0.1));

            var r_Earth=6371e3; // m, mean Earth radius

            // Find the cross track distance from path A to position B.

            // SOLUTION:

            // Find the unit normal to the great circle between n_EA1_E and n_EA2_E:
            var c_E = _NV.unit(Utilities.Cross(n_EA1_E,n_EA2_E));

            // Find the great circle cross track distance:
            var s_xt = (Math.Acos(Utilities.Dot(c_E,n_EB_E))-Math.PI/2)*r_Earth;

            // Find the Euclidean cross track distance:
            var d_xt= -Utilities.Dot(c_E,n_EB_E)*r_Earth;

            Console.Out.WriteLine("Ex10, Cross track distance = "+ s_xt + " m, Euclidean = "+ d_xt+ " m");
            Console.Out.WriteLine();
        }

    }
}
