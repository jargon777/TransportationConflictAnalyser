/* This code takes a CSV file with points and process it. 
 * It needs the DotSpatial library to run (not included with this file). 
 * If you want to see this code compile You'll need to comment out bunches of code (but then it won't do anything!)
 * 
THIS PROGRAM IS RELEASED UNDER:
    The MIT License (MIT)

    Copyright (c) 2012 by Matthew Muresan, mimuresa@uwaterloo.ca

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
 */

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.IO; /*ensure that this library is included, or StreamWriter/StreamReader won't work. If they aren't, not sure but I think you can still call them by going System.IO.StreamWriter etc :P*/
using System.ComponentModel;
using System.Data;
using System.Drawing;


namespace Transportation_Project {
    /// <summary>
    /// Struct to hold writable data.
    /// </summary>
    public struct DETECTORCRASHPOTENTIALS {
        public int[] DetectorTTCvehicleCount;
        public double[] DetectorTTCaverageTTC;
        public double[] DetectorTTCtotalExposure;

        public int[] DetectorDRACvehicleCount;
        public double[] DetectorDRACtotalExposure;

        public int[] DetectorPSDvehicleCount;

        public int[] DetectorCPIvehicleCount;
        public double[] DetectorCPIscaledTotal;
        public double[] DetectorCPItotalExposedTime;

        public double[] DetectorTotalExposure;
        public double[] DetectorTotalVehicles;

        /// <summary>
        /// Write_Element's constructor
        /// </summary>
        /// <param name="id">ID of the vehicle read</param>
        /// <param name="path_start">The trajectory start of the vehicle</param>
        /// <param name="path_end">The trajectory end of the vehicle</param>
        /// <param name="ttc">Time to Crash</param>
        /// <param name="tit">Time Integrated TTC</param>
        /// <param name="drac">Deceleration</param>
        /// <param name="cpi">Crash Potential Index</param>
        public DETECTORCRASHPOTENTIALS(int detector_total) {
            this.DetectorTTCaverageTTC = new double[detector_total];
            this.DetectorTTCtotalExposure = new double[detector_total];
            this.DetectorTTCvehicleCount = new int[detector_total];

            this.DetectorDRACvehicleCount = new int[detector_total];
            this.DetectorDRACtotalExposure = new double[detector_total];

            this.DetectorPSDvehicleCount = new int[detector_total];

            this.DetectorCPIvehicleCount = new int[detector_total];
            this.DetectorCPIscaledTotal = new double[detector_total];
            this.DetectorCPItotalExposedTime = new double[detector_total];

            this.DetectorTotalExposure = new double[detector_total];
            this.DetectorTotalVehicles = new double[detector_total];
        }
        public void TTCvaluesAppend(int detector, double TTCexposureduration, double TTCtime) {
            this.DetectorTTCvehicleCount[detector]++;
            this.DetectorTTCtotalExposure[detector] += TTCexposureduration;
            this.DetectorTTCaverageTTC[detector] += (TTCtime - this.DetectorTTCaverageTTC[detector]) / DetectorTTCvehicleCount[detector];
        }
        public void DRACvaluesAppend(int detector, double DRACexposureduration) {
            this.DetectorDRACvehicleCount[detector]++;
            this.DetectorDRACtotalExposure[detector] += DRACexposureduration;
        }
        public void PSDvaluesAppend(int detector) {
            this.DetectorPSDvehicleCount[detector]++;
        }
        public void CPIvaluesAppend(int detector, double CPIscaledtime, double CPIexposuretime) {
            this.DetectorCPIvehicleCount[detector]++;
            this.DetectorCPItotalExposedTime[detector] += CPIexposuretime;
            this.DetectorCPIscaledTotal[detector] += CPIscaledtime;
        }
    }

    /* Centroid data struct */
    public struct TRAJECTORY {
        public List<int> Detectors;
        public int? start;
        public int? end;

        /// <summary>
        /// Multi-Element Constructor
        /// </summary>
        /// <param name="detectors">List of points</param>
        public TRAJECTORY(List<int> detectors) {
            this.Detectors = detectors;
            if (detectors.Count != 0) {
                this.start = detectors.First();
                this.end = detectors.Last();
            }
            else {
                this.start = null;
                this.end = null;
                Console.WriteLine("WARNING: Trajectory with null starts and ends created. Is this okay?");
                Console.ReadLine();
            }
        }
        /// <summary>
        /// Single-Element Constructor
        /// </summary>
        /// <param name="detector">ID for the first element in the constructor</param>
        public TRAJECTORY(int detector) {
            this.Detectors = new List<int>();
            this.start = detector;
            this.end = detector;
            this.Detectors.Add(detector);
        }
    }

    public struct CRASHPOTENTIALS {
        public double TIT;
        public double TTCexposureTime;
        public double DRACexposureTime;
        public double CPIindex;
        public CRASHPOTENTIALS(double ttcexposuretime, double tit, double DRACexposuretime) {
            this.TIT = tit;
            this.TTCexposureTime = ttcexposuretime;
            this.DRACexposureTime = DRACexposuretime;
            this.CPIindex = 0;

        }
        public CRASHPOTENTIALS(bool run) {
            this.TTCexposureTime = 0;
            this.TIT = 0;
            this.DRACexposureTime = 0;
            this.CPIindex = 0;
        }
    }


    /// <summary>
    /// Vehicle data read from CSV
    /// </summary>
    public class VEHICLE {
        public int Veh_id;
        public double Length;
        public double[] Transit_Time;
        public double[] Speed;
        public double[] Accel;
        public TRAJECTORY Veh_Traj;
        public CRASHPOTENTIALS CrashPotentials;
        private const int MAXIMUM_DETECTORS = 415;

        /// <summary>
        /// The constructor
        /// </summary>
        /// <param name="veh_id">Vehicle ID</param>
        /// <param name="t_time">Transit Time</param>
        /// <param name="speed"></param>
        /// <param name="accel"></param>
        /// <param name="veh_traj">Vehicle Trjectory</param>
        public VEHICLE(int veh_id, double[] t_time, double[] speed, double[] accel, TRAJECTORY veh_traj, double length, CRASHPOTENTIALS crashpotentials) {
            this.Veh_id = veh_id;
            this.Transit_Time = accel;
            this.Speed = speed;
            this.Accel = accel;
            this.Veh_Traj = veh_traj;
            this.Length = length;
            this.CrashPotentials = crashpotentials;
        }
        /// <summary>
        /// Constructor for trajectory only
        /// </summary>
        /// <param name="veh_id">Vehicle ID</param>
        /// <param name="veh_traj">Vehicle Trjectory</param>
        public VEHICLE(int veh_id, TRAJECTORY veh_traj) {
            this.Veh_id = veh_id;
            this.Transit_Time = new double[MAXIMUM_DETECTORS];
            this.Speed = new double[MAXIMUM_DETECTORS];
            this.Accel = new double[MAXIMUM_DETECTORS];
            this.Veh_Traj = veh_traj;
            this.Length = 4.47;
            this.CrashPotentials = new CRASHPOTENTIALS(true);
        }
    }

    class Program {
        const int detector_total = 415;
        /*********************/
        /*** LOAD THE DATA ***/
        /*********************/
        /* Declare a list to hold the variables that I will be using. Lists are lists of my structs */
        public static List<VEHICLE> Vehicles = new List<VEHICLE>();

        //data on the detectors
        public static DETECTORCRASHPOTENTIALS Detectors = new DETECTORCRASHPOTENTIALS(detector_total);

        /* Since this program is a one-shot and my use type only, I saved my paths as variables and modified them in my code */
        private static List<string> WriteCSVs = new List<string>();
        private static List<string> ReadCSVs = new List<string>();
        private static string TrajectoryCSVs = "input/TRAJ.csv";

        static void Main(string[] args)
        {
            ReadCSVs.Add("input/t_time.csv");
            ReadCSVs.Add("input/speed.csv");
            ReadCSVs.Add("input/accel.csv");

            WriteCSVs.Add("output/vehicles.csv");
            WriteCSVs.Add("output/detectors.csv");
            
            Console.WriteLine("Checking if file output file exists and writing header... ");
            _WriteHeader();
            Console.WriteLine("Done.");

            Console.WriteLine("Parsing Vehicle Trajectory Data... ");
            _ReadTrajectory();
            Console.WriteLine("Done.");

            Console.Write("Parsing Time, Speed, and Acceleration Data... ");
            _ReadVehicleStats();
            Console.WriteLine("Done. ");


            /****************************/
            /***** PERFORM CALCULATIONS */
            /****************************/
            Console.WriteLine("Beginning ALL calculations... ");
            _CalculateOutputs();
            Console.WriteLine("Done ");

            Console.WriteLine("Writing Output to file...");
            _WriteData();
            Console.WriteLine("Done ");

            Console.ReadLine();
        }

        //Private functions to keep the main method clean.
        //Reading Methods
        private static void _WriteHeader() {
            for (int i = 0; i < WriteCSVs.Count; i++) {
                using (StreamWriter writer = new StreamWriter(WriteCSVs[i], false)) {
                    if (i == 0) {
                        writer.WriteLine("Multi-Function Transportation Assignment 3 Tool Output, Vehicles");
                        writer.WriteLine("VehicleID,TrajectoryID,TrajectoryStart,TrajectoryEnd,TTCexposure,DRACexposure,CPIindex");
                    }
                    else if (i == 1) {
                        writer.WriteLine("Multi-Function Transportation Assignment 3 Tool Output, Detectors");
                        writer.WriteLine("DetectorID,TTCinteractions, TTCexposure, DRACinteractions, DRACtotalExposure, CPIintereactions, CPIscaledexposure, CPItotalexposure, PSDinteractions, TotalVehicles, TotalExposure");
                    }
                }
            }
        }
        private static void _ReadTrajectory() {
            using (StreamReader readFile = new StreamReader(TrajectoryCSVs)) {
                readFile.ReadLine(); //skip first line of the file, it's the header in the CSV that just contains description. 
                string line; //data from the line
                string[] rowdata; //data split from "line".

                line = readFile.ReadLine(); //read the trajectory count
                rowdata = line.Split(','); //This splits the data from 'line'

                readFile.ReadLine(); //void line

                line = readFile.ReadLine(); //read the sums
                rowdata = line.Split(',');

                readFile.ReadLine(); //Column Headers line

                while ((line = readFile.ReadLine()) != null) {
                    rowdata = line.Split(','); //This splits the data from 'line' by comma and populates my array with it.
                    List<int> nodes = new List<int>();
                    TRAJECTORY Curr_Traj;
                    int total_vehs = int.Parse(rowdata[3]);
                    int total_nodes_in_path = int.Parse(rowdata[2]);
                    for (int i = 5; i < total_nodes_in_path + 5; i++) {
                        nodes.Add(int.Parse(rowdata[i]));
                        Console.WriteLine("Found Node " + rowdata[i]);
                    }
                    Curr_Traj = new TRAJECTORY(nodes);

                    //read the vehicles using this trajectory
                    line = readFile.ReadLine(); //next line has vehicles
                    rowdata = line.Split(','); //This splits the data from 'line' by comma and populates my array with it.
                    for (int i = 5; i < total_vehs + 5; i++) {
                        Console.WriteLine("Found Vehicle " + rowdata[i] + " on Path {0}-{1}", Curr_Traj.start, Curr_Traj.end);
                        bool pass_fail = true;
                        foreach (VEHICLE vehicle in Vehicles) {
                            if (vehicle.Veh_id == int.Parse(rowdata[i])) {
                                pass_fail = false;
                                Console.WriteLine("Vehicle is Duplicate! Is this Okay?");
                                Console.ReadLine();
                                break;
                            }
                        }
                        if (pass_fail) Vehicles.Add(new VEHICLE(int.Parse(rowdata[i]), Curr_Traj));
                    }
                }
            }
        }
        private static void _ReadVehicleStats() {
            foreach (string path in ReadCSVs) {
                using (StreamReader readFile = new StreamReader(path)) {
                    string line; //I declare a string here to hold the data from a "ReadLine" call.
                    string[] rowdata; //Because CSV data is sparated by commas, and because I want the data in each column I declare an array that will hold this data.
                    List<int> column_header = new List<int>();
                    int tmp;
                    double tmpd;
                    line = readFile.ReadLine(); //read the header
                    rowdata = line.Split(',');
                    foreach (string element in rowdata) {
                        if (int.TryParse(element, out tmp)) column_header.Add(tmp);
                        else column_header.Add(-1);
                    }

                    while ((line = readFile.ReadLine()) != null) {
                        //row data columns are ordered as nodeid,x,y
                        rowdata = line.Split(','); //This splits the data from 'line' by comma and populates my array with it.
                        //iterate over all vehicles saved
                        foreach (VEHICLE vehicle in Vehicles) {
                            int.TryParse(rowdata[0], out tmp);
                            //if the line parsed mathches a vehicle in the list the data is to be associated with it
                            if (vehicle.Veh_id == tmp) {
                                //iterate over the vehicle's trajectory to pull the transit time
                                foreach (int detector in vehicle.Veh_Traj.Detectors) {
                                    int detector_col = -1;
                                    //iterate over the column headers to find the location of the correct header in the csv
                                    for (int i = 0; i < column_header.Count - 1; i++) {
                                        if (column_header[i] == (detector)) { detector_col = i; break; }
                                    }
                                    //associate the data from that column with the auto.
                                    if (double.TryParse(rowdata[detector_col], out tmpd)) {
                                        if (path == "input/t_time.csv") vehicle.Transit_Time[detector] = tmpd;
                                        if (path == "input/speed.csv") vehicle.Speed[detector] = tmpd / 3.6; //convert it to m/s
                                        if (path == "input/accel.csv") vehicle.Accel[detector] = tmpd;
                                    }
                                    else {
                                        if (path == "input/t_time.csv") vehicle.Transit_Time[detector] = 0;
                                        if (path == "input/speed.csv") vehicle.Speed[detector] = 0;
                                        if (path == "input/accel.csv") vehicle.Accel[detector] = 0;
                                        Console.WriteLine("Failed to parse a Value on Vehicle {0} Node {1}. Check this!", vehicle.Veh_id, detector_col);
                                        //This warning is expected on speed and acceleration as the first column is empty.
                                        //Console.ReadLine();
                                    }
                                }
                                break;
                            }
                        }
                    }
                }
            }
        }

        //Data Processing Methods
        private static void _CalculateOutputs() {
            /* For vehicle... 
             * iterate through the trajectory... 
             * for each trajectory point... 
             * find a vehicle that follows (i.e. passes that point)... 
             * determine where that vehicle is at the the time intially observed... */

            //iterate through all the vehicles
            foreach (VEHICLE vehicleLeading in Vehicles) {
                int previousDetectorLeading = 0;
                int? closestVehicle = null;
                double distanceFromAtTime = 0;
                double speedAtTime = 0;

                const double TTCthreshold = 1.5;
                double TITrunning = 0;
                double TTCforCurDetector = 0;
                double TTCrunningTmBlThrs = 0;
                bool TTCisTimingBlThrsh = false;

                const double DRACthreshold = 3.35;
                double DRACforCurDetector = 0;
                double DRACrunningTmBlThrs = 0;
                bool DRACisTimingBlThrsh = false;

                const double CPIthresholdMADR = 3.35; //MADR
                double CPIrunningTmBlThrs = 0;
                bool CPIisTimingBlThrsh = false;
                double CPIrunningScaledTotal = 0;

                double PSDdistanceMSD;

                int skip_detectors = 0;
                const int DETECTORS_TO_SKIP = 1;

                //look at the trajectory that vehicle is on
                foreach (int detectorLeading in vehicleLeading.Veh_Traj.Detectors) {
                    //skip some of the first few detectors. This is mostly used in testing, however the first five detectors should always be skipped (length of vehicle is 4.5).
                    if (skip_detectors < DETECTORS_TO_SKIP) {
                        previousDetectorLeading = detectorLeading;
                        skip_detectors++; 
                        continue;
                    }

                    //iterate through all the vehicles again to find a following vehicle
                    foreach (VEHICLE vehicleFollowing in Vehicles) {
                        if (vehicleFollowing.Veh_id == vehicleLeading.Veh_id) continue; //a vehicle cannot follow itself.
                        //iterate through its trajectory and see if we can find a match.
                        for (int i = 0; i < vehicleFollowing.Veh_Traj.Detectors.Count; i++ ) {
                            int detectorFollowing = vehicleFollowing.Veh_Traj.Detectors[i];
                            //If this vehicle passes the point we're looking at, then it could be in conflict with our "leading vehicle"
                            if (detectorLeading == detectorFollowing) {
                                //If the followingVehicle's transit time is greater than the leadingVehicle's transit time, then that means it is following it
                                if (vehicleLeading.Transit_Time[detectorLeading] <= vehicleFollowing.Transit_Time[detectorFollowing]) {
                                    //iterate backwards along the following vehicle's trajectory to find out if it is close enough to be a problem
                                    for (int ii = i - 1; ii >= 0; ii--) {
                                        //reassign detector following as we iterate backwards
                                        detectorFollowing = vehicleFollowing.Veh_Traj.Detectors[ii];
                                        //when we find the point in time that is less than the current point in time, then the following vehicle is a known distance before that point.
                                        if (vehicleFollowing.Transit_Time[detectorFollowing] <= vehicleLeading.Transit_Time[detectorLeading]) {
                                            //check if this is the first following vehicle identified
                                            if (closestVehicle != null) {
                                                double _distanceFromAtTime = (i - ii);
                                                double _timeBetweenDetectors = vehicleFollowing.Transit_Time[vehicleFollowing.Veh_Traj.Detectors[ii + 1]] - vehicleFollowing.Transit_Time[vehicleFollowing.Veh_Traj.Detectors[ii]];
                                                double _timePassedSinceDetector = vehicleLeading.Transit_Time[detectorLeading] - vehicleFollowing.Transit_Time[detectorFollowing];
                                                double _scaleFactor = _timePassedSinceDetector / _timeBetweenDetectors;
                                                _distanceFromAtTime += _scaleFactor;
                                                if (distanceFromAtTime > _distanceFromAtTime) {
                                                    closestVehicle = vehicleFollowing.Veh_id;
                                                    distanceFromAtTime = _distanceFromAtTime;

                                                    double _speedDifference = vehicleFollowing.Speed[vehicleFollowing.Veh_Traj.Detectors[ii + 1]] - vehicleFollowing.Speed[detectorFollowing];
                                                    speedAtTime = vehicleFollowing.Speed[detectorFollowing];
                                                    speedAtTime += _scaleFactor * _speedDifference; //adjust speed, assume linear de/acelleration between the two points.
                                                }
                                                break;
                                            }
                                            else {
                                                closestVehicle = vehicleFollowing.Veh_id;
                                                //the difference between i, the position where the conflict will happen, and ii, the current position is the distance in metres, as detectors are 1m apart.
                                                distanceFromAtTime = (i - ii);
                                                //a compensationfactor is required given that the vehicle may be in-between two detectors. Assume no accelleration. detectorFollowing is detectors[ii] of following. ii+1 is the next one.
                                                double _timeBetweenDetectors = vehicleFollowing.Transit_Time[vehicleFollowing.Veh_Traj.Detectors[ii + 1]] - vehicleFollowing.Transit_Time[vehicleFollowing.Veh_Traj.Detectors[ii]];
                                                double _timePassedSinceDetector = vehicleLeading.Transit_Time[detectorLeading] - vehicleFollowing.Transit_Time[detectorFollowing];
                                                double _scaleFactor = _timePassedSinceDetector / _timeBetweenDetectors;
                                                distanceFromAtTime += _scaleFactor;

                                                double _speedDifference = vehicleFollowing.Speed[vehicleFollowing.Veh_Traj.Detectors[ii + 1]] - vehicleFollowing.Speed[detectorFollowing];
                                                speedAtTime = vehicleFollowing.Speed[detectorFollowing];
                                                speedAtTime += _scaleFactor * _speedDifference; //adjust speed, assume linear de/acelleration between the two points.

                                                break;
                                            }
                                        }
                                    }
                                }
                                break; //even if a time could not be found, no need to continue looping through the rest of the path
                            }
                        }
                    }
                    if (closestVehicle != null) {
                        double _separationDistance = Math.Abs(distanceFromAtTime - vehicleLeading.Length);
                        double _speedDifferential = speedAtTime - vehicleLeading.Speed[detectorLeading];
                        double _proportionSpeedDifferential = 0.5;
                        if (_speedDifferential < 0) {
                            /*if the speed differnetial is negative, then the vehicles are now moving apart. This proportion
                             * represents the time where the vehicles were moving together. This assumes constant acceleration.
                             * Absolute value is take because there are a few odd cases where this will not help. In these cases
                             * the speed from the previous detector is actually the same as the current one. In this case, it is
                             * presumed that, even this guess is better than specifying a constant. If any constant should be
                             * specified, perhaps 0.5 is the best guess, or 0 to just ignore the case?
                             */ 
                            _proportionSpeedDifferential = Math.Abs((speedAtTime - vehicleLeading.Speed[previousDetectorLeading]) / 
                                (vehicleLeading.Speed[detectorLeading] - vehicleLeading.Speed[previousDetectorLeading]));
                            if (_proportionSpeedDifferential > 1) _proportionSpeedDifferential = 0.5; //for the really strange cases?
                            
                        }

                        /******************************* TTC CALCULATIONS **********************************************/
                        //determine the distance between vehicles, factoring for length, divide that by the speed differential. Store difference to new TTC
                        double _previousTTC = TTCforCurDetector;
                        double? _TTCrunningTmBlThrsIncrement = null;
                        double _TTCcmaVAL = 0;
                        TTCforCurDetector = _separationDistance / _speedDifferential;

                        if (vehicleLeading.Speed[detectorLeading] > speedAtTime) {
                            if (TTCisTimingBlThrsh) {
                                /* if a TTC timer was runing before, this means that the speed differential has now shifted such that they are moving apart.
                                 * Accordingly, use _proportionSpeedDifferential to scale the results.
                                 */
                                _TTCrunningTmBlThrsIncrement = (vehicleLeading.Transit_Time[detectorLeading] - vehicleLeading.Transit_Time[previousDetectorLeading]) * _proportionSpeedDifferential;
                                _TTCcmaVAL = ((TTCthreshold + _previousTTC) / 2);
                                TTCisTimingBlThrsh = false;
                            }
                        }
                        else {
                            if (TTCforCurDetector < TTCthreshold) {
                                //if the TTC is below the threshold, check to see if it was below the threshold before.
                                if (TTCisTimingBlThrsh) {
                                    _TTCrunningTmBlThrsIncrement = vehicleLeading.Transit_Time[detectorLeading] - vehicleLeading.Transit_Time[previousDetectorLeading];
                                    _TTCcmaVAL = (TTCforCurDetector + _previousTTC) / 2;
                                }
                                else {
                                    TTCisTimingBlThrsh = true;
                                    _TTCcmaVAL = (TTCthreshold + TTCforCurDetector) / 2;
                                    
                                    //assume vehicle was below TTC threshold for the _proportion value.
                                    if (_previousTTC < 0) _TTCrunningTmBlThrsIncrement = (vehicleLeading.Transit_Time[detectorLeading] - vehicleLeading.Transit_Time[previousDetectorLeading]) * _proportionSpeedDifferential;
                                    else {
                                        //technically this should always be true, except on the first run of the loop.
                                        if (_previousTTC > TTCforCurDetector) {
                                            double _TTCscaleFactor = (TTCthreshold - TTCforCurDetector) / (_previousTTC - TTCforCurDetector);
                                            _TTCrunningTmBlThrsIncrement = (vehicleLeading.Transit_Time[detectorLeading] - vehicleLeading.Transit_Time[previousDetectorLeading]) * _TTCscaleFactor;
                                        }
                                    }
                                }
                            }
                            else {
                                //if a TTC timer had been run before, but the value is no longer below threshold, increment it for an estimated proportion below the TTC.
                                if (TTCisTimingBlThrsh) {
                                    double _TTCscaleFactor = (TTCthreshold - _previousTTC) / (TTCforCurDetector - _previousTTC);
                                    _TTCrunningTmBlThrsIncrement = (vehicleLeading.Transit_Time[detectorLeading] - vehicleLeading.Transit_Time[previousDetectorLeading]) * _TTCscaleFactor;
                                    _TTCcmaVAL = (TTCthreshold + _previousTTC) / 2;
                                    TTCisTimingBlThrsh = false; //since we've exceeded the TTC, we're not timing the duration below the TTC anymore.
                                }
                            }
                        }

                        /***************************** DRAC CALCULATIONS ***********************************************/
                        //determine the distance between vehicles, factoring for length, divide that by the speed differential. Store difference to new DRAC
                        double _previousDRAC = DRACforCurDetector;
                        double? _DRACrunningTmBlThrsIncrement = null;
                        DRACforCurDetector = Math.Pow(_speedDifferential, 2) / _separationDistance;
                        if (_separationDistance == 0) {
                            Console.WriteLine("WARNING: The separation distance is ZERO!");
                            DRACforCurDetector = Math.Pow(_speedDifferential, 2) / 0.01; //arbitrarily assign a small distance.
                        }
                        //obviously there is no DRAC for a case where the lead vehicle is faster.
                        if (vehicleLeading.Speed[detectorLeading] > speedAtTime) {
                            if (DRACisTimingBlThrsh) {
                                /* if a DRAC timer was runing before, this means that the speed differential has now shifted such that the conflict no longer exists.
                                 * The differential has also shifted to the point where the vehicles are no longer moving towards each other. 
                                 */
                                _DRACrunningTmBlThrsIncrement = (vehicleLeading.Transit_Time[detectorLeading] - vehicleLeading.Transit_Time[previousDetectorLeading]) * _proportionSpeedDifferential;
                                DRACisTimingBlThrsh = false;
                            }
                        }
                        else {
                            //unlike TTC, for DRAC, bad exposure happens when the decelleration rate needed exceeds the sa
                            if (DRACforCurDetector > DRACthreshold) {
                                //if the DRAC is below the threshold, check to see if it was below the threshold before.
                                if (DRACisTimingBlThrsh) {
                                    _DRACrunningTmBlThrsIncrement = vehicleLeading.Transit_Time[detectorLeading] - vehicleLeading.Transit_Time[previousDetectorLeading];
                                }
                                else {
                                    DRACisTimingBlThrsh = true;

                                    //assume vehicle was below DRAC threshold for half of the time before if the value was negative (i.e. differential speeds in excess)
                                    if (_previousDRAC < 0) _DRACrunningTmBlThrsIncrement = (vehicleLeading.Transit_Time[detectorLeading] - vehicleLeading.Transit_Time[previousDetectorLeading]) * _proportionSpeedDifferential;
                                    else {
                                        //technically this should always be true, except on the first run of the loop.
                                        if (_previousDRAC > DRACforCurDetector) {
                                            double _DRACscaleFactor = (DRACthreshold - DRACforCurDetector) / (_previousDRAC - DRACforCurDetector);
                                            _DRACrunningTmBlThrsIncrement = (vehicleLeading.Transit_Time[detectorLeading] - vehicleLeading.Transit_Time[previousDetectorLeading]) * _DRACscaleFactor;
                                        }
                                    }
                                }
                            }
                            else {
                                //if a DRAC timer had been run before, but the value is no longer below threshold, increment it for an estimated proportion below the DRAC.
                                if (DRACisTimingBlThrsh) {
                                    double _DRACscaleFactor = (DRACthreshold - _previousDRAC) / (DRACforCurDetector - _previousDRAC);
                                    _DRACrunningTmBlThrsIncrement = (vehicleLeading.Transit_Time[detectorLeading] - vehicleLeading.Transit_Time[previousDetectorLeading]) * _DRACscaleFactor;
                                    DRACisTimingBlThrsh = false; //since we've exceeded the DRAC, we're not timing the duration below the DRAC anymore.
                                }
                            }
                        }

                        /***************************** PSD CALCULATIONS ************************************************/
                        PSDdistanceMSD = Math.Pow(_speedDifferential, 2) / DRACthreshold;
                        if (_separationDistance < PSDdistanceMSD) {
                            //PSD is only a count. So just add it directly to the detector matrix.
                            Detectors.PSDvaluesAppend(detectorLeading);
                        }

                        /***************************** CPI CALCULATIONS ************************************************/
                        //determine the distance between vehicles, factoring for length, divide that by the speed differential. Store difference to new DRAC
                        double? _CPIrunningTmBlThrsIncrement = null;
                        //obviously there is no DRAC for a case where the lead vehicle is faster.
                        if (vehicleLeading.Speed[detectorLeading] > speedAtTime) {
                            if (CPIisTimingBlThrsh) {
                                /* SEE TTC FOR EXPLANATIONS ON THIS LOGIC.
                                 */
                                _CPIrunningTmBlThrsIncrement = (vehicleLeading.Transit_Time[detectorLeading] - vehicleLeading.Transit_Time[previousDetectorLeading]) * _proportionSpeedDifferential;
                                CPIisTimingBlThrsh = false;
                            }
                        }
                        else {
                            if (DRACforCurDetector > CPIthresholdMADR) {
                                //if the DRAC is below the threshold, check to see if it was below the threshold before.
                                if (CPIisTimingBlThrsh) {
                                    _CPIrunningTmBlThrsIncrement = vehicleLeading.Transit_Time[detectorLeading] - vehicleLeading.Transit_Time[previousDetectorLeading];
                                }
                                else {
                                    CPIisTimingBlThrsh = true;

                                    //assume vehicle was below DRAC MADR for half of the time before if the value was negative (i.e. differential speeds in excess)
                                    if (_previousDRAC < 0) _CPIrunningTmBlThrsIncrement = (vehicleLeading.Transit_Time[detectorLeading] - vehicleLeading.Transit_Time[previousDetectorLeading]) * _proportionSpeedDifferential;
                                    else {
                                        //technically this should always be true, except on the first run of the loop.
                                        if (_previousDRAC > DRACforCurDetector) {
                                            double _CPIscaleFactor = (CPIthresholdMADR - DRACforCurDetector) / (_previousDRAC - DRACforCurDetector);
                                            _CPIrunningTmBlThrsIncrement = (vehicleLeading.Transit_Time[detectorLeading] - vehicleLeading.Transit_Time[previousDetectorLeading]) * _CPIscaleFactor;
                                        }
                                    }
                                }
                            }
                            else {
                                //if a CPI timer had been run before, but the value is no longer below threshold, increment it for an estimated proportion below the DRAC.
                                if (CPIisTimingBlThrsh) {
                                    double _CPIscaleFactor = (CPIthresholdMADR - _previousDRAC) / (DRACforCurDetector - _previousDRAC);
                                    _CPIrunningTmBlThrsIncrement = (vehicleLeading.Transit_Time[detectorLeading] - vehicleLeading.Transit_Time[previousDetectorLeading]) * _CPIscaleFactor;
                                    CPIisTimingBlThrsh = false; //since we've exceeded the MADR, we're not timing the duration below the CPI anymore.
                                }
                            }
                        }

                        /***************************** APPEND THE DATA TO THE DETECTOR AND VEHICLE**********************/
                        if (_TTCrunningTmBlThrsIncrement != null) {
                            TTCrunningTmBlThrs += Convert.ToDouble(_TTCrunningTmBlThrsIncrement);
                            TITrunning += TTCthreshold * Convert.ToDouble(_TTCrunningTmBlThrsIncrement) 
                                - (TTCforCurDetector * Convert.ToDouble(_TTCrunningTmBlThrsIncrement));
                            Detectors.TTCvaluesAppend(detectorLeading, Convert.ToDouble(_TTCrunningTmBlThrsIncrement), _TTCcmaVAL);
                        }
                        if (_DRACrunningTmBlThrsIncrement != null) {
                            DRACrunningTmBlThrs += Convert.ToDouble(_DRACrunningTmBlThrsIncrement);
                            TITrunning += DRACthreshold * Convert.ToDouble(_DRACrunningTmBlThrsIncrement)
                                - (DRACforCurDetector * Convert.ToDouble(_DRACrunningTmBlThrsIncrement));
                            Detectors.DRACvaluesAppend(detectorLeading, Convert.ToDouble(_DRACrunningTmBlThrsIncrement));
                        }
                        if (_CPIrunningTmBlThrsIncrement != null) {
                            CPIrunningScaledTotal += Convert.ToDouble(_CPIrunningTmBlThrsIncrement);
                            Detectors.CPIvaluesAppend(detectorLeading, CPIrunningScaledTotal, vehicleLeading.Transit_Time[detectorLeading] - vehicleLeading.Transit_Time[previousDetectorLeading]);
                        }

                    }
                    Detectors.DetectorTotalVehicles[detectorLeading]++;
                    Detectors.DetectorTotalExposure[detectorLeading] += vehicleLeading.Transit_Time[detectorLeading]
                        - vehicleLeading.Transit_Time[previousDetectorLeading];

                    closestVehicle = null; //reset the closest vehicle
                    previousDetectorLeading = detectorLeading; //identifies previous detector
                }
                //done one vehicle.
                vehicleLeading.CrashPotentials.TTCexposureTime = TTCrunningTmBlThrs;
                vehicleLeading.CrashPotentials.TIT = TITrunning;
                vehicleLeading.CrashPotentials.DRACexposureTime = DRACrunningTmBlThrs;
                if ((vehicleLeading.Transit_Time[Convert.ToInt32(vehicleLeading.Veh_Traj.end)] - vehicleLeading.Transit_Time[Convert.ToInt32(vehicleLeading.Veh_Traj.start)]) != 0) {
                    vehicleLeading.CrashPotentials.CPIindex = CPIrunningScaledTotal
                        / (vehicleLeading.Transit_Time[Convert.ToInt32(vehicleLeading.Veh_Traj.end)]
                        - vehicleLeading.Transit_Time[Convert.ToInt32(vehicleLeading.Veh_Traj.start)]);
                }
            }
        }

        //Data Output Methods
        private static void _WriteData() {
            for (int i = 0; i < WriteCSVs.Count; i++) {
                using (StreamWriter writer = new StreamWriter(WriteCSVs[i], true)) {
                    const char delimiter = ',';
                    if (i == 0) {
                        //writer.WriteLine("VehicleID,TrajectoryID,TrajectoryStart,TrajectoryEnd,TTCexposure,DRACexposure,CPIindex");
                        foreach (VEHICLE vehicle in Vehicles) {
                            writer.Write(vehicle.Veh_id);
                            writer.Write(delimiter);

                            writer.Write(vehicle.Veh_Traj.start);
                            writer.Write("-");
                            writer.Write(vehicle.Veh_Traj.end);
                            writer.Write(delimiter);

                            writer.Write(vehicle.Veh_Traj.start);
                            writer.Write(delimiter);

                            writer.Write(vehicle.Veh_Traj.end);
                            writer.Write(delimiter);

                            writer.Write(vehicle.CrashPotentials.TTCexposureTime);
                            writer.Write(delimiter);

                            writer.Write(vehicle.CrashPotentials.DRACexposureTime);
                            writer.Write(delimiter);

                            writer.Write(vehicle.CrashPotentials.CPIindex);

                            writer.WriteLine();
                        }
                    }
                    if (i == 1) {
                        //writer.WriteLine("DetectorID,TTCinteractions, TTCexposure, DRACinteractions, DRACtotalExposure, CPIintereactions, CPIscaledexposure, CPItotalexposure, PSDinteractions, TotalVehicles, TotalExposure");
                        for (int ii = 0; ii < detector_total; ii++) {
                            writer.Write(ii);
                            writer.Write(delimiter);

                            writer.Write(Detectors.DetectorTTCvehicleCount[ii]);
                            writer.Write(delimiter);

                            writer.Write(Detectors.DetectorTTCtotalExposure[ii]);
                            writer.Write(delimiter);

                            writer.Write(Detectors.DetectorDRACvehicleCount[ii]);
                            writer.Write(delimiter);

                            writer.Write(Detectors.DetectorDRACtotalExposure[ii]);
                            writer.Write(delimiter);

                            writer.Write(Detectors.DetectorCPIvehicleCount[ii]);
                            writer.Write(delimiter);

                            writer.Write(Detectors.DetectorCPIscaledTotal[ii]);
                            writer.Write(delimiter);

                            writer.Write(Detectors.DetectorCPItotalExposedTime[ii]);
                            writer.Write(delimiter);

                            writer.Write(Detectors.DetectorPSDvehicleCount[ii]);
                            writer.Write(delimiter);

                            writer.Write(Detectors.DetectorTotalVehicles[ii]);
                            writer.Write(delimiter);

                            writer.Write(Detectors.DetectorTotalExposure[ii]);

                            writer.WriteLine();
                        }
                    }
                }
            }
        }
    }
}
