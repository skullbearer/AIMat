using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
        //User Settable

        // tickStep is how many ticks between actions. Control should be smooth for any tick count, smaller numbers means faster response and higher accelerations possible.
        // For the sake of your server you're playing on, typically you would run between 5 and 10 ticks, but even 1 tick is more than reasonably performant for this script.
        int ticksBetweenAction = 10;
        

        // If you add this tag to the name of a control block, that is, Remote Control, Flight Seat, Cockpit, etc, it will use that for orientation.
        string controlFlag = "[AIM]";
        string defaultForward = "Forward";





        //Don't Touch below this line!
        List<IMyGyro> myGyros = new List<IMyGyro>();
        IMyShipController mySeat;
        Vector3D targetGPS;
        List<Vector3D> targList = new List<Vector3D>();
        int currGPS = 0;

        Dictionary<string, Action> _cmnds = new Dictionary<string, Action>(StringComparer.OrdinalIgnoreCase);
        MyCommandLine cL = new MyCommandLine();
        IEnumerator<bool> sM;

        GyroPTol xAxis = new GyroPTol();
        GyroPTol yAxis = new GyroPTol();
        GyroPTol zAxis = new GyroPTol();
      
        double timeStep;
        int tickStep;
        int tickCount;
        List<double> setList = new List<double>();
        int numInc = 10;
        bool posNeg = false;
        Dictionary<string, Vector3D> resultBook = new Dictionary<string, Vector3D>();
        int iNum;
        bool firstStep;
        bool reverseIt;
        bool brakingTest = false;
        double lastSpeed = 0;

        Func<bool> currAction;
        bool doProfile = false;

        enum fwdDir
        {
            Forward,
            Down,
            Backward,
            Up,
            Left,
            Right
        }

        fwdDir myFwd;

        public Program()
        {
            GetBlocks();
            targList.Clear();
            DisableOverrides();

            _cmnds["off"] = Off;
            _cmnds["aim"] = Aim;
            _cmnds["add"] = Add;
            _cmnds["next"] = Next;
            _cmnds["Ret"] = Retrograde;
            _cmnds["Pro"] = Prograde;
            _cmnds["Flip"] = Flip;
            _cmnds["test"] = ProfileGyros;
            _cmnds["cont"] = Continue;
            _cmnds["reset"] = Reset;
            _cmnds["pause"] = Idle;

            myFwd = fwdDir.Forward;
            tickStep = ticksBetweenAction;
            timeStep = (double)tickStep / 60.0;
            tickCount = tickStep; // Run the first time.
            defaultForward = defaultForward.ToLower();

            doProfile = false;
            resultBook.Clear();
            setList.Clear();

            Runtime.UpdateFrequency |= UpdateFrequency.None;
        }

        public void Reset()
        {
            tickStep = ticksBetweenAction;
            timeStep = (double)tickStep / 60.0;
            tickCount = tickStep;
            resultBook.Clear();
            setList.Clear();
            DisableOverrides();
        }
        public void Continue()
        {
            if (sM != null)
                Runtime.UpdateFrequency |= UpdateFrequency.Once;
            else
                Echo("No action in progress.");
        }
        public void Start()
        {
            Idle();
            sM?.Dispose();
            sM = null;
            sM = RunStuffOverTime();
            Runtime.UpdateFrequency |= UpdateFrequency.Once;
        }
        public void ProfileGyros()
        {
            //Echo("In ProfileGyros()");
            tickStep = 1;
            timeStep = (double)tickStep / 60.0;
            tickCount = tickStep;
            for(int i = 0; i < numInc; i++)
            {
                setList.Add(Math.PI - (double)i * (Math.PI / numInc));
                Echo($"{setList[setList.Count - 1]}");
            }
            if (posNeg)
            {
                for(int i = numInc-1; i >= 0; i--)
                {
                    setList.Add(-Math.PI + (double)i * (Math.PI / numInc));
                }
            }
            firstStep = true;
            reverseIt = false;
            doProfile = true;
            currAction = () => profileDriver();
            Start();
        }

        public bool profileDriver()
        {
            //Echo("In profileDriver()");
            double mySpd = Vector3D.TransformNormal(mySeat.GetShipVelocities().AngularVelocity, MatrixD.Transpose(mySeat.WorldMatrix)).X;
            //Echo($"{Math.Round(mySpd,2)},{Math.Round(setList[iNum],2)}");
            int dir;
            if (setList[iNum] == 0)
                dir = 1;
            else
                dir = Math.Sign(setList[iNum]);
            if (firstStep && mySpd > -Math.PI*dir)
            {
                Echo("Reversing to eliminate zero point error...");
                GyroCommand(new Vector3D(Math.PI * dir, 0, 0), mySeat.WorldMatrix, false, true);
                lastSpeed = mySpd;  
                return false;
            }
            else if (iNum < setList.Count - 1 && mySpd < setList[iNum] - 0.001)
            {
                GyroCommand(new Vector3D(-setList[iNum], 0, 0), mySeat.WorldMatrix, false, true);
                if (firstStep)
                    firstStep = false;
                else
                    resultBook.Add($"{mySpd},{setList[iNum]}", new Vector3D(mySpd, setList[iNum],(mySpd-lastSpeed)/timeStep));
                lastSpeed = mySpd;
                return false;
            }
            return true;
        }
        public void Add()
        {
            Vector3D _tempGPS;
            int _argNum = 0;
            string _desNm = null;
            if (cL.Argument(0).StartsWith("gps"))
            {
                targList.Clear();
            }
            else
                _argNum = 1;

            string _gpsHold = cL.Argument(_argNum);
            if (_gpsHold == null || !cL.Argument(_argNum).ToLower().StartsWith("gps"))
            {
                Echo($"Cannot add GPS, no GPS given in {cL.Argument(_argNum)}");
                return;
            }
            StringBuilder unParse = new StringBuilder();
            if (cL.ArgumentCount > 2)
            {
                for (int i = 1; i < cL.ArgumentCount - 1; i++)
                {
                    unParse.Append(cL.Argument(i));
                }
                if (!TryParseGps(unParse.ToString(), out targetGPS))
                    Echo("No GPS found!");
            }
            if (TryParseGps(_gpsHold, out _tempGPS))
            {
                targList.Add(_tempGPS);
            }
            else
            {
                Echo($"Cannot add GPS, {_gpsHold} is an incorrect format.");
                return;
            }
            //wptS.Add(_desNm);
            Echo($"Added: {_tempGPS}");
        }

        public void DisableOverrides()
        {
            GyroCommand(new Vector3D(0, 0, 0), mySeat.WorldMatrix, true, false);
            Runtime.UpdateFrequency = UpdateFrequency.None;
        }

        public void Save()
        {
            // Called when the program needs to save its state. Use
            // this method to save your state to the Storage field
            // or some other means. 
            // 
            // This method is optional and can be removed if not
            // needed.
        }

        public void Main(string argument, UpdateType updateSource)
        {
            //Echo($"{argument}");
            if (cL.TryParse(argument))
            {
                Action commandAction;

                string command = cL.Argument(0);

                if (cL.Switches.Count > 0)
                {
                    if (cL.Switch("Forward"))
                        myFwd = fwdDir.Forward;
                    else if (cL.Switch("Down"))
                        myFwd = fwdDir.Down;
                    else if (cL.Switch("Backward"))
                        myFwd = fwdDir.Backward;
                    else if (cL.Switch("Up"))
                        myFwd = fwdDir.Up;
                    else if (cL.Switch("Left"))
                        myFwd = fwdDir.Left;
                    else if (cL.Switch("Right"))
                        myFwd = fwdDir.Right;
                }

                if (command == null)
                    Echo("No command specified.");
                else if (_cmnds.TryGetValue(command, out commandAction))
                {
                    Echo($"{argument} will be run!");
                    commandAction();
                }
                else
                    Echo($"Unknown command {command}");
            }

            if ((updateSource & UpdateType.Once) == UpdateType.Once)
            {
                //Echo("Running StateMachine");
                RunStateMachine();
            }

            //Echo("End of Main.");

        }

        public void GetBlocks()
        {
            List<IMyShipController> _seatList = new List<IMyShipController>();
            GridTerminalSystem.GetBlocksOfType(myGyros, b => b.CubeGrid == Me.CubeGrid);
            GridTerminalSystem.GetBlocksOfType(_seatList, s => s.CubeGrid == Me.CubeGrid);
            if (_seatList.Count == 0)
                throw new Exception("Gotta have some kind of ship controller.");
            for (int i = _seatList.Count - 1; i >= 0; i--)
            {
                if (_seatList[i].IsUnderControl && _seatList[i].IsSameConstructAs(Me))
                {
                    mySeat = _seatList[i];
                }
                if (_seatList[i].CustomName.Contains(controlFlag) && _seatList[i].IsSameConstructAs(Me))
                {
                    mySeat = _seatList[i];
                    break;
                }
            }
            if (mySeat == null)
            { // If they aren't sitting in a seat when recompiled, just grab whatever controller.
                mySeat = _seatList[0];
            }
                
            for (int i = myGyros.Count - 1; i >= 0; i--)
            {
                if (myGyros[i].Closed || !myGyros[i].IsFunctional)
                    myGyros.RemoveAtFast(i);
                GyroCommand(new Vector3D(0, 0, 0), mySeat.WorldMatrix, true, false);
            }
        }

        public bool GyroAim(Vector3D targVec, IMyShipController _mySeat, MatrixD refRot, bool isGPS = false, double angTol = 0.1)
        {
            Vector3D AimVec;
            if (isGPS)
                AimVec = targVec - _mySeat.GetPosition();
            else
                AimVec = targVec;
            MatrixD _useMatrix = pickForward(_mySeat);

            Vector3D angS = mySeat.GetShipVelocities().AngularVelocity;
            angS = Vector3D.TransformNormal(angS, MatrixD.Transpose(_mySeat.WorldMatrix));
            Vector3D GCV = new Vector3D(0, 0, 0);
            Vector3D ang2trg = new Vector3D();
            Vector3D ang2fwd = Vector3D.Zero;

            GetRotationAnglesSimultaneous(AimVec, new Vector3D(0,0,0), _useMatrix, out ang2trg.X, out ang2trg.Y, out ang2trg.Z);

            ang2trg += ang2fwd;

            GCV.X = xAxis.PIDRunSelfAdjusting(ang2trg.X, angS.X, timeStep, angTol * angTol);
            GCV.Y = yAxis.PIDRunSelfAdjusting(ang2trg.Y, angS.Y, timeStep, angTol * angTol);
            //Echo($"{Vector3D.Round(ang2trg, 2)}\n{Vector3D.Round(angS, 2)}\n{Vector3D.Round(GCV, 2)}\n{Math.Round(xAxis.gainPA, 2)},{Math.Round(yAxis.gainPA, 2)},{Math.Round(zAxis.gainPA, 2)}\n{Math.Round(xAxis.gainPD, 2)},{Math.Round(yAxis.gainPD, 2)},{Math.Round(zAxis.gainPD, 2)}");
            GyroCommand(GCV, _useMatrix);
            Echo($"AngleToTarget: {Vector3D.Round(ang2trg, 2)}");

            // Allows the user to toggle perpetual aiming
            if (angTol <= 0.001)
                return false;
            else
                return ang2trg.Length() <= angTol;
        }

        public void GyroCommand(Vector3D RA, MatrixD refMtrx, bool isWorldAxis = false, bool onOff = true)
        {
            Vector3D lclRA;             //Vector to be used for local gyro coodinates

            foreach (IMyGyro g in myGyros)     //Command gyros to correct pitch, roll, and yaw
            {
                if (isWorldAxis) //transform world rotation vector to gyro local coordinates
                    lclRA = Vector3D.TransformNormal(RA, MatrixD.Transpose(g.WorldMatrix));
                else //transform controller local rotation vector to gyro local coordinates
                    lclRA = Vector3D.TransformNormal(Vector3D.TransformNormal(RA, refMtrx), MatrixD.Transpose(g.WorldMatrix));
                g.Pitch = (float)(lclRA.X); // Commands in RPM, need to calibrate, do not match HUD value.
                g.Yaw = (float)(lclRA.Y);
                g.Roll = (float)(lclRA.Z);
                g.GyroOverride = onOff;
            }
        }

        public void Off()
        {
            DisableOverrides();
            if (doProfile)
                OutputData();
            sM?.Dispose();
            sM = null;
        }

        public void TestSwitches()
        {
            foreach (string s in cL.Switches)
            {
                Echo(s);
            }
        }
        public MatrixD pickForward(IMyShipController b)
        {
            Vector3D _useFwd = b.WorldMatrix.Forward;
            Vector3D _useUp = b.WorldMatrix.Up;
            if (myFwd == fwdDir.Forward)
            {
                _useFwd = b.WorldMatrix.Forward;
                _useUp = b.WorldMatrix.Up;
            }
            else if (myFwd == fwdDir.Down)
            {
                _useFwd = b.WorldMatrix.Down;
                _useUp = b.WorldMatrix.Forward;
            }
            else if (myFwd == fwdDir.Backward)
            {
                _useFwd = b.WorldMatrix.Backward;
                _useUp = b.WorldMatrix.Down;
            }
            else if (myFwd == fwdDir.Up)
            {
                _useFwd = b.WorldMatrix.Up;
                _useUp = b.WorldMatrix.Backward;
            }
            else if (myFwd == fwdDir.Left)
            {
                _useFwd = b.WorldMatrix.Left;
                _useUp = b.WorldMatrix.Up;
            }
            else if (myFwd == fwdDir.Right)
            {
                _useFwd = b.WorldMatrix.Right;
                _useUp = b.WorldMatrix.Up;
            }

            MatrixD _newMatrix = MatrixD.Zero;
            _newMatrix.Forward = _useFwd;
            _newMatrix.Up = _useUp;
            _newMatrix.Left = Vector3D.Cross(_newMatrix.Up, _newMatrix.Forward);
            return _newMatrix;
        }

        public void Aim()
        {
            double _angTol = 0.01;
            if (cL.Switch("continuous"))
                _angTol = 0.0;

            if (cL.ArgumentCount > 1 && cL.Argument(1).StartsWith("gps"))
            {
                Add();
            }
            if (targList.Count < 1)
                Next();
            else
            {
                currAction = () => GyroAim(targetGPS, mySeat, pickForward(mySeat), true, _angTol);
                Start();
            }
        }

        public void Next()
        {
            if (targList.Count > 0)
            {
                if (currGPS < targList.Count - 1)
                    currGPS++;
                else
                    currGPS = 0;
                targetGPS = targList[currGPS];
                Aim();
            }
            else
                Echo("Need to add a GPS first!");
        }

        public void Flip()
        {
            double _angTol = 0.01;
            if (cL.Switch("continuous"))
                _angTol = 0.0;
            Vector3D _aimVec = mySeat.WorldMatrix.Backward;
            currAction = () => GyroAim(_aimVec, mySeat, pickForward(mySeat), false, _angTol) ;
            Start();
        }

        public void Retrograde()
        {
            double _angTol = 0.01;
            if (cL.Switch("continuous"))
                _angTol = 0.0;
            Vector3D _aimVec = -mySeat.GetShipVelocities().LinearVelocity;
            currAction = () => GyroAim(_aimVec, mySeat, pickForward(mySeat), false, _angTol);
            Start();
        }

        public void Prograde()
        {
            double _angTol = 0.01;
            if (cL.Switch("continuous"))
                _angTol = 0.0;
            Vector3D _aimVec = mySeat.GetShipVelocities().LinearVelocity;
            currAction = () => GyroAim(_aimVec, mySeat, pickForward(mySeat), false, _angTol);
            Start();
        }

        public void RunStateMachine()
        {
            bool hasMoreSteps = true;
            if (sM != null)
            {
                //Echo("We have a StateMachine.");
                if (tickCount == tickStep)
                {
                    //Echo("Running this tick.");
                    hasMoreSteps = sM.MoveNext();
                    //Echo($"Running Again: {hasMoreSteps}");
                    tickCount = 0;
                }

                if (hasMoreSteps)
                {
                    Runtime.UpdateFrequency |= UpdateFrequency.Once;
                }
                else
                {
                    Echo("Disposing the StateMachine.");
                    sM?.Dispose();
                    sM = null;
                }
                tickCount++;
            }
        }

        public void Idle()
        {
            DisableOverrides();
        }
        IEnumerator<bool> RunStuffOverTime()
        {
            if (currAction == null)
            {
                Idle();
                Echo("Ran without anything to do.");
                yield break;
            }
            while (true)
            {
                if (currAction())
                {
                    if (doProfile)
                    {
                        if (iNum < setList.Count - 1)
                        {
                            iNum++;
                            firstStep = true;
                            Echo("Next Step in profiling gyros.");
                        }
                        else
                        {
                            Echo("Done profiling the gyros! Check CustomData.");
                            OutputData();
                            doProfile = false;
                            firstStep = true;
                            Runtime.UpdateFrequency = UpdateFrequency.None;
                        }
                    }
                    else
                    {
                        Echo("Done with my task.");
                        Idle();
                        yield break;
                    }
                }
                //Echo("YIELD!");
                yield return true;
            }
        }

        public void OutputData()
        {
            StringBuilder buildOutput = new StringBuilder();
            foreach (string myKey in resultBook.Keys)
            {
                Echo($"Adding: {resultBook[myKey]}");
                buildOutput.Append($"{resultBook[myKey].X},{resultBook[myKey].Y},{resultBook[myKey].Z}\n");
            }
            Me.CustomData = buildOutput.ToString();
            Reset();
        }

        bool TryParseGps(string gpsString, out Vector3D vector)
        {
            //GPS:NAME:X:Y:Z:#COLOR:
            vector = new Vector3D(0, 0, 0);
            var gpsStringSplit = gpsString.Split(':');

            double x, y, z;

            if (gpsStringSplit.Length < 6)
                return false;
            bool passX = double.TryParse(gpsStringSplit[2], out x);
            bool passY = double.TryParse(gpsStringSplit[3], out y);
            bool passZ = double.TryParse(gpsStringSplit[4], out z);

            //Echo($"{x},{y},{z}");

            if (passX && passY && passZ)
            {
                vector = new Vector3D(x, y, z);
                return true;
            }
            else
                return false;
        }

        public static class VectorMath
        { // Whiplash141 from Unpolished Repository
            /// <summary>
            ///  Normalizes a vector only if it is non-zero and non-unit
            /// </summary>
            public static Vector3D SafeNormalize(Vector3D a)
            {
                if (Vector3D.IsZero(a))
                    return Vector3D.Zero;

                if (Vector3D.IsUnit(ref a))
                    return a;

                return Vector3D.Normalize(a);
            }

            /// <summary>
            /// Reflects vector a over vector b with an optional rejection factor
            /// </summary>
            public static Vector3D Reflection(Vector3D a, Vector3D b, double rejectionFactor = 1) //reflect a over b
            {
                Vector3D project_a = Projection(a, b);
                Vector3D reject_a = a - project_a;
                return project_a - reject_a * rejectionFactor;
            }

            /// <summary>
            /// Rejects vector a on vector b
            /// </summary>
            public static Vector3D Rejection(Vector3D a, Vector3D b) //reject a on b
            {
                if (Vector3D.IsZero(a) || Vector3D.IsZero(b))
                    return Vector3D.Zero;

                return a - a.Dot(b) / b.LengthSquared() * b;
            }

            /// <summary>
            /// Projects vector a onto vector b
            /// </summary>
            public static Vector3D Projection(Vector3D a, Vector3D b)
            {
                if (Vector3D.IsZero(a) || Vector3D.IsZero(b))
                    return Vector3D.Zero;

                if (Vector3D.IsUnit(ref b))
                    return a.Dot(b) * b;

                return a.Dot(b) / b.LengthSquared() * b;
            }

            /// <summary>
            /// Scalar projection of a onto b
            /// </summary>
            public static double ScalarProjection(Vector3D a, Vector3D b)
            {
                if (Vector3D.IsZero(a) || Vector3D.IsZero(b))
                    return 0;

                if (Vector3D.IsUnit(ref b))
                    return a.Dot(b);

                return a.Dot(b) / b.Length();
            }

            /// <summary>
            /// Computes angle between 2 vectors in radians.
            /// </summary>
            public static double AngleBetween(Vector3D a, Vector3D b)
            {
                if (Vector3D.IsZero(a) || Vector3D.IsZero(b))
                    return 0;
                else
                    return Math.Acos(MathHelper.Clamp(a.Dot(b) / Math.Sqrt(a.LengthSquared() * b.LengthSquared()), -1, 1));
            }

            /// <summary>
            /// Computes cosine of the angle between 2 vectors.
            /// </summary>
            public static double CosBetween(Vector3D a, Vector3D b)
            {
                if (Vector3D.IsZero(a) || Vector3D.IsZero(b))
                    return 0;
                else
                    return MathHelper.Clamp(a.Dot(b) / Math.Sqrt(a.LengthSquared() * b.LengthSquared()), -1, 1);
            }

            /// <summary>
            /// Returns if the normalized dot product between two vectors is greater than the tolerance.
            /// This is helpful for determining if two vectors are "more parallel" than the tolerance.
            /// </summary>
            /// <param name="a">First vector</param>
            /// <param name="b">Second vector</param>
            /// <param name="tolerance">Cosine of maximum angle</param>
            /// <returns></returns>
            public static bool IsDotProductWithinTolerance(Vector3D a, Vector3D b, double tolerance)
            {
                double dot = Vector3D.Dot(a, b);
                double num = a.LengthSquared() * b.LengthSquared() * tolerance * Math.Abs(tolerance);
                return Math.Abs(dot) * dot > num;
            }

        }
        void GetRotationAnglesSimultaneous(Vector3D desiredForwardVector, Vector3D desiredUpVector, MatrixD worldMatrix, out double pitch, out double yaw, out double roll)
        { // From Whiplash141 Unpolished Repository 05/11/2022
            desiredForwardVector = VectorMath.SafeNormalize(desiredForwardVector);

            MatrixD transposedWm;
            MatrixD.Transpose(ref worldMatrix, out transposedWm);
            Vector3D.Rotate(ref desiredForwardVector, ref transposedWm, out desiredForwardVector);
            Vector3D.Rotate(ref desiredUpVector, ref transposedWm, out desiredUpVector);

            Vector3D leftVector = Vector3D.Cross(desiredUpVector, desiredForwardVector);
            Vector3D axis;
            double angle;
            if (Vector3D.IsZero(desiredUpVector) || Vector3D.IsZero(leftVector))
            {
                axis = new Vector3D(desiredForwardVector.Y, -desiredForwardVector.X, 0);
                angle = Math.Acos(MathHelper.Clamp(-desiredForwardVector.Z, -1.0, 1.0));
            }
            else
            {
                leftVector = VectorMath.SafeNormalize(leftVector);
                Vector3D upVector = Vector3D.Cross(desiredForwardVector, leftVector);

                // Create matrix
                MatrixD targetMatrix = MatrixD.Zero;
                targetMatrix.Forward = desiredForwardVector;
                targetMatrix.Left = leftVector;
                targetMatrix.Up = upVector;

                axis = new Vector3D(targetMatrix.M23 - targetMatrix.M32,
                                    targetMatrix.M31 - targetMatrix.M13,
                                    targetMatrix.M12 - targetMatrix.M21);

                double trace = targetMatrix.M11 + targetMatrix.M22 + targetMatrix.M33;
                angle = Math.Acos(MathHelper.Clamp((trace - 1) * 0.5, -1, 1));
            }

            if (Vector3D.IsZero(axis))
            {
                angle = desiredForwardVector.Z < 0 ? 0 : Math.PI;
                yaw = angle;
                pitch = 0;
                roll = 0;
                return;
            }

            axis = VectorMath.SafeNormalize(axis);
            // Because gyros rotate about -X -Y -Z, we need to negate our angles
            yaw = -axis.Y * angle;
            pitch = -axis.X * angle;
            roll = -axis.Z * angle;
        }
    }
}
