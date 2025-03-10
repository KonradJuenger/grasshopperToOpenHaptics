using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Diagnostics;

namespace ghoh
{
    public class ghohVibrateCurve : GH_Component
    {
        private DateTime lastUpdateTime = DateTime.MinValue;
        private double lastDistance = 0.0;
        private double lastAmplitude = 0.0;
        private double lastCalcTime = 0.0;

        public ghohVibrateCurve() : base(
            "ghohVibrateCurve",
            "VibrateCurve",
            "Creates vibration feedback that varies in intensity based on proximity to a curve",
            "ghoh",
            "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enable", "E", "Enable or disable vibration", GH_ParamAccess.item, false);
            pManager.AddCurveParameter("Target", "T", "Reference curve for proximity detection", GH_ParamAccess.item);
            pManager.AddVectorParameter("Direction", "D", "Vibration direction vector", GH_ParamAccess.item, new Vector3d(0, 0, 1));
            pManager.AddNumberParameter("Deadzone", "DZ", "Distance to curve with no vibration (normal) or max vibration (inverted)", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("MaxDistance", "MD", "Maximum distance for vibration effect", GH_ParamAccess.item, 10.0);
            pManager.AddNumberParameter("MaxAmplitude", "MA", "Maximum vibration amplitude", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("Frequency", "F", "Vibration frequency in Hz (1-1000)", GH_ParamAccess.item, 100.0);
            pManager.AddBooleanParameter("Invert", "I", "Invert distance mapping behavior", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("SquareWave", "S", "Use square wave instead of sine wave", GH_ParamAccess.item, false);
            pManager.AddTransformParameter("Transform", "X", "Transform matrix for world to device space", GH_ParamAccess.item);
            pManager.AddNumberParameter("UpdateInterval", "U", "Output update interval in milliseconds (10-5000)", GH_ParamAccess.item, 100.0);

            pManager[2].Optional = true;  // Direction
            pManager[3].Optional = true;  // Deadzone
            pManager[6].Optional = true;  // Frequency
            pManager[7].Optional = true;  // Invert
            pManager[8].Optional = true;  // SquareWave
            pManager[9].Optional = true;  // Transform
            pManager[10].Optional = true; // UpdateInterval
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Distance", "D", "Current distance to curve", GH_ParamAccess.item);
            pManager.AddNumberParameter("Amplitude", "A", "Current vibration amplitude", GH_ParamAccess.item);
            pManager.AddNumberParameter("CalcTime", "T", "Time to calculate distance (ms)", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var handle = DeviceManager.DeviceHandle;
            if (handle == HDdll.HD_INVALID_HANDLE)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Device not initialized");
                return;
            }

            bool enable = false;
            Curve targetCurve = null;
            Vector3d direction = new Vector3d(0, 0, 1);
            double deadzone = 0.0;
            double maxDistance = 10.0;
            double maxAmplitude = 1.0;
            double frequency = 100.0;
            bool invertMapping = false;
            bool useSquareWave = false;
            Transform worldToDevice = Transform.Identity;
            double updateInterval = 100.0;

            if (!DA.GetData(0, ref enable)) return;
            if (!DA.GetData(1, ref targetCurve)) return;
            if (targetCurve == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Invalid curve input");
                return;
            }
            DA.GetData(2, ref direction);
            DA.GetData(3, ref deadzone);
            if (!DA.GetData(4, ref maxDistance)) return;
            if (!DA.GetData(5, ref maxAmplitude)) return;
            DA.GetData(6, ref frequency);
            DA.GetData(7, ref invertMapping);
            DA.GetData(8, ref useSquareWave);
            DA.GetData(9, ref worldToDevice);
            DA.GetData(10, ref updateInterval);

            // Validate and clamp parameters
            updateInterval = Math.Max(10.0, Math.Min(5000.0, updateInterval));
            deadzone = Math.Max(0.0, deadzone);
            maxDistance = Math.Max(deadzone + 0.001, maxDistance);  // Ensure max distance > deadzone
            maxAmplitude = Math.Max(0.0, Math.Min(3.0, maxAmplitude));
            frequency = Math.Max(1.0, Math.Min(1000.0, frequency));

            // Transform curve to device space if transform provided
            Curve transformedCurve = targetCurve;
            Vector3d transformedDirection = direction;
            if (!worldToDevice.Equals(Transform.Identity))
            {
                Transform deviceToWorld;
                if (worldToDevice.TryGetInverse(out deviceToWorld))
                {
                    transformedCurve = targetCurve.DuplicateCurve();
                    transformedCurve.Transform(deviceToWorld);
                    transformedDirection.Transform(deviceToWorld);
                }
            }

            // Convert direction to device space Vector3D
            var directionVector = new DeviceManager.Vector3D(
                transformedDirection.X,
                transformedDirection.Y,
                transformedDirection.Z
            );

            // Pass curve to ForceManager
            ForceManager.SetVibrateCurve(
                transformedCurve,
                directionVector,
                enable,
                deadzone,
                maxDistance,
                maxAmplitude,
                frequency,
                invertMapping,
                useSquareWave
            );

            // Check if enough time has passed to update output
            var currentTime = DateTime.Now;
            if ((currentTime - lastUpdateTime).TotalMilliseconds >= updateInterval)
            {
                lastDistance = ForceManager.vibrationCurveDistance;
                lastAmplitude = ForceManager.vibrationCurveAmplitude;
                lastCalcTime = ForceManager.vibrationCurveCalcTime;
                lastUpdateTime = currentTime;
            }

            // Output current values
            DA.SetData(0, lastDistance);
            DA.SetData(1, lastAmplitude);
            DA.SetData(2, lastCalcTime);
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c72c");
    }
}