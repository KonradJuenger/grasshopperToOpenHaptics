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
            "Creates vibration feedback based on proximity to a curve. Supports Sine, Square, and Pulse modes.",
            "ghoh",
            "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enable", "E", "Enable or disable vibration", GH_ParamAccess.item, false); // 0
            pManager.AddCurveParameter("Target", "T", "Reference curve for proximity detection (World Coordinates)", GH_ParamAccess.item); // 1
            pManager.AddVectorParameter("Direction", "D", "Vibration direction vector (World Coordinates)", GH_ParamAccess.item, new Vector3d(0, 0, 1)); // 2
            pManager.AddIntegerParameter("Mode", "M", "Vibration mode (0: Sine, 1: Square, 2: Pulse)", GH_ParamAccess.item, 0); // 3 - New
            pManager.AddNumberParameter("Deadzone", "DZ", "Distance (haptic units) to curve with no/max vibration", GH_ParamAccess.item, 0.0); // 4
            pManager.AddNumberParameter("MaxDistance", "MD", "Maximum distance (haptic units) for vibration effect", GH_ParamAccess.item, 10.0); // 5
            pManager.AddNumberParameter("MaxAmplitude", "MA", "Maximum vibration amplitude", GH_ParamAccess.item, 1.0); // 6
            pManager.AddNumberParameter("Frequency", "F", "Vibration frequency in Hz (1-1000) for Sine/Square/Pulse", GH_ParamAccess.item, 100.0); // 7
            pManager.AddBooleanParameter("Invert", "I", "Invert distance mapping (vibrate strong when close)", GH_ParamAccess.item, false); // 8
            pManager.AddNumberParameter("PulseDuration", "PD", "Duration of the 'on' state for a pulse (in seconds)", GH_ParamAccess.item, 0.1); // 9 - New
            pManager.AddNumberParameter("MinPause", "Pmin", "Minimum pause between pulses (at MaxDistance, in seconds)", GH_ParamAccess.item, 0.1); // 10 - New
            pManager.AddNumberParameter("MaxPause", "Pmax", "Maximum pause between pulses (at Deadzone, in seconds)", GH_ParamAccess.item, 1.0); // 11 - New
            pManager.AddTransformParameter("Transform", "X", "Transform matrix from World to Device space", GH_ParamAccess.item); // 12
            pManager.AddNumberParameter("UpdateInterval", "U", "Output update interval in milliseconds (10-5000)", GH_ParamAccess.item, 100.0); // 13

            pManager[2].Optional = true;
            pManager[3].Optional = true;
            pManager[4].Optional = true;
            pManager[7].Optional = true;
            pManager[8].Optional = true;
            pManager[9].Optional = true;
            pManager[10].Optional = true;
            pManager[11].Optional = true;
            pManager[12].Optional = true;
            pManager[13].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Distance", "D", "Current distance to curve (in haptic space units)", GH_ParamAccess.item);
            pManager.AddNumberParameter("Amplitude", "A", "Current vibration amplitude", GH_ParamAccess.item);
            pManager.AddNumberParameter("CalcTime", "T", "Time for ForceManager to calculate distance (ms)", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var handle = DeviceManager.DeviceHandle;
            if (handle == HDdll.HD_INVALID_HANDLE)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Device not initialized");
                ForceManager.SetVibrateCurve(null, new DeviceManager.Vector3D(), false, 0, 0, 0, 0, false, Vibration.VibrationMode.Sine, 0, 0, 0);
                return;
            }

            bool enable = false;
            Curve targetCurve_world = null;
            Vector3d direction_world = new Vector3d(0, 0, 1);
            int mode = 0;
            double deadzone = 0.0;
            double maxDistance = 10.0;
            double maxAmplitude = 1.0;
            double frequency = 100.0;
            bool invertMapping = false;
            double pulseDuration = 0.1;
            double minPause = 0.1;
            double maxPause = 1.0;
            Transform worldToDevice = Transform.Identity;
            double updateInterval = 100.0;

            DA.GetData(0, ref enable);
            DA.GetData(1, ref targetCurve_world);

            if (targetCurve_world == null)
            {
                ForceManager.SetVibrateCurve(null, new DeviceManager.Vector3D(), false, 0, 0, 0, 0, false, Vibration.VibrationMode.Sine, 0, 0, 0);
                if (enable) AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "A valid curve is required.");
                return;
            }

            DA.GetData(2, ref direction_world);
            DA.GetData(3, ref mode);
            DA.GetData(4, ref deadzone);
            DA.GetData(5, ref maxDistance);
            DA.GetData(6, ref maxAmplitude);
            DA.GetData(7, ref frequency);
            DA.GetData(8, ref invertMapping);
            DA.GetData(9, ref pulseDuration);
            DA.GetData(10, ref minPause);
            DA.GetData(11, ref maxPause);
            DA.GetData(12, ref worldToDevice);
            DA.GetData(13, ref updateInterval);

            // Validate and clamp parameters
            updateInterval = Math.Max(10.0, Math.Min(5000.0, updateInterval));
            deadzone = Math.Max(0.0, deadzone);
            maxDistance = Math.Max(deadzone + 0.001, maxDistance);
            maxAmplitude = Math.Max(0.0, maxAmplitude);
            frequency = Math.Max(1.0, Math.Min(1000.0, frequency));
            pulseDuration = Math.Max(0.0, pulseDuration);
            minPause = Math.Max(0.0, minPause);
            maxPause = Math.Max(minPause, maxPause);
            Vibration.VibrationMode vibMode = (Vibration.VibrationMode)Math.Max(0, Math.Min(2, mode));

            if (!direction_world.Unitize()) direction_world = new Vector3d(0, 0, 1);

            // Correct Transformation to Rhino Haptic space
            Curve curve_rhino = null;
            Vector3d direction_rhino = direction_world;

            Transform deviceToWorldInverse;
            if (!worldToDevice.Equals(Transform.Identity) && worldToDevice.TryGetInverse(out deviceToWorldInverse))
            {
                curve_rhino = targetCurve_world.DuplicateCurve();
                curve_rhino.Transform(deviceToWorldInverse);
                direction_rhino.Transform(deviceToWorldInverse);
            }
            else
            {
                curve_rhino = targetCurve_world.DuplicateCurve();
            }

            if (!direction_rhino.Unitize()) direction_rhino = new Vector3d(0, 1, 0);

            var directionVector_rhino = new DeviceManager.Vector3D(direction_rhino.X, direction_rhino.Y, direction_rhino.Z);

            // Pass all parameters to ForceManager
            ForceManager.SetVibrateCurve(
                curve_rhino,
                directionVector_rhino,
                enable,
                deadzone,
                maxDistance,
                maxAmplitude,
                frequency,
                invertMapping,
                vibMode,
                pulseDuration,
                minPause,
                maxPause
            );

            // Update output display
            var currentTime = DateTime.Now;
            if ((currentTime - lastUpdateTime).TotalMilliseconds >= updateInterval)
            {
                lastDistance = ForceManager.vibrationCurveDistance;
                lastAmplitude = ForceManager.vibrationCurveAmplitude;
                lastCalcTime = ForceManager.vibrationCurveCalcTime;
                lastUpdateTime = currentTime;
            }

            DA.SetData(0, lastDistance);
            DA.SetData(1, lastAmplitude);
            DA.SetData(2, lastCalcTime);
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c72c");
    }
}
