using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Diagnostics;

namespace ghoh
{
    public class ghohVibrateCurveCamera : GH_Component
    {
        private DateTime lastUpdateTime = DateTime.MinValue;
        private double lastDistance = 0.0;
        private double lastAmplitude = 0.0;
        private bool lastIsFront = false;
        private double lastCalcTime = 0.0;

        public ghohVibrateCurveCamera() : base(
            "ghohVibrateCurveCamera",
            "VibrateCam",
            "Vibrates based on proximity to a curve, with different frequencies if the device is in front of or behind the curve from the camera's perspective.",
            "ghoh",
            "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enable", "E", "Enable or disable vibration", GH_ParamAccess.item, false); // 0
            pManager.AddCurveParameter("Target", "T", "Reference curve for proximity detection (World Coordinates)", GH_ParamAccess.item); // 1
            pManager.AddVectorParameter("Direction", "D", "Vibration direction vector (World Coordinates)", GH_ParamAccess.item, new Vector3d(0, 0, 1)); // 2
            pManager.AddNumberParameter("Deadzone", "DZ", "Distance (haptic units) to curve with no/max vibration", GH_ParamAccess.item, 0.0); // 3
            pManager.AddNumberParameter("MaxDistance", "MD", "Maximum distance (haptic units) for vibration effect", GH_ParamAccess.item, 10.0); // 4
            pManager.AddNumberParameter("MaxAmplitude", "MA", "Maximum vibration amplitude", GH_ParamAccess.item, 1.0); // 5
            pManager.AddNumberParameter("FrequencyFront", "Ff", "Vibration frequency (Hz) when device is IN FRONT of the curve", GH_ParamAccess.item, 100.0); // 6
            pManager.AddNumberParameter("FrequencyBack", "Fb", "Vibration frequency (Hz) when device is BEHIND the curve", GH_ParamAccess.item, 50.0); // 7
            pManager.AddBooleanParameter("Invert", "I", "Invert distance mapping (vibrate strong when close)", GH_ParamAccess.item, false); // 8
            pManager.AddTransformParameter("Transform", "X", "Transform matrix from World to Device space", GH_ParamAccess.item); // 9
            pManager.AddNumberParameter("UpdateInterval", "U", "Output update interval in milliseconds (10-5000)", GH_ParamAccess.item, 100.0); // 10

            pManager[2].Optional = true;
            pManager[3].Optional = true;
            pManager[9].Optional = true;
            pManager[10].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Distance", "D", "Current distance to curve (in haptic space units)", GH_ParamAccess.item);
            pManager.AddNumberParameter("Amplitude", "A", "Current vibration amplitude", GH_ParamAccess.item);
            pManager.AddBooleanParameter("IsFront", "F", "True if the device is in front of the curve relative to the camera", GH_ParamAccess.item);
            pManager.AddNumberParameter("CalcTime", "T", "Time for ForceManager to calculate the effect (ms)", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var handle = DeviceManager.DeviceHandle;
            if (handle == HDdll.HD_INVALID_HANDLE)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Device not initialized");
                ForceManager.SetVibrateCurveCamera(null, new DeviceManager.Vector3D(), false, 0, 0, 0, 0, 0, false);
                return;
            }

            bool enable = false;
            Curve targetCurve_world = null;
            Vector3d direction_world = new Vector3d(0, 0, 1);
            double deadzone = 0.0;
            double maxDistance = 10.0;
            double maxAmplitude = 1.0;
            double frequencyFront = 100.0;
            double frequencyBack = 50.0;
            bool invertMapping = false;
            Transform worldToDevice = Transform.Identity;
            double updateInterval = 100.0;

            DA.GetData(0, ref enable);
            DA.GetData(1, ref targetCurve_world);

            if (targetCurve_world == null)
            {
                ForceManager.SetVibrateCurveCamera(null, new DeviceManager.Vector3D(), false, 0, 0, 0, 0, 0, false);
                if (enable) AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "A valid curve is required.");
                return;
            }

            DA.GetData(2, ref direction_world);
            DA.GetData(3, ref deadzone);
            DA.GetData(4, ref maxDistance);
            DA.GetData(5, ref maxAmplitude);
            DA.GetData(6, ref frequencyFront);
            DA.GetData(7, ref frequencyBack);
            DA.GetData(8, ref invertMapping);
            DA.GetData(9, ref worldToDevice);
            DA.GetData(10, ref updateInterval);

            // Validate and clamp parameters
            updateInterval = Math.Max(10.0, Math.Min(5000.0, updateInterval));
            deadzone = Math.Max(0.0, deadzone);
            maxDistance = Math.Max(deadzone + 0.001, maxDistance);
            maxAmplitude = Math.Max(0.0, maxAmplitude);
            frequencyFront = Math.Max(1.0, Math.Min(1000.0, frequencyFront));
            frequencyBack = Math.Max(1.0, Math.Min(1000.0, frequencyBack));

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
            ForceManager.SetVibrateCurveCamera(
                curve_rhino,
                directionVector_rhino,
                enable,
                deadzone,
                maxDistance,
                maxAmplitude,
                frequencyFront,
                frequencyBack,
                invertMapping
            );

            // Update output display
            var currentTime = DateTime.Now;
            if ((currentTime - lastUpdateTime).TotalMilliseconds >= updateInterval)
            {
                lastDistance = ForceManager.vibrationCurveCameraDistance;
                lastAmplitude = ForceManager.vibrationCurveCameraAmplitude;
                lastIsFront = ForceManager.vibrationCurveCameraIsFront;
                lastCalcTime = ForceManager.vibrationCurveCameraCalcTime;
                lastUpdateTime = currentTime;
            }

            DA.SetData(0, lastDistance);
            DA.SetData(1, lastAmplitude);
            DA.SetData(2, lastIsFront);
            DA.SetData(3, lastCalcTime);
        }

        protected override System.Drawing.Bitmap Icon => null; // You can add a custom icon here

        public override Guid ComponentGuid => new Guid("a1b2c3d4-e5f6-7890-1234-567890abcdef"); // Replace with a new unique GUID
    }
}
