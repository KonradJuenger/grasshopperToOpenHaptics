using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic; // Required for List

namespace ghoh
{
    public class ghohVibratePoint : GH_Component
    {
        private DateTime lastUpdateTime = DateTime.MinValue;
        private double lastDistance = 0.0; // Represents distance to the first target for output
        private double lastAmplitude = 0.0; // Represents amplitude for the first target for output

        public ghohVibratePoint() : base(
            "ghohVibratePoint", // Internal name
            "Vibrate At Points", // Component name in Grasshopper UI
            "Creates vibration feedback based on proximity to one or more points. All points share common vibration parameters. The MaxAmplitude input defines the overall cap for the combined vibration.",
            "ghoh",
            "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enable", "E", "Enable or disable vibration", GH_ParamAccess.item, false);
            pManager.AddPointParameter("Targets", "T", "Reference point(s) for proximity detection", GH_ParamAccess.list);
            pManager.AddVectorParameter("Direction", "D", "Vibration direction vector (world space, applied to all points)", GH_ParamAccess.item, new Vector3d(0, 0, 1));
            pManager.AddNumberParameter("Deadzone", "DZ", "Radius around points with no vibration (normal) or max vibration (inverted)", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("MaxDistance", "MD", "Maximum distance for vibration effect from points", GH_ParamAccess.item, 10.0);
            // MODIFIED: Description updated to reflect its role as an overall cap.
            pManager.AddNumberParameter("MaxAmplitude", "MA", "Maximum vibration amplitude. This acts as an overall cap for the combined effect if multiple points are active.", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("Frequency", "F", "Vibration frequency in Hz (1-1000)", GH_ParamAccess.item, 100.0);
            pManager.AddBooleanParameter("Invert", "I", "Invert distance mapping behavior", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("SquareWave", "S", "Use square wave instead of sine wave", GH_ParamAccess.item, false);
            pManager.AddTransformParameter("Transform", "X", "Optional transform matrix for world to device space. If not provided, world coordinates are assumed to be device coordinates.", GH_ParamAccess.item);
            pManager.AddNumberParameter("UpdateInterval", "U", "Output update interval in milliseconds (10-5000) for display values", GH_ParamAccess.item, 100.0);

            pManager[2].Optional = true;
            pManager[3].Optional = true;
            // Index 5 is MaxAmplitude
            pManager[6].Optional = true; // Frequency
            pManager[7].Optional = true; // Invert
            pManager[8].Optional = true; // SquareWave
            pManager[9].Optional = true; // Transform
            pManager[10].Optional = true; // UpdateInterval
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Distance (First Pt)", "D1", "Current distance to the first target point (if any)", GH_ParamAccess.item);
            pManager.AddNumberParameter("Amplitude (First Pt)", "A1", "Calculated individual amplitude for the first target point (if any), before overall capping of combined forces.", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var handle = DeviceManager.DeviceHandle;
            if (handle == HDdll.HD_INVALID_HANDLE)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Device not initialized. Vibration disabled.");
                ForceManager.ClearVibratePoints();
                return;
            }

            bool enable = false;
            List<Point3d> worldTargets = new List<Point3d>();
            Vector3d worldDirection = new Vector3d(0, 0, 1);
            double deadzone = 0.0;
            double maxDistance = 10.0;
            double overallMaxAmplitude = 1.0; // This variable will hold the value from "MaxAmplitude" input.
            double frequency = 100.0;
            bool invertMapping = false;
            bool useSquareWave = false;
            Transform worldToDeviceTransform = Transform.Identity;
            double updateInterval = 100.0;

            DA.GetData(0, ref enable);
            DA.GetDataList(1, worldTargets);
            DA.GetData(2, ref worldDirection);
            DA.GetData(3, ref deadzone);
            DA.GetData(4, ref maxDistance);
            DA.GetData(5, ref overallMaxAmplitude); // Read the "MaxAmplitude" input.
            DA.GetData(6, ref frequency);
            DA.GetData(7, ref invertMapping);
            DA.GetData(8, ref useSquareWave);
            DA.GetData(9, ref worldToDeviceTransform);
            DA.GetData(10, ref updateInterval);

            if (!enable || worldTargets == null || worldTargets.Count == 0)
            {
                ForceManager.ClearVibratePoints();
                if (enable && (worldTargets == null || worldTargets.Count == 0))
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Vibration enabled but no target points provided.");
                }
                DA.SetData(0, 0.0);
                DA.SetData(1, 0.0);
                return;
            }

            updateInterval = Math.Max(10.0, Math.Min(5000.0, updateInterval));
            deadzone = Math.Max(0.0, deadzone);
            maxDistance = Math.Max(deadzone + 0.001, maxDistance);
            // The overallMaxAmplitude is clamped inside ForceManager, but good to be aware of its intended range.
            // overallMaxAmplitude = Math.Max(0.0, Math.Min(3.0, overallMaxAmplitude)); // Clamping is done in ForceManager
            frequency = Math.Max(1.0, Math.Min(1000.0, frequency));

            List<DeviceManager.Vector3D> deviceSpaceTargetVectors = new List<DeviceManager.Vector3D>();
            Transform inverseTransform = Transform.Identity;
            bool applyTransform = !worldToDeviceTransform.Equals(Transform.Identity);

            if (applyTransform)
            {
                if (!worldToDeviceTransform.TryGetInverse(out inverseTransform))
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Failed to invert transformation matrix. Using world coordinates as device coordinates.");
                    applyTransform = false;
                }
            }

            foreach (var worldPt in worldTargets)
            {
                Point3d devicePt = worldPt;
                if (applyTransform)
                {
                    devicePt.Transform(inverseTransform);
                }
                deviceSpaceTargetVectors.Add(new DeviceManager.Vector3D(devicePt.X, devicePt.Y, devicePt.Z));
            }

            Vector3d deviceSpaceDirection = worldDirection;
            if (applyTransform)
            {
                deviceSpaceDirection.Transform(inverseTransform);
            }
            var deviceDirectionVector = new DeviceManager.Vector3D(
                deviceSpaceDirection.X, deviceSpaceDirection.Y, deviceSpaceDirection.Z);

            // Pass the overallMaxAmplitude read from the component to ForceManager.
            ForceManager.SetVibratePoints(
                deviceSpaceTargetVectors,
                deviceDirectionVector,
                true,
                deadzone,
                maxDistance,
                overallMaxAmplitude, // This is passed as the overall cap.
                frequency,
                invertMapping,
                useSquareWave
            );

            var state = DeviceManager.GetCurrentState();
            try
            {
                if (state.Transform == null)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Could not get device state for output display.");
                    DA.SetData(0, 0.0);
                    DA.SetData(1, 0.0);
                    return;
                }

                var currentDevicePosition = new Point3d(
                    -state.Transform[12], state.Transform[14], state.Transform[13]);

                double outputDistance = 0.0;
                double outputAmplitudeForFirstPoint = 0.0; // Amplitude for the first point display

                if (deviceSpaceTargetVectors.Count > 0)
                {
                    var firstDeviceSpaceTarget = deviceSpaceTargetVectors[0];
                    var firstTargetAsPoint3d = new Point3d(firstDeviceSpaceTarget.X, firstDeviceSpaceTarget.Y, firstDeviceSpaceTarget.Z);
                    outputDistance = currentDevicePosition.DistanceTo(firstTargetAsPoint3d);

                    // Calculate the individual amplitude for the first point for display purposes.
                    // This uses overallMaxAmplitude as the potential max for this single point's calculation.
                    if (invertMapping)
                    {
                        if (outputDistance >= maxDistance) outputAmplitudeForFirstPoint = 0.0;
                        else if (outputDistance < deadzone) outputAmplitudeForFirstPoint = overallMaxAmplitude;
                        else
                        {
                            double usableRange = maxDistance - deadzone;
                            outputAmplitudeForFirstPoint = (1.0 - (outputDistance - deadzone) / usableRange) * overallMaxAmplitude;
                        }
                    }
                    else
                    {
                        if (outputDistance >= maxDistance) outputAmplitudeForFirstPoint = overallMaxAmplitude;
                        else if (outputDistance < deadzone) outputAmplitudeForFirstPoint = 0.0;
                        else
                        {
                            double usableRange = maxDistance - deadzone;
                            outputAmplitudeForFirstPoint = ((outputDistance - deadzone) / usableRange) * overallMaxAmplitude;
                        }
                    }
                    outputAmplitudeForFirstPoint = Math.Max(0.0, Math.Min(outputAmplitudeForFirstPoint, overallMaxAmplitude));
                }

                var currentTime = DateTime.Now;
                if ((currentTime - lastUpdateTime).TotalMilliseconds >= updateInterval)
                {
                    lastDistance = outputDistance;
                    lastAmplitude = outputAmplitudeForFirstPoint;
                    lastUpdateTime = currentTime;
                }
                DA.SetData(0, lastDistance);
                DA.SetData(1, lastAmplitude);
            }
            finally
            {
                state.ReturnArrays();
            }
        }

        protected override System.Drawing.Bitmap Icon => null;
        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c71b"); // Keep your existing GUID
    }
}
