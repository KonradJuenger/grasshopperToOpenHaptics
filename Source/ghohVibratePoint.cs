using Grasshopper.Kernel;
using Grasshopper.Kernel.Types; // For GH_Point
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;

namespace ghoh
{
    public class ghohVibratePoint : GH_Component
    {
        private DateTime lastUpdateTime = DateTime.MinValue;
        // The cached values are now in the consistent "Rhino Haptic" coordinate space
        private List<double> lastDistances_RhinoSpace = new List<double>();
        private List<double> lastIndividualAmplitudes = new List<double>();
        private double lastResultantAmplitude = 0.0;
        private double lastEffectiveAmplitudeFactor = 0.0;
        private int lastDominantPointIndex = -1;

        public ghohVibratePoint() : base(
            "ghohVibratePoint",
            "VibratePoints",
            "Vibration based on proximity to multiple target points. The transformation logic now matches ghohPullToPoint, performing calculations in a consistent haptic coordinate space.",
            "ghoh",
            "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enable", "E", "Enable or disable vibration", GH_ParamAccess.item, false); // 0
            pManager.AddPointParameter("Targets", "P", "Target points (World Coordinates)", GH_ParamAccess.list); // 1
            pManager.AddVectorParameter("Direction", "Dir", "Vibration direction vector (World Coordinates)", GH_ParamAccess.item, new Vector3d(0, 0, 1)); // 2
            pManager.AddNumberParameter("Deadzone", "DZ", "Distance (haptic units) to point with no vibration (normal) or max vibration (inverted)", GH_ParamAccess.item, 0.0); // 3
            pManager.AddNumberParameter("MaxDistance", "MD", "Maximum distance (haptic units) for vibration effect. Must be greater than Deadzone.", GH_ParamAccess.item, 10.0); // 4
            pManager.AddNumberParameter("MaxAmplitude", "MA", "Maximum vibration amplitude (force units)", GH_ParamAccess.item, 1.0); // 5
            pManager.AddNumberParameter("Frequency", "F", "Vibration frequency in Hz (1-1000)", GH_ParamAccess.item, 100.0); // 6
            pManager.AddBooleanParameter("Invert", "I", "Invert distance mapping (vibrate strong when close)", GH_ParamAccess.item, false); // 7
            pManager.AddBooleanParameter("SquareWave", "SW", "Use square wave instead of sine wave", GH_ParamAccess.item, false); // 8
            pManager.AddTransformParameter("Transform", "X", "Transform matrix from World to Device space (same as PullToPoint)", GH_ParamAccess.item); // 9 
            pManager.AddNumberParameter("UpdateInterval", "U", "Output update interval (ms, 10-5000)", GH_ParamAccess.item, 100.0); // 10

            // Optional inputs
            pManager[2].Optional = true;
            pManager[3].Optional = true;
            pManager[4].Optional = true;
            pManager[5].Optional = true;
            pManager[6].Optional = true;
            pManager[7].Optional = true;
            pManager[8].Optional = true;
            pManager[9].Optional = true;
            pManager[10].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Distances (Haptic)", "D", "Current distances from device (TCP) to each target point (in haptic space units)", GH_ParamAccess.list);
            pManager.AddNumberParameter("IndividualAmplitudes", "IA", "Calculated amplitude contribution from each point (before min/max logic, in force units)", GH_ParamAccess.list);
            pManager.AddNumberParameter("ResultantAmplitude", "RA", "Final vibration amplitude applied (after min/max logic, in force units)", GH_ParamAccess.item);
            pManager.AddNumberParameter("EffectiveFactor", "EF", "Effective amplitude factor (0-1) after min/max logic, based on dominant point", GH_ParamAccess.item);
            pManager.AddIntegerParameter("DominantIndex", "DI", "Index of the target point determining the resultant amplitude (-1 if none or disabled)", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var handle = DeviceManager.DeviceHandle;
            if (handle == HDdll.HD_INVALID_HANDLE)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Haptic device not initialized.");
                ForceManager.SetVibratePoints(new List<DeviceManager.Vector3D>(), new DeviceManager.Vector3D(0, 0, 1), false, 0, 0, 0, 0, false, false);
                ClearOutputsAndSetDA(DA);
                return;
            }

            bool enable = false;
            List<GH_Point> ghTargetPoints_world = new List<GH_Point>();
            Vector3d direction_world = new Vector3d(0, 0, 1);
            double deadzone = 0.0;
            double maxDistance = 10.0;
            double maxAmplitude = 1.0;
            double frequency = 100.0;
            bool invertMapping = false;
            bool useSquareWave = false;
            Transform worldToDeviceTransform = Transform.Identity;
            double updateInterval = 100.0;

            DA.GetData(0, ref enable);
            DA.GetDataList(1, ghTargetPoints_world);
            DA.GetData(2, ref direction_world);
            DA.GetData(3, ref deadzone);
            DA.GetData(4, ref maxDistance);
            DA.GetData(5, ref maxAmplitude);
            DA.GetData(6, ref frequency);
            DA.GetData(7, ref invertMapping);
            DA.GetData(8, ref useSquareWave);
            DA.GetData(9, ref worldToDeviceTransform);
            DA.GetData(10, ref updateInterval);

            List<Point3d> targetPoints_world_valid = ghTargetPoints_world
                .Where(gh_pt => gh_pt != null && gh_pt.Value.IsValid)
                .Select(gh_pt => gh_pt.Value)
                .ToList();

            // Validate and sanitize parameters
            updateInterval = Math.Max(10.0, Math.Min(5000.0, updateInterval));
            deadzone = Math.Max(0.0, deadzone);
            maxDistance = Math.Max(deadzone + 0.001, maxDistance);
            maxAmplitude = Math.Max(0.0, maxAmplitude);
            frequency = Math.Max(1.0, Math.Min(1000.0, frequency));
            if (!direction_world.Unitize()) direction_world = new Vector3d(0, 0, 1);

            // --- BUG FIX: Match the transformation logic from ghohPullToPoint ---
            // The points and direction are transformed into the "Rhino Haptic" coordinate system.
            Transform deviceToWorldInverse = Transform.Identity;
            bool useInverse = !worldToDeviceTransform.Equals(Transform.Identity) && worldToDeviceTransform.TryGetInverse(out deviceToWorldInverse);

            List<Point3d> targetPoints_rhino_pt3d = new List<Point3d>();
            Vector3d direction_rhino_vec3d = direction_world;

            if (useInverse)
            {
                foreach (Point3d p_world in targetPoints_world_valid)
                {
                    Point3d p_transformed = p_world;
                    p_transformed.Transform(deviceToWorldInverse); // Apply inverse transform to match pullToPoint
                    targetPoints_rhino_pt3d.Add(p_transformed);
                }
                direction_rhino_vec3d.Transform(deviceToWorldInverse); // Also transform the direction vector
            }
            else
            {
                targetPoints_rhino_pt3d.AddRange(targetPoints_world_valid); // No transform needed
            }

            if (!direction_rhino_vec3d.Unitize()) direction_rhino_vec3d = new Vector3d(0, 0, 1);

            // Convert to DeviceManager types for ForceManager
            List<DeviceManager.Vector3D> targetPoints_rhino_dm = targetPoints_rhino_pt3d
                .Select(pt => new DeviceManager.Vector3D(pt.X, pt.Y, pt.Z))
                .ToList();
            DeviceManager.Vector3D direction_rhino_dm = new DeviceManager.Vector3D(
                direction_rhino_vec3d.X, direction_rhino_vec3d.Y, direction_rhino_vec3d.Z);

            bool systemShouldBeActive = enable && targetPoints_rhino_dm.Count > 0;

            // Update ForceManager with haptic-space coordinates and parameters
            // SetVibratePoints now expects targets and direction in "Rhino Haptic" coordinates.
            ForceManager.SetVibratePoints(
                targetPoints_rhino_dm,
                direction_rhino_dm,
                systemShouldBeActive,
                deadzone, maxDistance, maxAmplitude, frequency, invertMapping, useSquareWave
            );

            DateTime currentTime = DateTime.Now;
            if ((currentTime - lastUpdateTime).TotalMilliseconds >= updateInterval)
            {
                if (systemShouldBeActive)
                {
                    lastDistances_RhinoSpace.Clear();
                    lastIndividualAmplitudes.Clear();

                    DeviceManager.DeviceState state = DeviceManager.GetCurrentState();
                    if (state.Transform != null)
                    {
                        try
                        {
                            // --- BUG FIX: Perform output calculations in the correct coordinate space ---
                            // Get device position in Native Device Coordinates.
                            Point3d devicePosition_native = new Point3d(state.Transform[12], state.Transform[13], state.Transform[14]);

                            // Convert to "Rhino Haptic" space to match ForceManager calculations.
                            Point3d devicePosition_rhino = new Point3d(-devicePosition_native.X, devicePosition_native.Z, devicePosition_native.Y);

                            List<double> currentAmplitudeFactors = new List<double>();

                            for (int i = 0; i < targetPoints_rhino_pt3d.Count; i++)
                            {
                                Point3d targetPt_rhino = targetPoints_rhino_pt3d[i]; // Already in Rhino Haptic space
                                double dist_rhino = devicePosition_rhino.DistanceTo(targetPt_rhino); // Distance in haptic space units
                                lastDistances_RhinoSpace.Add(dist_rhino);

                                double amplitudeFactor;
                                double usableRange = maxDistance - deadzone;
                                if (usableRange < 0.001) usableRange = 0.001;

                                if (invertMapping) // Vibrate strong when close
                                {
                                    if (dist_rhino >= maxDistance) amplitudeFactor = 0.0;
                                    else if (dist_rhino <= deadzone) amplitudeFactor = 1.0;
                                    else amplitudeFactor = 1.0 - ((dist_rhino - deadzone) / usableRange);
                                }
                                else // Vibrate strong when far
                                {
                                    if (dist_rhino >= maxDistance) amplitudeFactor = 1.0;
                                    else if (dist_rhino <= deadzone) amplitudeFactor = 0.0;
                                    else amplitudeFactor = (dist_rhino - deadzone) / usableRange;
                                }
                                amplitudeFactor = Math.Max(0.0, Math.Min(1.0, amplitudeFactor));
                                currentAmplitudeFactors.Add(amplitudeFactor);
                                lastIndividualAmplitudes.Add(amplitudeFactor * maxAmplitude);
                            }

                            if (currentAmplitudeFactors.Count > 0)
                            {
                                if (invertMapping) // Strong when close: highest factor (closest point) dominates
                                {
                                    lastEffectiveAmplitudeFactor = currentAmplitudeFactors.Max();
                                }
                                else // Strong when far: lowest factor (closest point) dominates
                                {
                                    lastEffectiveAmplitudeFactor = currentAmplitudeFactors.Min();
                                }
                                lastDominantPointIndex = currentAmplitudeFactors.IndexOf(lastEffectiveAmplitudeFactor);
                                lastResultantAmplitude = lastEffectiveAmplitudeFactor * maxAmplitude;
                            }
                            else // No target points
                            {
                                ClearCachedOutputs();
                            }
                        }
                        finally
                        {
                            state.ReturnArrays();
                        }
                    }
                    else // Could not get device state
                    {
                        ClearCachedOutputs();
                    }
                }
                else // System is not active
                {
                    ClearCachedOutputs();
                }
                lastUpdateTime = currentTime;
            }

            DA.SetDataList(0, lastDistances_RhinoSpace);
            DA.SetDataList(1, lastIndividualAmplitudes);
            DA.SetData(2, lastResultantAmplitude);
            DA.SetData(3, lastEffectiveAmplitudeFactor);
            DA.SetData(4, lastDominantPointIndex);
        }

        private void ClearCachedOutputs()
        {
            if (lastDistances_RhinoSpace.Any() || lastIndividualAmplitudes.Any() ||
                lastResultantAmplitude != 0.0 || lastEffectiveAmplitudeFactor != 0.0 || lastDominantPointIndex != -1)
            {
                lastDistances_RhinoSpace.Clear();
                lastIndividualAmplitudes.Clear();
                lastResultantAmplitude = 0.0;
                lastEffectiveAmplitudeFactor = 0.0;
                lastDominantPointIndex = -1;
            }
        }

        private void ClearOutputsAndSetDA(IGH_DataAccess DA)
        {
            ClearCachedOutputs();
            DA.SetDataList(0, lastDistances_RhinoSpace);
            DA.SetDataList(1, lastIndividualAmplitudes);
            DA.SetData(2, lastResultantAmplitude);
            DA.SetData(3, lastEffectiveAmplitudeFactor);
            DA.SetData(4, lastDominantPointIndex);
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("DB4A7C1F-7D8B-4A3E-9E1A-5B6C2F0A8B3D"); // Keep original GUID

        public override GH_Exposure Exposure => GH_Exposure.primary;
    }
}
