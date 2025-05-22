using Grasshopper.Kernel;
using Grasshopper.Kernel.Types; // For GH_Point
using Rhino.Geometry;
using System;
using System.Collections.Generic; // For List
using System.Linq; // For .Select

namespace ghoh
{
    public class ghohPullToPoint : GH_Component
    {
        private DateTime lastUpdateTime = DateTime.MinValue;
        private Vector3d lastForceVectorForOutput = Vector3d.Zero; // For the GH output

        public ghohPullToPoint() : base(
            "ghohPullToPoint",
            "PullPoints", // Changed name slightly
            "Pulls the haptic device towards the closest active point from a list of target points. Features adjustable snapping range with falloff.",
            "ghoh",
            "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enable", "E", "Enable or disable force application", GH_ParamAccess.item, false); // 0
            pManager.AddPointParameter("Targets", "T", "Target points to pull towards", GH_ParamAccess.list); // 1 (Changed to list)
            pManager.AddNumberParameter("MaxForce", "F", "Maximum force to apply for any single point", GH_ParamAccess.item, 1.0); // 2
            pManager.AddNumberParameter("MaxDistance", "D", "Distance at which force becomes constant (or starts to falloff)", GH_ParamAccess.item, 1.0); // 3
            pManager.AddNumberParameter("FalloffDistance", "FO", "Distance beyond MaxDistance over which pull force fades to zero. If 0 or negative, original behavior applies.", GH_ParamAccess.item, 0.0); // 4 (New)
            pManager.AddTransformParameter("Transform", "X", "Transform matrix for world to device space", GH_ParamAccess.item); // 5
            pManager.AddBooleanParameter("UseSmoothing", "S", "Enable position smoothing for smoother transitions to target points", GH_ParamAccess.item, false); // 6
            pManager.AddNumberParameter("MaxStep", "MS", "Maximum distance a smoothed target can move per update (larger = less smoothing)", GH_ParamAccess.item, 5.0); // 7
            pManager.AddNumberParameter("UpdateInterval", "U", "Output force vector update interval in milliseconds (10-5000)", GH_ParamAccess.item, 100.0); // 8

            pManager[4].Optional = true;  // FalloffDistance
            pManager[5].Optional = true;  // Transform
            pManager[6].Optional = true;  // UseSmoothing
            pManager[7].Optional = true;  // MaxStep
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("Force", "F", "Current force vector towards the most influential point", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var handle = DeviceManager.DeviceHandle;
            if (handle == HDdll.HD_INVALID_HANDLE)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Device not initialized");
                // Ensure ForceManager is cleared if device becomes uninitialized
                ForceManager.SetMultiPullToPoints(new List<DeviceManager.Vector3D>(), false, 0, 0, 0, false, 0);
                DA.SetData(0, Vector3d.Zero);
                return;
            }

            bool enable = false;
            List<GH_Point> ghTargetPoints = new List<GH_Point>();
            double maxForce = 1.0;
            double maxDistance = 1.0;
            double falloffDistance = 0.0;
            Transform worldToDevice = Transform.Identity;
            bool useSmoothing = false;
            double maxStep = 5.0;
            double updateInterval = 100.0;

            if (!DA.GetData(0, ref enable)) return;
            if (!DA.GetDataList(1, ghTargetPoints)) return;

            List<Point3d> targetPoints = ghTargetPoints.Where(gh_pt => gh_pt != null && gh_pt.Value.IsValid).Select(gh_pt => gh_pt.Value).ToList();

            if ((targetPoints == null || targetPoints.Count == 0) && enable)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "No valid target points provided while component is enabled. Disabling pull force.");
                ForceManager.SetMultiPullToPoints(new List<DeviceManager.Vector3D>(), false, 0, 0, 0, false, 0);
                DA.SetData(0, Vector3d.Zero);
                return;
            }
            if ((targetPoints == null || targetPoints.Count == 0) && !enable)
            {
                ForceManager.SetMultiPullToPoints(new List<DeviceManager.Vector3D>(), false, 0, 0, 0, false, 0);
                DA.SetData(0, Vector3d.Zero);
                return;
            }

            if (!DA.GetData(2, ref maxForce)) return;
            if (!DA.GetData(3, ref maxDistance)) return;
            DA.GetData(4, ref falloffDistance);
            DA.GetData(5, ref worldToDevice);
            DA.GetData(6, ref useSmoothing);
            DA.GetData(7, ref maxStep);
            DA.GetData(8, ref updateInterval);

            updateInterval = Math.Max(10.0, Math.Min(5000.0, updateInterval));
            maxStep = Math.Max(0.1, maxStep);
            maxForce = Math.Max(0.0, maxForce);
            maxDistance = Math.Max(0.001, maxDistance);
            falloffDistance = Math.Max(0.0, falloffDistance);


            List<DeviceManager.Vector3D> transformedTargetVectors = new List<DeviceManager.Vector3D>();
            if (targetPoints != null)
            {
                Transform deviceToWorldInverse = Transform.Identity;
                bool useInverse = !worldToDevice.Equals(Transform.Identity) && worldToDevice.TryGetInverse(out deviceToWorldInverse);

                foreach (Point3d p_world in targetPoints)
                {
                    Point3d p_transformed = p_world;
                    if (useInverse)
                    {
                        p_transformed.Transform(deviceToWorldInverse);
                    }
                    transformedTargetVectors.Add(new DeviceManager.Vector3D(
                        p_transformed.X,
                        p_transformed.Y,
                        p_transformed.Z
                    ));
                }
            }

            ForceManager.SetMultiPullToPoints(
                transformedTargetVectors,
                enable,
                maxForce,
                maxDistance,
                falloffDistance,
                useSmoothing,
                maxStep
            );

            var currentTime = DateTime.Now;
            bool shouldUpdateOutput = (currentTime - lastUpdateTime).TotalMilliseconds >= updateInterval;

            if (shouldUpdateOutput)
            {
                lastForceVectorForOutput = Vector3d.Zero;
                if (enable && transformedTargetVectors.Count > 0)
                {
                    DeviceManager.DeviceState state = DeviceManager.GetCurrentState();
                    if (state.Transform != null)
                    {
                        try
                        {
                            Point3d devicePosition_Rhino = new Point3d(
                                -state.Transform[12],
                                 state.Transform[14],
                                 state.Transform[13]
                            );

                            double strongestForceMag = -1.0;
                            Vector3d dominantForceVec_Rhino = Vector3d.Zero;

                            // For display, we use the raw targets or a simplified smoothing.
                            // The actual smoothing for force calculation happens in ForceManager.
                            // Here, we use the transformed (but not ForceManager-smoothed) targets.
                            List<Point3d> displayTargets_Rhino = transformedTargetVectors.Select(tv => new Point3d(tv.X, tv.Y, tv.Z)).ToList();

                            for (int i = 0; i < displayTargets_Rhino.Count; i++)
                            {
                                Point3d target_Rhino_pt = displayTargets_Rhino[i];
                                Vector3d vecToTarget_Rhino = target_Rhino_pt - devicePosition_Rhino;
                                double dist = vecToTarget_Rhino.Length;
                                double currentForceMag = 0;

                                if (dist < 0.001) { /* on point */ }
                                else if (falloffDistance > 0.001)
                                {
                                    double totalAttractionRange = maxDistance + falloffDistance;
                                    if (dist < totalAttractionRange)
                                    {
                                        if (dist > maxDistance)
                                        {
                                            double falloffProgress = (dist - maxDistance) / falloffDistance;
                                            currentForceMag = maxForce * (1.0 - falloffProgress);
                                        }
                                        else
                                        {
                                            currentForceMag = maxForce * (dist / maxDistance);
                                        }
                                    }
                                }
                                else
                                {
                                    if (dist <= maxDistance)
                                    {
                                        currentForceMag = maxForce * (dist / maxDistance);
                                    }
                                    else
                                    {
                                        currentForceMag = maxForce;
                                    }
                                }
                                currentForceMag = Math.Max(0, Math.Min(currentForceMag, maxForce));

                                if (currentForceMag > strongestForceMag)
                                {
                                    strongestForceMag = currentForceMag;
                                    if (dist > 0.001) vecToTarget_Rhino.Unitize();
                                    dominantForceVec_Rhino = vecToTarget_Rhino * strongestForceMag;
                                }
                            }
                            lastForceVectorForOutput = dominantForceVec_Rhino;
                        }
                        finally
                        {
                            state.ReturnArrays();
                        }
                    }
                }
                lastUpdateTime = currentTime;
            }
            DA.SetData(0, lastForceVectorForOutput);
        }

        protected override System.Drawing.Bitmap Icon => null;

        // Generate a new GUID for this modified component
        public override Guid ComponentGuid => new Guid("a0b8e4f2-1c7d-4f8e-b9f9-3d1c9a8b7e6f");
    }
}