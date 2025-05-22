using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic; // Required for List

namespace ghoh
{
    public class ghohPullToPointSimple : GH_Component
    {
        public ghohPullToPointSimple() : base(
            "ghohPullToPointSimple",
            "PullPointSimple",
            "Simplified version that pulls the haptic device to a single point with proportional force based on distance. No falloff beyond MaxDistance, no smoothing.",
            "ghoh",
            "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enable", "E", "Enable or disable force application", GH_ParamAccess.item, false);
            pManager.AddPointParameter("Target", "T", "Target point to pull towards", GH_ParamAccess.item);
            pManager.AddNumberParameter("MaxForce", "F", "Maximum force to apply", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("MaxDistance", "D", "Distance at which force becomes constant", GH_ParamAccess.item, 1.0);
            pManager.AddTransformParameter("Transform", "X", "Transform matrix for world to device space", GH_ParamAccess.item);

            pManager[4].Optional = true;  // Transform
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            // No outputs for this simplified component. 
            // If you need a force vector output for visualization, add it here
            // and implement the calculation in SolveInstance like in ghohPullToPoint.cs
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var handle = DeviceManager.DeviceHandle;
            if (handle == HDdll.HD_INVALID_HANDLE)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Device not initialized");
                // Ensure ForceManager is cleared if device becomes uninitialized and this component was active
                ForceManager.SetMultiPullToPoints(new List<DeviceManager.Vector3D>(), false, 0, 0, 0, false, 0);
                return;
            }

            bool enable = false;
            Point3d target_world = Point3d.Origin; // Target point from GH input (world space)
            double maxForce = 1.0;
            double maxDistance = 1.0;
            Transform worldToDeviceTransform = Transform.Identity; // World (GH) to Device (HDAPI native)

            if (!DA.GetData(0, ref enable)) return;
            if (!DA.GetData(1, ref target_world)) return; // Get target even if disabled
            if (!DA.GetData(2, ref maxForce)) return;
            if (!DA.GetData(3, ref maxDistance)) return;
            DA.GetData(4, ref worldToDeviceTransform);

            if (!target_world.IsValid && enable)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Invalid target point. Disabling pull force for this component.");
                ForceManager.SetMultiPullToPoints(new List<DeviceManager.Vector3D>(), false, 0, 0, 0, false, 0);
                return;
            }
            if (!target_world.IsValid && !enable) // Not enabled, invalid point, ensure FM is cleared for this component
            {
                ForceManager.SetMultiPullToPoints(new List<DeviceManager.Vector3D>(), false, 0, 0, 0, false, 0);
                return;
            }


            // Ensure parameters are valid
            maxForce = Math.Max(0.0, maxForce);
            maxDistance = Math.Max(0.001, maxDistance);

            var singleTargetList_RhinoCoords = new List<DeviceManager.Vector3D>();

            if (target_world.IsValid) // Only proceed with valid point
            {
                // Transform target from world space (GH) to Rhino-like device space for ForceManager
                Point3d target_RhinoLikeDeviceSpace = target_world;
                if (!worldToDeviceTransform.Equals(Transform.Identity))
                {
                    Transform deviceToWorld_RhinoLike_Inverse; // From device's Rhino-like system to GH World
                    if (worldToDeviceTransform.TryGetInverse(out deviceToWorld_RhinoLike_Inverse))
                    {
                        // The input 'worldToDeviceTransform' is defined as "world to device space".
                        // This typically means it takes a point in world coordinates and gives its
                        // representation in the device's native coordinate system.
                        // However, ForceManager expects points in "Rhino-like coordinates" relative
                        // to the device's origin.
                        // If 'worldToDeviceTransform' directly gives the device's native coordinates,
                        // we first apply it, then convert native device to Rhino-like.
                        // But the previous components (PullToCurve, PullToPoint) did:
                        // transformedCurve.Transform(deviceToWorld); where deviceToWorld = worldToDevice.Inverse()
                        // This implies worldToDevice is from GH_World to Device_Rhino_Like_Origin.
                        // Let's stick to that convention for consistency.
                        // So, we need the inverse of (GH_World -> Device_Rhino_Like_Origin) to transform
                        // the GH_World point into Device_Rhino_Like_Origin's coordinate system.

                        target_RhinoLikeDeviceSpace.Transform(deviceToWorld_RhinoLike_Inverse);
                    }
                }

                var targetVector_RhinoLike = new DeviceManager.Vector3D(
                    target_RhinoLikeDeviceSpace.X,
                    target_RhinoLikeDeviceSpace.Y,
                    target_RhinoLikeDeviceSpace.Z
                );

                if (enable) // Only add to the list if the component is enabled
                {
                    singleTargetList_RhinoCoords.Add(targetVector_RhinoLike);
                }
            }

            // Update force through ForceManager using the multi-point method
            ForceManager.SetMultiPullToPoints(
                singleTargetList_RhinoCoords, // This list will be empty if not enabled or target invalid
                enable && target_world.IsValid, // Final enable flag for ForceManager
                maxForce,
                maxDistance,
                0.0,     // falloffDistance = 0 for original simple behavior (constant force beyond maxDistance)
                false,   // useSmoothing = false
                0.0      // maxStep (not used if smoothing is false)
            );
        }

        protected override System.Drawing.Bitmap Icon => null;

        // Use the GUID you provided in the original file for ghohPullToPointSimple
        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c69e");
    }
}