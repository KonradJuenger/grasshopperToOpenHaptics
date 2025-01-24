using Grasshopper.Kernel;
using Rhino.Geometry;
using System;

namespace ghoh
{
    public class ghohPullToPointSimple : GH_Component
    {
        public ghohPullToPointSimple() : base(
            "ghohPullToPointSimple",
            "PullPointSimple",
            "Simplified version that pulls the haptic device to a point with proportional force based on distance",
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
            // No outputs
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
            Point3d target = Point3d.Origin;
            double maxForce = 1.0;
            double maxDistance = 1.0;
            Transform worldToDevice = Transform.Identity;

            if (!DA.GetData(0, ref enable)) return;
            if (!DA.GetData(1, ref target)) return;
            if (!DA.GetData(2, ref maxForce)) return;
            if (!DA.GetData(3, ref maxDistance)) return;
            DA.GetData(4, ref worldToDevice);

            var state = DeviceManager.GetCurrentState();

            try
            {
                // Get device position in its native coordinate system
                var devicePosition = new Point3d(
                    -state.Transform[12],    // -X (negative to match coordinate system)
                    state.Transform[14],     // Z becomes Y
                    state.Transform[13]      // Y becomes Z
                );

                // Transform target to device space if transform provided
                Point3d transformedTarget = target;
                if (!worldToDevice.Equals(Transform.Identity))
                {
                    Transform deviceToWorld = worldToDevice;
                    if (deviceToWorld.TryGetInverse(out deviceToWorld))
                    {
                        transformedTarget.Transform(deviceToWorld);
                    }
                }

                // Calculate direction and distance (both points in device space)
                double distance = devicePosition.DistanceTo(transformedTarget);
                Vector3d currentForce = Vector3d.Zero;

                if (enable && distance > 0.001)
                {
                    Vector3d direction = transformedTarget - devicePosition;
                    direction.Unitize();
                    double forceMagnitude = distance > maxDistance ? maxForce : maxForce * (distance / maxDistance);
                    currentForce = direction * forceMagnitude;
                }

                // Update target in DeviceManager for servo loop
                var targetVector = new DeviceManager.Vector3D(
                    transformedTarget.X,
                    transformedTarget.Y,
                    transformedTarget.Z
                );

                DeviceManager.UpdateTargetPoint(targetVector, enable, maxForce, maxDistance, false, 0);
            }
            finally
            {
                state.ReturnArrays();
            }
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c69e");
    }
}