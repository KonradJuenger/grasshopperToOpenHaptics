using Grasshopper.Kernel;
using Rhino.Geometry;
using System;

namespace ghoh
{
    public class ghohPullToPoint : GH_Component
    {
        public ghohPullToPoint() : base(
            "ghohPullToPoint",
            "PullPoint",
            "Pulls the haptic device to a point with proportional force based on distance",
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
            pManager.AddBooleanParameter("Interpolate", "I", "Enable target interpolation for smoother transitions", GH_ParamAccess.item, false);

            // Make transform and interpolation parameters optional
            pManager[4].Optional = true;  // Transform
            pManager[5].Optional = true;  // Interpolate
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("Force", "F", "Current force vector", GH_ParamAccess.item);
            pManager.AddNumberParameter("Distance", "L", "Distance to target", GH_ParamAccess.item);
            pManager.AddPointParameter("DevicePos", "P", "Current device position", GH_ParamAccess.item);
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
            bool interpolate = false;
            DA.GetData(5, ref interpolate);

            if (!DA.GetData(0, ref enable)) return;
            if (!DA.GetData(1, ref target)) return;
            if (!DA.GetData(2, ref maxForce)) return;
            if (!DA.GetData(3, ref maxDistance)) return;
            DA.GetData(4, ref worldToDevice);

            // Get current state before transforming target
            var state = DeviceManager.GetCurrentState();

            try
            {
                // Get device position in its native coordinate system
                var devicePosition = new Point3d(
                    -state.Transform[12],    // -X (negative to match coordinate system)
                    state.Transform[14],     // Z becomes Y
                    state.Transform[13]      // Y becomes Z
                );

                Point3d transformedDevicePos = devicePosition;
                Point3d transformedTarget = target;

                // Apply world-to-device transformation if provided
                if (!worldToDevice.Equals(Transform.Identity))
                {
                    transformedDevicePos.Transform(worldToDevice);

                    Transform deviceToWorld = worldToDevice;
                    if (deviceToWorld.TryGetInverse(out deviceToWorld))
                    {
                        transformedTarget.Transform(deviceToWorld);
                    }
                }

                // Calculate direction and distance for visualization
                Vector3d currentForce = Vector3d.Zero;
                double distance = transformedDevicePos.DistanceTo(transformedTarget);

                if (enable && distance > 0.001)
                {
                    // Calculate direction
                    Vector3d direction = transformedTarget - transformedDevicePos;
                    direction.Unitize();

                    // Calculate force magnitude
                    double forceMagnitude = distance > maxDistance ? maxForce : maxForce * (distance / maxDistance);

                    // Calculate force vector for visualization
                    currentForce = direction * forceMagnitude;
                }

                // Update target in DeviceManager for servo loop
                var targetVector = new DeviceManager.Vector3D(
                    transformedTarget.X,
                    transformedTarget.Y,
                    transformedTarget.Z
                );

                DeviceManager.UpdateTargetPoint(targetVector, enable, maxForce, maxDistance, interpolate);
                
                // Output results
                DA.SetData(0, currentForce);
                DA.SetData(1, distance);
                DA.SetData(2, transformedDevicePos);
            }
            finally
            {
                state.ReturnArrays();
            }
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c69d");
    }
}