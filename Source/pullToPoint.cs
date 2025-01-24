using Grasshopper.Kernel;
using Rhino.Geometry;
using System;

namespace ghoh
{
    public class ghohPullToPoint : GH_Component
    {
        private DateTime lastUpdateTime = DateTime.MinValue;
        private Vector3d lastForce = Vector3d.Zero;

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
            pManager.AddNumberParameter("InterpolationWindow", "W", "Time window for interpolation (milliseconds)", GH_ParamAccess.item, 30.0);
            pManager.AddNumberParameter("UpdateInterval", "U", "Output update interval in milliseconds (10-5000)", GH_ParamAccess.item, 100.0);

            pManager[4].Optional = true;  // Transform
            pManager[5].Optional = true;  // Interpolate
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("Force", "F", "Current force vector", GH_ParamAccess.item);
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
            double interpolationWindow = 30.0;
            double updateInterval = 100.0;

            if (!DA.GetData(0, ref enable)) return;
            if (!DA.GetData(1, ref target)) return;
            if (!DA.GetData(2, ref maxForce)) return;
            if (!DA.GetData(3, ref maxDistance)) return;
            DA.GetData(4, ref worldToDevice);
            DA.GetData(5, ref interpolate);
            DA.GetData(6, ref interpolationWindow);
            DA.GetData(7, ref updateInterval);

            // Clamp update interval
            updateInterval = Math.Max(10.0, Math.Min(5000.0, updateInterval));

            // Check if enough time has passed to update output
            var currentTime = DateTime.Now;
            bool shouldUpdateOutput = (currentTime - lastUpdateTime).TotalMilliseconds >= updateInterval;

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

                    // Calculate force magnitude with smoothing at maxDistance
                    double forceMagnitude;
                    if (distance >= maxDistance)
                    {
                        forceMagnitude = maxForce;
                    }
                    else
                    {
                        // Smooth transition near maxDistance using sine curve
                        double t = distance / maxDistance;
                        forceMagnitude = maxForce * (Math.Sin(t * Math.PI / 2));
                    }

                    // For display, convert device space direction to Rhino world space
                    currentForce = new Vector3d(
                        direction.Y,    // Device X -> -Rhino X
                        -direction.X,     // Device Z -> Rhino Y
                        direction.Z      // Device Y -> Rhino Z
                    ) * forceMagnitude;
                }

                // Update target in DeviceManager for servo loop (stays in device space)
                var targetVector = new DeviceManager.Vector3D(
                    transformedTarget.X,
                    transformedTarget.Y,
                    transformedTarget.Z
                );

                DeviceManager.UpdateTargetPoint(
                    targetVector,
                    enable,
                    maxForce,
                    maxDistance,
                    interpolate,
                    interpolationWindow
                );

                // Update output only if enough time has passed
                if (shouldUpdateOutput)
                {
                    lastForce = currentForce;
                    lastUpdateTime = currentTime;
                }

                // Always output last force (either updated or previous)
                DA.SetData(0, lastForce);
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