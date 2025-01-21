using Grasshopper.Kernel;
using Rhino.Geometry;
using System;

namespace ghoh
{
    public class ghohPullToPoint : GH_Component
    {
        private DateTime lastLogTime = DateTime.MinValue;
        private const double LOG_INTERVAL_SECONDS = 0.5;

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
            pManager[4].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("Force", "F", "Current force vector", GH_ParamAccess.item);
            pManager.AddNumberParameter("Distance", "L", "Distance to target", GH_ParamAccess.item);
            pManager.AddPointParameter("DevicePos", "P", "Current device position", GH_ParamAccess.item);
        }

        private bool ShouldLog()
        {
            var now = DateTime.Now;
            if ((now - lastLogTime).TotalSeconds >= LOG_INTERVAL_SECONDS)
            {
                lastLogTime = now;
                return true;
            }
            return false;
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

            bool shouldLogThisIteration = ShouldLog();

            // Update force parameters in DeviceManager
            var parameters = new ForceParameters
            {
                Enabled = enable,
                Target = target,
                MaxForce = maxForce,
                MaxDistance = maxDistance,
                WorldToDevice = worldToDevice
            };

            DeviceManager.UpdateForceParameters(parameters);

            if (shouldLogThisIteration)
            {
                Logger.Log($"PullToPoint - Parameters updated: Enable={enable}, Target={target}, MaxForce={maxForce}, MaxDistance={maxDistance}");
            }

            // Get current state for output parameters
            var state = DeviceManager.GetCurrentState();
            try
            {
                // Get current device position in world space
                var devicePosition = new Point3d(
                    -state.Transform[12],
                    state.Transform[14],
                    state.Transform[13]
                );

                Point3d deviceInWorldSpace = devicePosition;
                if (!worldToDevice.Equals(Transform.Identity))
                {
                    deviceInWorldSpace.Transform(worldToDevice);
                }

                // Calculate current direction and distance to target (for output only)
                var directionToTarget = target - deviceInWorldSpace;
                var distance = directionToTarget.Length;

                // Calculate current force vector (for output only)
                Vector3d totalForce = Vector3d.Zero;
                if (enable && distance > 0.001)
                {
                    var normalizedDir = directionToTarget / distance;

                    if (distance > maxDistance)
                    {
                        totalForce = normalizedDir * maxForce;
                    }
                    else
                    {
                        var scale = maxForce * (distance / maxDistance);
                        totalForce = normalizedDir * scale;
                    }
                }

                DA.SetData(0, totalForce);
                DA.SetData(1, distance);
                DA.SetData(2, deviceInWorldSpace);

                if (shouldLogThisIteration)
                {
                    Logger.Log($"PullToPoint - Outputs: Force={totalForce}, Distance={distance}, DevicePos={deviceInWorldSpace}");
                }
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