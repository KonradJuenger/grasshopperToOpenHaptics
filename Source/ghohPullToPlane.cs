using Grasshopper.Kernel;
using Rhino.Geometry;
using System;

namespace ghoh
{
    public class ghohPullToPlane : GH_Component
    {
        public ghohPullToPlane() : base(
            "ghohPullToPlaneSimple",
            "PullPlaneSimple",
            "Simplified version that pulls the haptic device towards a plane with proportional force based on distance",
            "ghoh",
            "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enable", "E", "Enable or disable force application", GH_ParamAccess.item, false);
            pManager.AddPlaneParameter("Target", "T", "Target plane to pull towards", GH_ParamAccess.item);
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
            Plane targetPlane = Plane.WorldXY;
            double maxForce = 1.0;
            double maxDistance = 1.0;
            Transform worldToDevice = Transform.Identity;

            if (!DA.GetData(0, ref enable)) return;
            if (!DA.GetData(1, ref targetPlane)) return;
            if (!DA.GetData(2, ref maxForce)) return;
            if (!DA.GetData(3, ref maxDistance)) return;
            DA.GetData(4, ref worldToDevice);

            // Transform target plane to device space if transform provided
            if (!worldToDevice.Equals(Transform.Identity))
            {
                Transform deviceToWorld;
                if (worldToDevice.TryGetInverse(out deviceToWorld))
                {
                    targetPlane.Transform(deviceToWorld);
                }
            }

            // Ensure the plane normal is unitized
            targetPlane.Normal.Unitize();

            // Convert plane parameters to device space vectors
            var origin = new DeviceManager.Vector3D(
                targetPlane.Origin.X,
                targetPlane.Origin.Y,
                targetPlane.Origin.Z
            );
            var normal = new DeviceManager.Vector3D(
                targetPlane.Normal.X,
                targetPlane.Normal.Y,
                targetPlane.Normal.Z
            );

            // Update force through ForceManager
            ForceManager.SetPullToPlane(
                origin,
                normal,
                enable,
                maxForce,
                maxDistance
            );
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c69f");
    }
}