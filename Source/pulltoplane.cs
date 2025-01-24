using Grasshopper.Kernel;
using Rhino.Geometry;
using System;

namespace ghoh
{
    public class ghohPullToPlane : GH_Component
    {
        public ghohPullToPlane() : base(
            "ghohPullToPlane",
            "PullPlane",
            "Constrains the haptic device movement to a plane with proportional force based on distance",
            "ghoh",
            "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enable", "E", "Enable or disable force application", GH_ParamAccess.item, false);
            pManager.AddPlaneParameter("ConstraintPlane", "P", "Plane to constrain movement to", GH_ParamAccess.item);
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
            Plane constraintPlane = Plane.WorldXY;
            double maxForce = 1.0;
            double maxDistance = 1.0;
            Transform worldToDevice = Transform.Identity;

            if (!DA.GetData(0, ref enable)) return;
            if (!DA.GetData(1, ref constraintPlane)) return;
            if (!DA.GetData(2, ref maxForce)) return;
            if (!DA.GetData(3, ref maxDistance)) return;
            DA.GetData(4, ref worldToDevice);

            // Simply update the plane constraint parameters in DeviceManager
            DeviceManager.UpdatePlaneConstraint(
                constraintPlane.Origin,
                constraintPlane.Normal,
                worldToDevice,
                enable
            );

            // Update force parameters
            var currentPos = new DeviceManager.Vector3D(0, 0, 0); // Position doesn't matter for plane constraint
            DeviceManager.UpdateTargetPoint(currentPos, enable, maxForce, maxDistance, false, 0);
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("f5937550-b8e1-4dd3-a7c2-1e001933d79f");
    }
}