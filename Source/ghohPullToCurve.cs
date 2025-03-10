using Grasshopper.Kernel;
using Rhino.Geometry;
using System;

namespace ghoh
{
    public class ghohPullToCurve : GH_Component
    {
        public ghohPullToCurve() : base(
            "ghohPullToCurve",
            "PullCurve",
            "Pulls the haptic device towards the closest point on a curve, with option to pull ahead on the curve",
            "ghoh",
            "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enable", "E", "Enable or disable force application", GH_ParamAccess.item, false);
            pManager.AddCurveParameter("Target", "T", "Target curve to pull towards", GH_ParamAccess.item);
            pManager.AddNumberParameter("MaxForce", "F", "Maximum force to apply", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("MaxDistance", "D", "Distance at which force becomes constant", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("LookAhead", "L", "Distance to look ahead on curve in mm", GH_ParamAccess.item, 0.01);
            pManager.AddTransformParameter("Transform", "X", "Transform matrix for world to device space", GH_ParamAccess.item);
            pManager.AddVectorParameter("TCPOffset", "O", "Optional offset vector from TCP in device coordinates", GH_ParamAccess.item, Vector3d.Zero);

            pManager[4].Optional = true;  // LookAhead
            pManager[5].Optional = true;  // Transform
            pManager[6].Optional = true;  // TCPOffset
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
            Curve targetCurve = null;
            double maxForce = 1.0;
            double maxDistance = 1.0;
            double lookAhead = 0.01;
            Transform worldToDevice = Transform.Identity;
            Vector3d tcpOffset = Vector3d.Zero;

            if (!DA.GetData(0, ref enable)) return;
            if (!DA.GetData(1, ref targetCurve)) return;
            if (targetCurve == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Invalid curve input");
                return;
            }
            if (!DA.GetData(2, ref maxForce)) return;
            if (!DA.GetData(3, ref maxDistance)) return;
            DA.GetData(4, ref lookAhead);
            DA.GetData(5, ref worldToDevice);
            DA.GetData(6, ref tcpOffset);

            // Transform curve to device space if transform provided
            Curve transformedCurve = targetCurve;
            if (!worldToDevice.Equals(Transform.Identity))
            {
                Transform deviceToWorld;
                if (worldToDevice.TryGetInverse(out deviceToWorld))
                {
                    transformedCurve = targetCurve.DuplicateCurve();
                    transformedCurve.Transform(deviceToWorld);
                }
            }

            // Set TCP offset first
            var offsetVector = new DeviceManager.Vector3D(
                tcpOffset.X,
                tcpOffset.Y,
                tcpOffset.Z
            );
            ForceManager.SetTCPOffset(offsetVector);

            // Pass curve to ForceManager
            ForceManager.SetPullToCurve(
                transformedCurve,
                enable,
                maxForce,
                maxDistance,
                lookAhead
            );
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c73d");
    }
}