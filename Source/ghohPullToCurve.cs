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
            "Pulls the haptic device towards the closest point on a curve, with option to pull ahead on the curve or to a traveling point",
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
            pManager.AddBooleanParameter("Fade", "Fd", "Fade force from start (0) to end (full) of curve", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("PullAlong", "P", "Enable pulling along curve (using one of the methods below)", GH_ParamAccess.item, false);
            pManager.AddIntegerParameter("Method", "M", "Pull method (0: tangent direction, 1: traveling point)", GH_ParamAccess.item, 0);
            pManager.AddNumberParameter("TangentForce", "TF", "Force multiplier for tangent direction (for method 0)", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("Speed", "S", "Speed of traveling point in mm/sec (for method 1)", GH_ParamAccess.item, 10.0);
            pManager.AddBooleanParameter("Reset", "R", "Reset traveling point to start of curve (for method 1)", GH_ParamAccess.item, false);
            pManager.AddTransformParameter("Transform", "X", "Transform matrix for world to device space", GH_ParamAccess.item);
            pManager.AddVectorParameter("TCPOffset", "O", "Optional offset vector from TCP in device coordinates", GH_ParamAccess.item, Vector3d.Zero);

            pManager[4].Optional = true;  // Fade
            pManager[5].Optional = true;  // PullAlong
            pManager[6].Optional = true;  // Method
            pManager[7].Optional = true;  // TangentForce
            pManager[8].Optional = true;  // Speed
            pManager[9].Optional = true;  // Reset
            pManager[10].Optional = true; // Transform
            pManager[11].Optional = true; // TCPOffset
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
            bool fade = false;
            bool pullAlong = false;
            int method = 0;
            double tangentForce = 1.0;
            double speed = 10.0;
            bool reset = false;
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
            DA.GetData(4, ref fade);
            DA.GetData(5, ref pullAlong);
            DA.GetData(6, ref method);
            DA.GetData(7, ref tangentForce);
            DA.GetData(8, ref speed);
            DA.GetData(9, ref reset);
            DA.GetData(10, ref worldToDevice);
            DA.GetData(11, ref tcpOffset);

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

            // Pass curve to ForceManager with enhanced parameters
            ForceManager.SetPullToCurve(
                transformedCurve,
                enable,
                maxForce,
                maxDistance,
                fade,
                method,
                tangentForce,
                pullAlong,
                speed,
                reset
            );
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c73d");
    }
}