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
            "Pulls the haptic device towards the closest point on a curve. Features adjustable snapping range with falloff, and options to pull along the curve (tangentially or towards a traveling point).",
            "ghoh",
            "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enable", "E", "Enable or disable force application", GH_ParamAccess.item, false); // 0
            pManager.AddCurveParameter("Target", "T", "Target curve to pull towards", GH_ParamAccess.item); // 1
            pManager.AddNumberParameter("MaxForce", "F", "Maximum force to apply", GH_ParamAccess.item, 1.0); // 2
            pManager.AddNumberParameter("MaxDistance", "D", "Distance at which force becomes constant (or starts to falloff)", GH_ParamAccess.item, 1.0); // 3
            pManager.AddNumberParameter("FalloffDistance", "FO", "Distance beyond MaxDistance over which pull force fades to zero. If 0 or negative, original behavior (constant force beyond MaxDistance) applies.", GH_ParamAccess.item, 0.0); // 4 (New)
            pManager.AddBooleanParameter("Fade", "Fd", "Fade force from start (0) to end (full) of curve", GH_ParamAccess.item, false); // 5
            pManager.AddBooleanParameter("PullAlong", "P", "Enable pulling along curve (using one of the methods below)", GH_ParamAccess.item, false); // 6
            pManager.AddIntegerParameter("Method", "M", "Pull method (0: tangent direction, 1: traveling point)", GH_ParamAccess.item, 0); // 7
            pManager.AddNumberParameter("TangentForce", "TF", "Force multiplier for tangent (method 0) or magnitude for traveling point pull (method 1)", GH_ParamAccess.item, 1.0); // 8
            pManager.AddNumberParameter("Speed", "S", "Speed of traveling point in mm/sec (for method 1)", GH_ParamAccess.item, 10.0); // 9
            pManager.AddBooleanParameter("Reset", "R", "Reset traveling point to start of curve (for method 1)", GH_ParamAccess.item, false); // 10
            pManager.AddTransformParameter("Transform", "X", "Transform matrix for world to device space", GH_ParamAccess.item); // 11
            pManager.AddVectorParameter("TCPOffset", "O", "Optional offset vector from TCP in device coordinates", GH_ParamAccess.item, Vector3d.Zero); // 12

            pManager[4].Optional = true;  // FalloffDistance
            pManager[5].Optional = true;  // Fade
            pManager[6].Optional = true;  // PullAlong
            pManager[7].Optional = true;  // Method
            pManager[8].Optional = true;  // TangentForce
            pManager[9].Optional = true;  // Speed
            pManager[10].Optional = true; // Reset
            pManager[11].Optional = true; // Transform
            pManager[12].Optional = true; // TCPOffset
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
            double falloffDistance = 0.0; // New parameter
            bool fade = false;
            bool pullAlong = false;
            int method = 0;
            double tangentForce = 1.0;
            double speed = 10.0;
            bool reset = false;
            Transform worldToDevice = Transform.Identity;
            Vector3d tcpOffset = Vector3d.Zero;

            if (!DA.GetData(0, ref enable)) return;
            if (!DA.GetData(1, ref targetCurve)) return; // Try to get curve even if not enabled to pass null

            if (targetCurve == null && enable)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Invalid curve input while component is enabled.");
                // Call with 11 arguments
                ForceManager.SetPullToCurve(null, false, 0, 0, 0, false, 0, 0, false, 0, false);
                return;
            }
            if (targetCurve == null && !enable)
            {
                // Call with 11 arguments
                ForceManager.SetPullToCurve(null, false, 0, 0, 0, false, 0, 0, false, 0, false);
                return;
            }

            if (!DA.GetData(2, ref maxForce)) return;
            if (!DA.GetData(3, ref maxDistance)) return;
            DA.GetData(4, ref falloffDistance);
            DA.GetData(5, ref fade);
            DA.GetData(6, ref pullAlong);
            DA.GetData(7, ref method);
            DA.GetData(8, ref tangentForce);
            DA.GetData(9, ref speed);
            DA.GetData(10, ref reset);
            DA.GetData(11, ref worldToDevice);
            DA.GetData(12, ref tcpOffset);

            Curve transformedCurve = targetCurve;
            if (targetCurve != null && !worldToDevice.Equals(Transform.Identity))
            {
                Transform deviceToWorld;
                if (worldToDevice.TryGetInverse(out deviceToWorld))
                {
                    transformedCurve = targetCurve.DuplicateCurve();
                    transformedCurve.Transform(deviceToWorld);
                }
            }
            else if (targetCurve != null)
            {
                transformedCurve = targetCurve.DuplicateCurve();
            }


            var offsetVector = new DeviceManager.Vector3D(
                tcpOffset.X,
                tcpOffset.Y,
                tcpOffset.Z
            );
            ForceManager.SetTCPOffset(offsetVector);

            // Main call with 11 arguments
            ForceManager.SetPullToCurve(
                transformedCurve,
                enable,
                maxForce,
                maxDistance,
                falloffDistance,
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