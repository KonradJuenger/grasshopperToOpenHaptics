using Grasshopper.Kernel;
using Rhino.Geometry;
using System;

namespace ghoh
{
    public class ghohSetForce : GH_Component
    {
        private const double MAX_FORCE = 10.0;  // Maximum force in Newtons

        public ghohSetForce() : base(
            "ghohSetForce",
            "SetForce",
            "Sets force to the haptic device with optional Kalman filtering",
            "ghoh",
            "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enable", "E", "Enable or disable force application", GH_ParamAccess.item, false);
            pManager.AddVectorParameter("Force", "F", "Force vector to apply to the device", GH_ParamAccess.item, new Vector3d(0, 0, 0));
            pManager.AddTransformParameter("Transform", "X", "Optional transform matrix", GH_ParamAccess.item);
            pManager.AddBooleanParameter("FilterEnabled", "Filt", "Enable Kalman filtering", GH_ParamAccess.item, true);
            pManager.AddNumberParameter("ProcessNoise", "Q", "Process noise covariance (0.001-1.0)", GH_ParamAccess.item, 0.05);
            pManager.AddNumberParameter("MeasNoise", "R", "Measurement noise covariance (0.001-1.0)", GH_ParamAccess.item, 0.3);

            pManager[2].Optional = true;  // Transform
            pManager[3].Optional = true;  // FilterEnabled
            pManager[4].Optional = true;  // ProcessNoise
            pManager[5].Optional = true;  // MeasNoise
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("AppliedForce", "AF", "Actually applied force vector", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool enable = false;
            Vector3d force = new Vector3d();
            Transform transform = Transform.Identity;
            bool useFilter = true;
            double q = 0.05, r = 0.3;

            if (!DA.GetData(0, ref enable)) return;
            if (!DA.GetData(1, ref force)) return;
            DA.GetData(2, ref transform);
            DA.GetData(3, ref useFilter);
            DA.GetData(4, ref q);
            DA.GetData(5, ref r);

            // Clamp noise parameters to valid range
            q = Math.Max(0.001, Math.Min(q, 1.0));
            r = Math.Max(0.001, Math.Min(r, 1.0));

            // Apply transform if provided
            if (!transform.Equals(Transform.Identity))
            {
                Transform inverseTransform = transform;
                if (inverseTransform.TryGetInverse(out inverseTransform))
                {
                    force.Transform(inverseTransform);
                }
            }

            // Clamp force magnitude for safety
            double magnitude = force.Length;
            if (magnitude > MAX_FORCE)
            {
                force *= MAX_FORCE / magnitude;
            }

            // Update filter parameters if filtering is enabled
            if (useFilter)
            {
                ForceManager.SetFilterParams(q, r);
            }

            // Convert to device coordinates and apply force
            double[] forceArray = new double[3]
            {
                -force.X,    // Negate X for device space
                force.Z,     // Y becomes Z
                force.Y      // Z becomes Y
            };

            ForceManager.SetDirectForce(forceArray, enable, useFilter);

            // Output the force being applied
            DA.SetData(0, force);
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c69c");
    }
}