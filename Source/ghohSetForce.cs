using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using MathNet.Numerics.LinearAlgebra;

namespace ghoh
{
    public class ghohSetForce : GH_Component
    {
        // UKF instance and state
        private static UKF forceFilter;
        private static Vector3d lastRawForce = Vector3d.Zero;
        private static bool filterEnabled = true;
        private static bool forceEnabled = false;
        private const double MAX_FORCE = 10.0;  // Maximum force in Newtons

        public ghohSetForce() : base(
            "ghohSetForce",
            "SetForce",
            "Sets force to the haptic device with Kalman filtering",
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
            pManager.AddVectorParameter("FilteredForce", "FF", "Current filtered force vector", GH_ParamAccess.item);
            pManager.AddNumberParameter("FilterQuality", "FQ", "Filter quality metric (0-1)", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool enable = false;
            Vector3d force = new Vector3d();
            Transform transform = Transform.Identity;
            double q = 0.05, r = 0.3;

            if (!DA.GetData(0, ref enable)) return;
            if (!DA.GetData(1, ref force)) return;
            DA.GetData(2, ref transform);
            DA.GetData(3, ref filterEnabled);
            DA.GetData(4, ref q);
            DA.GetData(5, ref r);

            // Update force enabled state
            forceEnabled = enable;

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
            //force = ClampForce(force);

            // Store raw force for servo loop updates regardless of filter state
            lastRawForce = force;

            // Initialize or update UKF
            if (forceFilter == null)
            {
                forceFilter = new UKF(3);
                forceFilter.SetNoiseParams(q, r);
            }
            else if (enable && filterEnabled)
            {
                forceFilter.SetNoiseParams(q, r);
            }

            // Update filter with new measurement if enabled
            if (enable && filterEnabled)
            {
                var measurement = new[] { force.X, force.Y, force.Z };
                forceFilter.Update(measurement);

                // Get filtered force for output
                var filteredState = forceFilter.getState();
                var filteredForce = new Vector3d(filteredState[0], filteredState[1], filteredState[2]);

                // Calculate filter quality metric (based on covariance trace)
                var covariance = forceFilter.getCovariance();
                double quality = CalculateFilterQuality(covariance);

                // Set outputs
                DA.SetData(0, filteredForce);
                DA.SetData(1, quality);
            }
            else
            {
                // If filter disabled, output raw force
                DA.SetData(0, force);
                DA.SetData(1, 1.0); // Perfect quality when not filtering

                if (forceFilter != null)
                {
                    forceFilter.Reset();
                }
            }
        }

        public static void UpdateServoForces()
        {
            if (!forceEnabled) return;  // Return if forces are disabled entirely

            try
            {
                double[] forceArray;

                if (filterEnabled && forceFilter != null)
                {
                    // Get filtered force prediction
                    forceFilter.Predict();
                    var filteredForce = forceFilter.getState();

                    forceArray = new double[3]
                    {
                -filteredForce[0],  // Negate X for device space
                filteredForce[2],   // Y becomes Z
                filteredForce[1]    // Z becomes Y
                    };
                }
                else
                {
                    // Use last raw force directly
                    forceArray = new double[3]
                    {
                -lastRawForce.X,
                lastRawForce.Z,
                lastRawForce.Y
                    };
                }

                ForceManager.SetDirectForce(forceArray, true);
                ForceManager.UpdateForces(); // Add this line to apply the forces
            }
            catch (Exception ex)
            {
                Logger.Log($"Error in UpdateServoForces: {ex.Message}");
            }
        }

        private Vector3d ClampForce(Vector3d force)
        {
            double magnitude = force.Length;
            if (magnitude > MAX_FORCE)
            {
                force *= MAX_FORCE / magnitude;
            }
            return force;
        }

        private double CalculateFilterQuality(double[,] covariance)
        {
            // Calculate normalized trace of covariance matrix
            double trace = 0;
            for (int i = 0; i < 3; i++)
            {
                trace += covariance[i, i];
            }
            // Convert to quality metric (0-1, higher is better)
            return 1.0 / (1.0 + trace);
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c69c");
    }
}