using Grasshopper.Kernel;
using Rhino.Geometry;
using System;

namespace ghoh
{
    public class ghohSetViscous : GH_Component
    {
        private const double MAX_FORCE = 10.0;  // Maximum force in Newtons

        public ghohSetViscous() : base(
            "ghohSetViscous",
            "SetViscous",
            "Applies viscous damping force to the haptic device",
            "ghoh",
            "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enable", "E", "Enable or disable viscous damping", GH_ParamAccess.item, false);
            pManager.AddNumberParameter("Gain", "G", "Viscosity coefficient (0.0-1.0)", GH_ParamAccess.item, 0.2);
            pManager.AddNumberParameter("MaxForce", "F", "Maximum force magnitude in Newtons", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("Deadband", "D", "Velocity threshold below which forces begin to reduce", GH_ParamAccess.item, 10.0);
            pManager.AddNumberParameter("Softness", "S", "Width of transition zone for deadband (0 = hard cutoff)", GH_ParamAccess.item, 5.0);
            pManager.AddNumberParameter("VelocityFilter", "VF", "Velocity filter coefficient (0.0-1.0, higher = more filtering)", GH_ParamAccess.item, 0.9);
            pManager.AddIntegerParameter("WindowSize", "WS", "Number of samples for force moving average", GH_ParamAccess.item, 10);

            // Make most parameters optional
            pManager[3].Optional = true;
            pManager[4].Optional = true;
            pManager[5].Optional = true;
            pManager[6].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("AppliedForce", "AF", "Actually applied viscous force vector", GH_ParamAccess.item);
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
            double gain = 0.2;
            double maxForce = 1.0;
            double deadband = 10.0;
            double softness = 5.0;
            double velocityFilter = 0.9;
            int windowSize = 10;

            if (!DA.GetData(0, ref enable)) return;
            if (!DA.GetData(1, ref gain)) return;
            if (!DA.GetData(2, ref maxForce)) return;
            DA.GetData(3, ref deadband);
            DA.GetData(4, ref softness);
            DA.GetData(5, ref velocityFilter);
            DA.GetData(6, ref windowSize);

            // Clamp values for safety
            gain = Math.Max(0.0, Math.Min(1.0, gain));
            maxForce = Math.Max(0.0, Math.Min(MAX_FORCE, maxForce));
            deadband = Math.Max(0.0, Math.Min(100.0, deadband));
            softness = Math.Max(0.0, Math.Min(deadband, softness)); // Softness can't exceed deadband
            velocityFilter = Math.Max(0.0, Math.Min(0.99, velocityFilter));
            windowSize = Math.Max(1, Math.Min(50, windowSize));

            // Apply viscous damping with all parameters
            Vector3d appliedForce = ForceManager.SetViscousDamping(
                enable, gain, maxForce, deadband, softness, velocityFilter, windowSize);

            // Output the force being applied
            DA.SetData(0, appliedForce);
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("a8c53e2f-b6e1-42c1-9e15-f2d76a9c5d8c");
    }
}