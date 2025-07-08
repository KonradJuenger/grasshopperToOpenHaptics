using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace ghoh
{
    public class ghohDogOnLeash : GH_Component
    {
        public ghohDogOnLeash()
          : base("ghohDogOnLeash", "DogLeash",
              "Configures the 'Dog on a Leash' haptic effect. The live position of the dog is output by the ghohRead component.",
              "ghoh", "device")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enable", "E", "Enable or disable the 'Dog on a Leash' force effect.", GH_ParamAccess.item, false);
            pManager.AddCurveParameter("Curve", "C", "The path the 'dog' will follow.", GH_ParamAccess.item);
            pManager.AddNumberParameter("LeashLength", "L", "The distance at which the leash starts to pull (mm).", GH_ParamAccess.item, 20.0);
            pManager.AddNumberParameter("MaxDistance", "D", "The distance at which the pull force reaches its maximum (mm).", GH_ParamAccess.item, 40.0);
            pManager.AddNumberParameter("MaxForce", "F", "The maximum force applied by the leash (N).", GH_ParamAccess.item, 1.5);
            pManager.AddNumberParameter("DogSpeed", "S", "The speed of the 'dog' on the curve (mm/sec).", GH_ParamAccess.item, 30.0);
            pManager.AddTransformParameter("Transform", "X", "Optional transform matrix for world to device space.", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Reset", "R", "A toggle or button to reset the 'dog' to the start of the curve.", GH_ParamAccess.item, false);

            pManager[6].Optional = true; // Transform
            pManager[7].Optional = true; // Reset
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            // Outputs have been moved to the ghohRead component for live updates.
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            if (DeviceManager.DeviceHandle == HDdll.HD_INVALID_HANDLE)
            {
                return;
            }

            bool enable = false;
            Curve targetCurve = null;
            double leashLength = 20.0;
            double maxDistance = 40.0;
            double maxForce = 1.5;
            double dogSpeed = 30.0;
            Transform worldToDevice = Transform.Identity;
            bool reset = false;

            DA.GetData(0, ref enable);
            DA.GetData(1, ref targetCurve);
            DA.GetData(2, ref leashLength);
            DA.GetData(3, ref maxDistance);
            DA.GetData(4, ref maxForce);
            DA.GetData(5, ref dogSpeed);
            DA.GetData(6, ref worldToDevice);
            DA.GetData(7, ref reset);

            if (!enable)
            {
                ForceManager.SetDogOnLeash(false, null, 0, 0, 0, 0, true);
                return;
            }

            if (targetCurve == null || !targetCurve.IsValid)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "A valid curve is required when the component is enabled.");
                ForceManager.SetDogOnLeash(false, null, 0, 0, 0, 0, true);
                return;
            }

            Curve transformedCurve = targetCurve.DuplicateCurve();
            if (!worldToDevice.Equals(Transform.Identity))
            {
                if (worldToDevice.TryGetInverse(out Transform deviceToWorld))
                {
                    transformedCurve.Transform(deviceToWorld);
                }
                else
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "The provided transform is not invertible.");
                    return;
                }
            }

            ForceManager.SetDogOnLeash(enable, transformedCurve, leashLength, maxDistance, maxForce, dogSpeed, reset);
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("1a8a2b7e-3c4d-4e5f-8a6b-9f0c1d2e3f4a");
    }
}
