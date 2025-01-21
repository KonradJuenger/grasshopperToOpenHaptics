using Grasshopper.Kernel;
using Rhino.Geometry;
using System;

namespace ghoh
{
    public class ghohSetForce : GH_Component
    {
        public ghohSetForce() : base("ghohSetForce", "SetForce", "Sets force to the haptic device", "ghoh", "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enable", "E", "Enable or disable force application", GH_ParamAccess.item, false);
            pManager.AddVectorParameter("Force", "F", "Force vector to apply to the device", GH_ParamAccess.item, new Vector3d(0, 0, 0));
            pManager.AddTransformParameter("Transform", "X", "Optional transform matrix for scaling and additional transformations", GH_ParamAccess.item);
            pManager[2].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            // No output parameters needed
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Logger.Log("ghohSetForce - SolveInstance started");

            bool enable = false;
            Vector3d force = new Vector3d();
            Transform additionalTransform = Transform.Identity;

            if (!DA.GetData(0, ref enable)) return;
            if (!DA.GetData(1, ref force)) return;
            DA.GetData(2, ref additionalTransform);

            Logger.Log($"ghohSetForce - Retrieved force: {force.X},{force.Y},{force.Z}");
            Logger.Log($"ghohSetForce - Retrieved enable: {enable}");

            int handle = DeviceManager.DeviceHandle;
            Logger.Log($"ghohSetForce - Device handle: {handle}");

            if (handle == HDdll.HD_INVALID_HANDLE)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Device not initialized.");
                Logger.Log("ghohSetForce - Device not initialized.");
                return;
            }

            // Apply any additional transformation if provided
            if (!additionalTransform.Equals(Transform.Identity))
            {
                Transform inverseTransform = additionalTransform;
                inverseTransform.TryGetInverse(out inverseTransform);
                force.Transform(inverseTransform);
            }

            // Remap the force vector components according to the required transformation:
            // Rhino -> Device
            // X -> -X (negative to match coordinate system)
            // Y -> Z
            // Z -> Y
            double[] forceArray = new double[3]
            {
                -force.X,    // -X
                force.Z,     // Y
                force.Y      // Z
            };

            Logger.Log($"ghohSetForce - Applying force: [{forceArray[0]}, {forceArray[1]}, {forceArray[2]}], Enable: {enable}");

            // Apply the force to the device
            DeviceManager.ApplyForce(forceArray, enable);
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c69c");
    }
}