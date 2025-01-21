using Grasshopper.Kernel;
using Rhino.Geometry;
using System;

namespace ghoh
{
    public class ghohRead : GH_Component
    {
        public ghohRead() : base("ghohRead", "read", "Reads data from the haptic device", "ghoh", "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddTransformParameter("Transform", "X", "Optional transform matrix for scaling and additional transformations", GH_ParamAccess.item);
            pManager[0].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Plane", "P", "Output Plane", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Button 1 Status", "B1", "Output Button 1 Status", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Button 2 Status", "B2", "Output Button 2 Status", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Transform additionalTransform = Transform.Identity;
            DA.GetData(0, ref additionalTransform);

            int handle = DeviceManager.DeviceHandle;
            if (handle == HDdll.HD_INVALID_HANDLE)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Device not initialized.");
                Logger.Log("Device not initialized in ghohRead component.");
                return;
            }

            // Get cached state instead of querying device directly
            DeviceManager.DeviceState state = DeviceManager.GetCurrentState();
            try
            {
                // Remap axes according to the required transformation:
                // X -> Z, Y -> X, Z -> Y
                var origin = new Point3d(
                    -state.Transform[12],    // -X (negative to match coordinate system)
                    state.Transform[14],     // Z becomes Y
                    state.Transform[13]      // Y becomes Z
                );

                // Similarly transform the orientation vectors
                var xDirection = new Vector3d(
                    -state.Transform[0],     // -X
                    state.Transform[2],      // Z
                    state.Transform[1]       // Y
                );

                var yDirection = new Vector3d(
                    -state.Transform[4],     // -X
                    state.Transform[6],      // Z
                    state.Transform[5]       // Y
                );

                // Create plane from transformed vectors
                var plane = new Plane(origin, xDirection, yDirection);

                // Apply any additional transformation if provided
                if (!additionalTransform.Equals(Transform.Identity))
                {
                    plane.Transform(additionalTransform);
                }

                bool button1Status = (state.Buttons & 0x01) != 0;
                bool button2Status = (state.Buttons & 0x02) != 0;

                DA.SetData(0, plane);
                DA.SetData(1, button1Status);
                DA.SetData(2, button2Status);

                Logger.Log($"ghohRead component output - Plane: Origin ({origin}), XDir ({xDirection}), YDir ({yDirection}), Button1: {button1Status}, Button2: {button2Status}");
            }
            finally
            {
                state.ReturnArrays(); // Return arrays to pool
            }
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c69b");
    }
}