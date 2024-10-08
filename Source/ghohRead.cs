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
            // No input parameters needed
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Plane", "P", "Output Plane", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Button 1 Status", "B1", "Output Button 1 Status", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Button 2 Status", "B2", "Output Button 2 Status", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            int handle = DeviceManager.DeviceHandle;
            if (handle == HDdll.HD_INVALID_HANDLE)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Device not initialized.");
                Logger.Log("Device not initialized in ghohRead component.");
                return;
            }

            DeviceManager.DeviceState state = DeviceManager.GetDeviceState();

            var origin = new Point3d(-state.Transform[12], state.Transform[14] + 88.11, state.Transform[13] + 65.51);
            var yDirection = new Vector3d(-state.Transform[0], state.Transform[2], state.Transform[1]);
            var xDirection = new Vector3d(-state.Transform[4], state.Transform[6], state.Transform[5]);
            var plane = new Plane(origin, xDirection, yDirection);

            bool button1Status = (state.Buttons & 0x01) != 0;
            bool button2Status = (state.Buttons & 0x02) != 0;

            DA.SetData(0, plane);
            DA.SetData(1, button1Status);
            DA.SetData(2, button2Status);

            Logger.Log($"ghohRead component output Plane: Origin ({origin}), XDir ({xDirection}), YDir ({yDirection}), Button1: {button1Status}, Button2: {button2Status}");
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c69b");
    }
}