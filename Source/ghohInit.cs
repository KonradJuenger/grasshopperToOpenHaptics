using Grasshopper.Kernel;
using System;

namespace ghoh
{
    public class ghohInit : GH_Component
    {
        public ghohInit() : base("ghohInit", "init", "Initializes the haptic device", "ghoh", "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Toggle", "->", "Toggle to initialize or deinitialize the haptic device", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Device Handle", "H", "Device handle as string", GH_ParamAccess.item);
            pManager.AddTextParameter("Error", "Err", "Error message", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool toggle = false;

            if (!DA.GetData(0, ref toggle)) return;

            Logger.Log("ghohInit - SolveInstance called");
            Logger.Log($"ghohInit - Toggle value: {toggle}");

            if (!toggle)
            {
                Logger.Log("ghohInit - Deinitializing device");
                DeviceManager.Deinitialize();
                DA.SetData(0, "Device deinitialized.");
                DA.SetData(1, null);
                return;
            }

            string errorMessage;
            if (!DeviceManager.Initialize(out errorMessage))
            {
                DA.SetData(0, null);
                DA.SetData(1, errorMessage);
                Logger.Log($"ghohInit - Initialization failed: {errorMessage}");
                return;
            }

            // Get the device handle as a string
            string deviceHandleStr = DeviceManager.DeviceHandle.ToString();
            DA.SetData(0, deviceHandleStr);
            DA.SetData(1, null);

            Logger.Log("ghohInit - Device initialized successfully");
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c69a");
    }
}