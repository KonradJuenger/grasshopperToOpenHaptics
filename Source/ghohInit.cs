using Grasshopper.Kernel;
using System;

namespace ghoh
{
    public class ghohInit : GH_Component
    {
        public ghohInit() : base("ghohInit", "init", "Description", "ghoh", "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Toggle", "->", "Toggle", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddIntegerParameter("Device Handle", "H", "Number", GH_ParamAccess.item);
            pManager.AddTextParameter("Error", "Err", "Message", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool toggle = false;
            DA.GetData(0, ref toggle);

            if (!toggle)
            {
                DeviceManager.Deinitialize();
                DA.SetData(0, HDdll.HD_BAD_HANDLE);
                return;
            }

            string errorMessage;
            if (!DeviceManager.Initialize(out errorMessage))
            {
                DA.SetData(1, errorMessage);
                return;
            }

            DA.SetData(0, DeviceManager.DeviceHandle);
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c69a");
    }
}
