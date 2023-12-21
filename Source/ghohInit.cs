using Grasshopper.Kernel;
using System;

namespace ghoh
{
    public class ghohInit : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public ghohInit() : base("ghohInit", "init", "Description", "ghoh", "device")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Toggle", "->", "Toggle", GH_ParamAccess.item);
            
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddIntegerParameter("Device Handle", "H", "Number", GH_ParamAccess.item);
            pManager.AddTextParameter("Error", "Err", "Message", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool toggle = false;
            DA.GetData(0, ref toggle);
            if (!toggle) {
                DA.SetData(0, HDdll.HD_BAD_HANDLE);
                return;
            }

            // Allowing re-inits is convenient, but it can get confusing.
            // TODO: Should we keep this or rather bail out?
            if (DeviceHandle != HDdll.HD_BAD_HANDLE) {
                HDdll.hdStopScheduler();
                HDdll.hdDisableDevice(DeviceHandle);
            }

            DeviceHandle = HDdll.hdInitDevice(HDdll.HD_DEFAULT_DEVICE);

            HDdll.ErrorInfo Err = HDdll.hdGetError();
            if (Err.ErrorCode != HDdll.HD_SUCCESS) {
                DA.SetData(1, $"0x{Err.ErrorCode:X}");

                // FIXME: Rhino crashes with 0xC0000374 (STATUS_HEAP_CORRUPTION)
                // string Msg = HDdll.hdGetErrorString(Err.ErrorCode);
                // RhinoApp.WriteLine("Error: {0}", Msg);
                return;
            }

            HDdll.hdStartScheduler();

            DA.SetData(0, DeviceHandle);

            // TODO: We will need a deinit module with
            //   hdStopScheduler();
            //   hdDisableDevice(DeviceHandle)
        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// You can add image files to your project resources and access them like this:
        /// return Resources.IconForThisComponent;
        /// </summary>
        protected override System.Drawing.Bitmap Icon => null;

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c69a");

        private uint DeviceHandle = HDdll.HD_BAD_HANDLE;
    }
}