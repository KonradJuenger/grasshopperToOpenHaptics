using Grasshopper.Kernel;
using System;

namespace ghoh
{
    public class ghogRead : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public ghogRead() : base("ghogRead", "read", "Description", "ghoh", "device")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddIntegerParameter("Device Handle", "H", "Number", GH_ParamAccess.item, (int)HDdll.HD_BAD_HANDLE);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Position X", "X", "Float", GH_ParamAccess.item);
            pManager.AddNumberParameter("Position Y", "Y", "Float", GH_ParamAccess.item);
            pManager.AddNumberParameter("Position Z", "Z", "Float", GH_ParamAccess.item);
        }

        // FIXME: This is a bad hack! It break when two modules run
        //        SolveInstance() for different devices!
        private static System.UInt32 CurrentDevice = HDdll.HD_BAD_HANDLE;

        /// <summary>
        /// This is the method will be called from the scheduler thread.
        /// The main thread is blocked in that time.
        /// </summary>
        unsafe static System.UInt32 DevicePositionCallback(void* pData)
        {
            if (CurrentDevice == HDdll.HD_BAD_HANDLE)
                return HDdll.HD_CALLBACK_DONE;

            double* pPosition = (double*) pData;
            HDdll.hdBeginFrame(HDdll.hdGetCurrentDevice());
            HDdll.hdGetDoublev(HDdll.HD_CURRENT_POSITION, pPosition);
            HDdll.hdEndFrame(HDdll.hdGetCurrentDevice());

            return HDdll.HD_CALLBACK_DONE;
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            int handle = (int)HDdll.HD_BAD_HANDLE;
            if (!DA.GetData(0, ref handle))
                return;
            if (handle == HDdll.HD_BAD_HANDLE)
                return;
            CurrentDevice = (uint)handle;

            unsafe
            {
                // Create a pointer to heap memory within a fixed block:
                // It prevents the Garbage Collector from deleting or moving the
                // respective heap item.
                fixed (double* position = new double[3])
                {
                    // Schedule the callback in the next available slot.
                    // Synchronous means that we wait for it to complete.
                    HDdll.hdScheduleSynchronous(DevicePositionCallback, (void*)position,
                        HDdll.HD_DEFAULT_SCHEDULER_PRIORITY);

                    // The position memory block is still pinned. It should
                    // contain the desired position values now.
                    for (var i = 0; i < 3; i += 1)
                    {
                        DA.SetData(i, position[i]);
                    }
                }
            }

            CurrentDevice = HDdll.HD_BAD_HANDLE;
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
        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c69b");
    }
}