using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Runtime.InteropServices;

namespace ghoh
{
    public class ghohSetForce : GH_Component
    {
        public ghohSetForce() : base("ghohSetForce", "setForce", "Apply force to the haptic device", "ghoh", "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddVectorParameter("Force", "F", "Force vector to apply (in Newtons)", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Enable", "E", "Enable or disable force output", GH_ParamAccess.item, false);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
        }

        private bool forceEnabled = false;
        private Vector3d currentForce = Vector3d.Zero;
        private IntPtr forceCallbackHandle = IntPtr.Zero;
        private GCHandle gcHandle;

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Vector3d force = Vector3d.Zero;
            bool enable = false;

            if (!DA.GetData(0, ref force))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Failed to get force input.");
                return;
            }

            if (!DA.GetData(1, ref enable))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Failed to get enable input.");
                return;
            }

            uint handle = DeviceManager.DeviceHandle;
            if (handle == HDdll.HD_BAD_HANDLE)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Device not initialized.");
                return;
            }

            forceEnabled = enable;
            currentForce = force;

            if (forceEnabled)
            {
                if (forceCallbackHandle == IntPtr.Zero)
                {
                    if (!gcHandle.IsAllocated)
                        gcHandle = GCHandle.Alloc(this);

                    unsafe
                    {
                        forceCallbackHandle = HDdll.hdScheduleAsynchronous(ForceOutputCallback,
                            (void*)GCHandle.ToIntPtr(gcHandle), HDdll.HD_DEFAULT_SCHEDULER_PRIORITY);
                    }
                }
            }
            else
            {
                if (forceCallbackHandle != IntPtr.Zero)
                {
                    HDdll.hdUnschedule(forceCallbackHandle);
                    HDdll.hdWaitForCompletion(forceCallbackHandle, HDdll.HD_WAIT_INFINITE);
                    forceCallbackHandle = IntPtr.Zero;

                    if (gcHandle.IsAllocated)
                        gcHandle.Free();
                }
            }
        }

        private unsafe static uint ForceOutputCallback(void* pData)
        {
            try
            {
                uint deviceHandle = DeviceManager.DeviceHandle;
                if (deviceHandle == HDdll.HD_BAD_HANDLE)
                    return HDdll.HD_CALLBACK_DONE;

                GCHandle handle = GCHandle.FromIntPtr((IntPtr)pData);
                if (!handle.IsAllocated)
                {
                    return HDdll.HD_CALLBACK_DONE;
                }

                ghohSetForce instance = handle.Target as ghohSetForce;

                if (instance == null || !instance.forceEnabled)
                {
                    return HDdll.HD_CALLBACK_DONE;
                }

                HDdll.hdBeginFrame(deviceHandle);

                double[] forceArray = new double[3] { instance.currentForce.X, instance.currentForce.Y, instance.currentForce.Z };
                fixed (double* forcePtr = forceArray)
                {
                    HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, forcePtr);
                }

                HDdll.hdEndFrame(deviceHandle);

                return HDdll.HD_CALLBACK_CONTINUE;
            }
            catch (Exception ex)
            {
                // Log the exception or handle it appropriately
                // For now, we simply stop the callback to prevent crashing
                return HDdll.HD_CALLBACK_DONE;
            }
        }

        public override void RemovedFromDocument(GH_Document document)
        {
            base.RemovedFromDocument(document);
            Cleanup();
        }

        private void Cleanup()
        {
            if (forceCallbackHandle != IntPtr.Zero)
            {
                HDdll.hdUnschedule(forceCallbackHandle);
                HDdll.hdWaitForCompletion(forceCallbackHandle, HDdll.HD_WAIT_INFINITE);
                forceCallbackHandle = IntPtr.Zero;
            }

            if (gcHandle.IsAllocated)
                gcHandle.Free();
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("DCA0A7F0-0C6B-4E84-8E2D-5C9A0FA2B6D1");
    }
}
