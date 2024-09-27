using Grasshopper.Kernel;
using Rhino.Geometry;
using System;

namespace ghoh
{
    public class ghogRead : GH_Component
    {
        public ghogRead() : base("ghogRead", "read", "Description", "ghoh", "device")
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
            uint handle = DeviceManager.DeviceHandle;
            if (handle == HDdll.HD_BAD_HANDLE)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Device not initialized.");
                return;
            }

            unsafe
            {
                fixed (double* transform = new double[16])
                {
                    HDdll.hdScheduleSynchronous(DevicePositionCallback, (void*)transform,
                        HDdll.HD_DEFAULT_SCHEDULER_PRIORITY);

                    var origin = new Point3d(-transform[12], transform[14] + 88.11, transform[13] + 65.51);
                    var yDirection = new Vector3d(-transform[0], transform[2], transform[1]);
                    var xDirection = new Vector3d(-transform[4], transform[6], transform[5]);
                    var plane = new Plane(origin, xDirection, yDirection);

                    DA.SetData(0, plane);
                }

                fixed (double* buttonStatus = new double[1])
                {
                    HDdll.hdScheduleSynchronous(DeviceButtonCallback, (void*)buttonStatus,
                        HDdll.HD_DEFAULT_SCHEDULER_PRIORITY);

                    bool button1Status = (((int)*buttonStatus) & 0x01) != 0;
                    bool button2Status = (((int)*buttonStatus) & 0x02) != 0;
                    DA.SetData(1, button1Status);
                    DA.SetData(2, button2Status);
                }
            }
        }

        unsafe static uint DevicePositionCallback(void* pData)
        {
            uint deviceHandle = DeviceManager.DeviceHandle;
            if (deviceHandle == HDdll.HD_BAD_HANDLE)
                return HDdll.HD_CALLBACK_DONE;

            double* pTransform = (double*)pData;
            HDdll.hdBeginFrame(deviceHandle);
            HDdll.hdGetDoublev(HDdll.HD_CURRENT_TRANSFORM, pTransform);
            HDdll.hdEndFrame(deviceHandle);

            return HDdll.HD_CALLBACK_DONE;
        }

        unsafe static uint DeviceButtonCallback(void* pData)
        {
            uint deviceHandle = DeviceManager.DeviceHandle;
            if (deviceHandle == HDdll.HD_BAD_HANDLE)
                return HDdll.HD_CALLBACK_DONE;

            double* pButtonStatus = (double*)pData;
            HDdll.hdBeginFrame(deviceHandle);
            HDdll.hdGetDoublev(HDdll.HD_CURRENT_BUTTONS, pButtonStatus);
            HDdll.hdEndFrame(deviceHandle);

            return HDdll.HD_CALLBACK_DONE;
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c69b");
    }
}
