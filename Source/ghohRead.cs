using Grasshopper.Kernel;
using Rhino.Geometry;
using System;

namespace ghoh
{
    public class ghogRead : GH_Component
    {
        public ghogRead() : base("ghogRead", "read", "Reads data from the haptic device", "ghoh", "device")
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
                Logger.Log("Device not initialized in ghogRead component.");
                return;
            }

            double[] transform = new double[16];
            DevicePositionCallback(transform);

            var origin = new Point3d(-transform[12], transform[14] + 88.11, transform[13] + 65.51);
            var yDirection = new Vector3d(-transform[0], transform[2], transform[1]);
            var xDirection = new Vector3d(-transform[4], transform[6], transform[5]);
            var plane = new Plane(origin, xDirection, yDirection);

            DA.SetData(0, plane);

            double[] buttonStatus = new double[1];
            DeviceButtonCallback(buttonStatus);

            bool button1Status = (((int)buttonStatus[0]) & 0x01) != 0;
            bool button2Status = (((int)buttonStatus[0]) & 0x02) != 0;
            DA.SetData(1, button1Status);
            DA.SetData(2, button2Status);

            Logger.Log($"ghogRead component output Plane: Origin ({origin}), XDir ({xDirection}), YDir ({yDirection}), Button1: {button1Status}, Button2: {button2Status}");
        }

        static void DevicePositionCallback(double[] transform)
        {
            int deviceHandle = DeviceManager.DeviceHandle;
            if (deviceHandle == HDdll.HD_INVALID_HANDLE)
                return;

            HDdll.hdScheduleSynchronous((_) =>
            {
                HDdll.hdBeginFrame(deviceHandle);
                HDdll.hdGetDoublev(HDdll.HD_CURRENT_TRANSFORM, transform);
                HDdll.hdEndFrame(deviceHandle);
                return HDdll.HD_CALLBACK_DONE;
            }, IntPtr.Zero, HDdll.HD_DEFAULT_SCHEDULER_PRIORITY);
        }

        static void DeviceButtonCallback(double[] buttonStatus)
        {
            int deviceHandle = DeviceManager.DeviceHandle;
            if (deviceHandle == HDdll.HD_INVALID_HANDLE)
                return;

            HDdll.hdScheduleSynchronous((_) =>
            {
                HDdll.hdBeginFrame(deviceHandle);
                HDdll.hdGetDoublev(HDdll.HD_CURRENT_BUTTONS, buttonStatus);
                HDdll.hdEndFrame(deviceHandle);
                return HDdll.HD_CALLBACK_DONE;
            }, IntPtr.Zero, HDdll.HD_DEFAULT_SCHEDULER_PRIORITY);
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c69b");
    }
}
