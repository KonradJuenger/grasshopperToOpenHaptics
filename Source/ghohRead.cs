using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace ghoh
{
    public class ghohRead : GH_Component
    {
        public ghohRead() : base("ghohRead", "read", "Reads data from the haptic device with optional TCP offset", "ghoh", "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddTransformParameter("Transform", "X", "Optional transform matrix for scaling and additional transformations", GH_ParamAccess.item);
            pManager.AddVectorParameter("TCPOffset", "O", "Optional offset vector from TCP in local device coordinates", GH_ParamAccess.item, Vector3d.Zero);
            pManager[0].Optional = true;
            pManager[1].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Plane", "P", "Output Plane", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Button 1 Status", "B1", "Output Button 1 Status", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Button 2 Status", "B2", "Output Button 2 Status", GH_ParamAccess.item);
           // pManager.AddTransformParameter("Raw Transform", "RT", "Raw 4×4 transformation matrix", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Transform additionalTransform = Transform.Identity;
            Vector3d tcpOffset = Vector3d.Zero;
            DA.GetData(0, ref additionalTransform);
            DA.GetData(1, ref tcpOffset);

            int handle = DeviceManager.DeviceHandle;
            if (handle == HDdll.HD_INVALID_HANDLE)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Device not initialized.");
                return;
            }

            
            var state = DeviceManager.GetCurrentState();
            /*
            var rawTransform = Transform.Identity;
            rawTransform.M00 = state.Transform[0];
            rawTransform.M01 = state.Transform[1];
            rawTransform.M02 = state.Transform[2];
            rawTransform.M03 = state.Transform[3];
            rawTransform.M10 = state.Transform[4];
            rawTransform.M11 = state.Transform[5];
            rawTransform.M12 = state.Transform[6];
            rawTransform.M13 = state.Transform[7];
            rawTransform.M20 = state.Transform[8];
            rawTransform.M21 = state.Transform[9];
            rawTransform.M22 = state.Transform[10];
            rawTransform.M23 = state.Transform[11];
            rawTransform.M30 = state.Transform[12];
            rawTransform.M31 = state.Transform[13];
            rawTransform.M32 = state.Transform[14];
            rawTransform.M33 = state.Transform[15];


            // Set the output
            DA.SetData(3, rawTransform);
            */
            // Create initial plane from cached state
            var origin = new Point3d(
                -state.Transform[12],
                state.Transform[14],
                state.Transform[13]
            );

            var xDirection = new Vector3d(
                -state.Transform[0],
                state.Transform[2],
                state.Transform[1]
            );

            var yDirection = new Vector3d(
                -state.Transform[4],
                state.Transform[6],
                state.Transform[5]
            );

            var plane = new Plane(origin, xDirection, yDirection);

            // Apply TCP offset if provided
            if (!tcpOffset.IsZero)
            {
                // Get the plane's coordinate system vectors
                Vector3d zDirection = Vector3d.CrossProduct(xDirection, yDirection);

                // Create the offset in the plane's coordinate system
                Vector3d offsetInPlaneSpace =
                    tcpOffset.X * xDirection +
                    tcpOffset.Y * yDirection +
                    tcpOffset.Z * zDirection;

                // Move the plane's origin
                plane.Origin += offsetInPlaneSpace;
            }

            // Apply additional transform if provided
            if (!additionalTransform.Equals(Transform.Identity))
            {
                plane.Transform(additionalTransform);
            }

            bool button1Status = (state.Buttons & 0x01) != 0;
            bool button2Status = (state.Buttons & 0x02) != 0;

            DA.SetData(0, plane);
            DA.SetData(1, button1Status);
            DA.SetData(2, button2Status);

            state.ReturnArrays();
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c69b");
    }
}