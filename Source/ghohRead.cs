using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;

namespace ghoh
{
    public class ghohRead : GH_Component
    {
        public ghohRead() : base("ghohRead", "Read",
            "Reads data from the haptic device. Includes different interaction modes and special outputs.",
            "ghoh", "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddIntegerParameter("Mode", "M", "Interaction mode:\n0 = Standard (direct control)\n1 = Lazy String", GH_ParamAccess.item, 0);
            pManager.AddTransformParameter("Transform", "X", "Optional transform matrix for scaling and additional transformations", GH_ParamAccess.item);
            pManager.AddVectorParameter("TCPOffset", "O", "Optional offset vector from TCP in local device coordinates", GH_ParamAccess.item, Vector3d.Zero);

            // Inputs for Lazy String Mode
            pManager.AddNumberParameter("StringLength", "L", "[Lazy String Mode] The radius of the slack zone (mm).", GH_ParamAccess.item, 20.0);
            pManager.AddNumberParameter("ReturnForce", "F", "[Lazy String Mode] The force feedback (N) when the string is taut.", GH_ParamAccess.item, 1.5);

            pManager[1].Optional = true;
            pManager[2].Optional = true;
            pManager[3].Optional = true;
            pManager[4].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Real Cursor", "P_Real", "The real-time plane of the physical haptic device.", GH_ParamAccess.item);
            pManager.AddPlaneParameter("Virtual Cursor", "P_Virt", "The plane of the virtual cursor, used for interactions in non-standard modes.", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Button 1 Status", "B1", "Output Button 1 Status", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Button 2 Status", "B2", "Output Button 2 Status", GH_ParamAccess.item);
            pManager.AddPointParameter("DogPosition", "DogPos", "The current 3D position of the 'dog' if the DogOnLeash mode is active.", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            int handle = DeviceManager.DeviceHandle;
            if (handle == HDdll.HD_INVALID_HANDLE)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Device not initialized.");
                return;
            }

            // --- Read all inputs ---
            int mode = 0;
            Transform additionalTransform = Transform.Identity;
            Vector3d tcpOffset = Vector3d.Zero;
            double stringLength = 20.0;
            double returnForce = 1.5;

            DA.GetData(0, ref mode);
            DA.GetData(1, ref additionalTransform);
            DA.GetData(2, ref tcpOffset);
            DA.GetData(3, ref stringLength);
            DA.GetData(4, ref returnForce);

            // --- Set the interaction mode in the ForceManager ---
            if (mode == 1) // Lazy String Mode
            {
                ForceManager.SetLazyStringEffect(true, stringLength, returnForce);
            }
            else // Standard Mode (or any other future mode that is disabled by default)
            {
                ForceManager.SetLazyStringEffect(false, 0, 0);
            }

            // --- Process and output data ---

            // Get the raw state of the physical device
            var state = DeviceManager.GetCurrentState();
            if (state.Transform == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Could not retrieve device state.");
                return;
            }

            // --- Calculate the REAL cursor plane ---
            Plane realPlane = CalculatePlaneFromTransform(state.Transform, tcpOffset, additionalTransform);

            // --- Calculate the VIRTUAL cursor plane ---
            Plane virtualPlane;
            if (mode == 1)
            {
                var virtualPosNative = ForceManager.TooltipPosition_Native;
                var tempTransform = (double[])state.Transform.Clone();
                tempTransform[12] = virtualPosNative.X;
                tempTransform[13] = virtualPosNative.Y;
                tempTransform[14] = virtualPosNative.Z;
                virtualPlane = CalculatePlaneFromTransform(tempTransform, Vector3d.Zero, additionalTransform);
            }
            else
            {
                virtualPlane = realPlane;
            }

            // --- Get Dog Position ---
            Point3d dogPos_world = Point3d.Unset;
            Point3d dogPos_device = ForceManager.DogWorldPosition_RhinoCoords;
            if (dogPos_device.IsValid)
            {
                dogPos_world = dogPos_device;
                dogPos_world.Transform(additionalTransform);
            }

            // Get button status
            bool button1Status = (state.Buttons & 0x01) != 0;
            bool button2Status = (state.Buttons & 0x02) != 0;

            // Set all outputs
            DA.SetData(0, realPlane);
            DA.SetData(1, virtualPlane);
            DA.SetData(2, button1Status);
            DA.SetData(3, button2Status);
            DA.SetData(4, dogPos_world);

            // IMPORTANT: Return the array from GetCurrentState to the pool
            state.ReturnArrays();
        }

        private Plane CalculatePlaneFromTransform(double[] transform, Vector3d tcpOffset, Transform additionalTransform)
        {
            var origin = new Point3d(-transform[12], transform[14], transform[13]);
            var xDirection = new Vector3d(-transform[0], transform[2], transform[1]);
            var yDirection = new Vector3d(-transform[4], transform[6], transform[5]);
            var plane = new Plane(origin, xDirection, yDirection);

            if (!tcpOffset.IsZero)
            {
                Vector3d offsetInWorldSpace =
                    tcpOffset.X * plane.XAxis +
                    tcpOffset.Y * plane.YAxis +
                    tcpOffset.Z * plane.ZAxis;
                plane.Origin += offsetInWorldSpace;
            }

            if (!additionalTransform.Equals(Transform.Identity))
            {
                plane.Transform(additionalTransform);
            }

            return plane;
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c69b"); // Keep GUID for replacement
    }
}
