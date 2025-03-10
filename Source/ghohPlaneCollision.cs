using Grasshopper.Kernel;
using Rhino.Geometry;
using System;

namespace ghoh
{
    public class ghohPlaneCollision : GH_Component
    {
        public ghohPlaneCollision() : base(
            "ghohPlaneCollision",
            "PlaneCollision",
            "Creates a collision boundary with a plane that restricts movement through the plane but allows free movement away from it",
            "ghoh",
            "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enable", "E", "Enable or disable collision force", GH_ParamAccess.item, false);
            pManager.AddPlaneParameter("Boundary", "B", "Boundary plane for collision", GH_ParamAccess.item);
            pManager.AddNumberParameter("MaxForce", "F", "Maximum force to apply when collision occurs", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("MaxDistance", "D", "Distance of penetration at which force becomes constant", GH_ParamAccess.item, 1.0);
            pManager.AddTransformParameter("Transform", "X", "Transform matrix for world to device space", GH_ParamAccess.item);
            pManager.AddVectorParameter("TCPOffset", "O", "Optional offset vector from TCP in device coordinates", GH_ParamAccess.item, Vector3d.Zero);

            pManager[4].Optional = true;  // Transform
            pManager[5].Optional = true;  // TCPOffset
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Penetration", "P", "Current penetration distance (negative when penetrating)", GH_ParamAccess.item);
            pManager.AddVectorParameter("Force", "F", "Current applied force vector", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var handle = DeviceManager.DeviceHandle;
            if (handle == HDdll.HD_INVALID_HANDLE)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Device not initialized");
                return;
            }

            bool enable = false;
            Plane boundaryPlane = Plane.WorldXY;
            double maxForce = 1.0;
            double maxDistance = 1.0;
            Transform worldToDevice = Transform.Identity;
            Vector3d tcpOffset = Vector3d.Zero;

            if (!DA.GetData(0, ref enable)) return;
            if (!DA.GetData(1, ref boundaryPlane)) return;
            if (!DA.GetData(2, ref maxForce)) return;
            if (!DA.GetData(3, ref maxDistance)) return;
            DA.GetData(4, ref worldToDevice);
            DA.GetData(5, ref tcpOffset);

            // Transform boundary plane to device space if transform provided
            if (!worldToDevice.Equals(Transform.Identity))
            {
                Transform deviceToWorld;
                if (worldToDevice.TryGetInverse(out deviceToWorld))
                {
                    boundaryPlane.Transform(deviceToWorld);
                }
            }

            // Ensure the plane normal is unitized
            boundaryPlane.Normal.Unitize();

            // Convert plane parameters to device space vectors
            var origin = new DeviceManager.Vector3D(
                boundaryPlane.Origin.X,
                boundaryPlane.Origin.Y,
                boundaryPlane.Origin.Z
            );
            var normal = new DeviceManager.Vector3D(
                boundaryPlane.Normal.X,
                boundaryPlane.Normal.Y,
                boundaryPlane.Normal.Z
            );

            // Set TCP offset first
            var offsetVector = new DeviceManager.Vector3D(
                tcpOffset.X,
                tcpOffset.Y,
                tcpOffset.Z
            );
            ForceManager.SetTCPOffset(offsetVector);

            // Call the ForceManager method for plane collision
            ForceManager.SetPlaneCollision(
                origin,
                normal,
                enable,
                maxForce,
                maxDistance
            );

            // Get current device state for output calculations
            var state = DeviceManager.GetCurrentState();
            try
            {
                // Calculate current device position
                var devicePos = new Point3d(
                    -state.Transform[12],
                    state.Transform[14],
                    state.Transform[13]
                );

                // Calculate penetration depth
                Vector3d toDevice = devicePos - boundaryPlane.Origin;
                double penetration = Vector3d.Multiply(toDevice, boundaryPlane.Normal);
                DA.SetData(0, penetration);

                // Calculate force (for visualization)
                Vector3d forceVector = Vector3d.Zero;
                if (enable && penetration < 0)
                {
                    double absDistance = -penetration;
                    double forceMagnitude = absDistance >= maxDistance ?
                        maxForce : maxForce * (absDistance / maxDistance);
                    forceVector = boundaryPlane.Normal * forceMagnitude;
                }
                DA.SetData(1, forceVector);
            }
            finally
            {
                state.ReturnArrays();
            }
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001822c70a");
    }
}