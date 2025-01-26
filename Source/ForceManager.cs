using System;

namespace ghoh
{
    public static class ForceManager
    {
        // Flags for enabled forces
        private static bool directForceEnabled;
        private static bool pullToPointEnabled;
        private static bool pullToPlaneEnabled;

        // Direct force parameters
        private static double[] currentDirectForce = new double[3];

        // Pull to point parameters
        private static DeviceManager.Vector3D targetPoint;
        private static double maxForceValuePoint = 1.0;
        private static double maxDistanceValuePoint = 1.0;

        // Pull to plane parameters
        private static DeviceManager.Vector3D planeOrigin;
        private static DeviceManager.Vector3D planeNormal;
        private static double maxForceValuePlane = 1.0;
        private static double maxDistanceValuePlane = 1.0;

        public static void SetDirectForce(double[] force, bool enable)
        {
            currentDirectForce = force;
            directForceEnabled = enable;
            UpdateForces();
        }

        public static void SetPullToPoint(
            DeviceManager.Vector3D target,
            bool enable,
            double maxForce,
            double maxDistance,
            bool useInterpolation = false,
            double interpWindow = 30.0
        )
        {
            pullToPointEnabled = enable;
            targetPoint = target;
            maxForceValuePoint = maxForce;
            maxDistanceValuePoint = maxDistance;
            UpdateForces();
        }

        public static void SetPullToPlane(
            DeviceManager.Vector3D origin,
            DeviceManager.Vector3D normal,
            bool enable,
            double maxForce,
            double maxDistance
        )
        {
            pullToPlaneEnabled = enable;
            planeOrigin = origin;
            planeNormal = normal;
            maxForceValuePlane = maxForce;
            maxDistanceValuePlane = maxDistance;
            UpdateForces();
        }

        public static void UpdateForces()
        {
            var totalForce = new double[3];
            var position = new double[3];
            HDdll.hdGetDoublev(HDdll.HD_CURRENT_POSITION, position);

            // Apply direct force
            if (directForceEnabled)
            {
                for (int i = 0; i < 3; i++)
                    totalForce[i] += currentDirectForce[i];
            }

            // Apply pull-to-point force
            if (pullToPointEnabled)
            {
                var pointForce = CalculatePullToPointForce(position);
                for (int i = 0; i < 3; i++)
                    totalForce[i] += pointForce[i];
            }

            // Apply pull-to-plane force
            if (pullToPlaneEnabled)
            {
                var planeForce = CalculatePullToPlaneForce(position);
                for (int i = 0; i < 3; i++)
                    totalForce[i] += planeForce[i];
            }

            // Set final force
            HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, totalForce);
        }

        private static double[] CalculatePullToPointForce(double[] position)
        {
            var devicePos = new DeviceManager.Vector3D(
                -position[0],  // Negate X for coordinate system match
                position[2],   // Z becomes Y
                position[1]    // Y becomes Z
            );

            // Calculate direction and distance to target
            var dx = targetPoint.X - devicePos.X;
            var dy = targetPoint.Y - devicePos.Y;
            var dz = targetPoint.Z - devicePos.Z;

            var distance = Math.Sqrt(dx * dx + dy * dy + dz * dz);

            if (distance < 0.001)
            {
                return new double[] { 0, 0, 0 };
            }

            // Normalize direction and calculate force magnitude
            var scale = distance > maxDistanceValuePoint ? maxForceValuePoint : maxForceValuePoint * (distance / maxDistanceValuePoint);
            var fx = (dx / distance) * scale;
            var fy = (dy / distance) * scale;
            var fz = (dz / distance) * scale;

            // Convert back to device coordinates
            return new double[]
            {
                -fx,  // Negate X for device space
                fz,   // Y becomes Z
                fy    // Z becomes Y
            };
        }

        private static double[] CalculatePullToPlaneForce(double[] position)
        {
            var devicePos = new DeviceManager.Vector3D(
                -position[0],  // Negate X for coordinate system match
                position[2],   // Z becomes Y
                position[1]    // Y becomes Z
            );

            // Calculate vector from plane origin to device position
            double dx = devicePos.X - planeOrigin.X;
            double dy = devicePos.Y - planeOrigin.Y;
            double dz = devicePos.Z - planeOrigin.Z;

            // Compute signed distance to plane (dot product with normal)
            double distance = dx * planeNormal.X + dy * planeNormal.Y + dz * planeNormal.Z;
            double absDistance = Math.Abs(distance);

            double forceMagnitude;
            if (absDistance > maxDistanceValuePlane)
            {
                forceMagnitude = maxForceValuePlane;
            }
            else
            {
                forceMagnitude = (absDistance / maxDistanceValuePlane) * maxForceValuePlane;
            }

            // Determine direction towards the plane
            double direction = distance > 0 ? -1 : 1;
            double fx = planeNormal.X * direction * forceMagnitude;
            double fy = planeNormal.Y * direction * forceMagnitude;
            double fz = planeNormal.Z * direction * forceMagnitude;

            // Convert back to device coordinates
            return new double[]
            {
                -fx,  // Negate X for device space
                fz,   // Y becomes Z
                fy    // Z becomes Y
            };
        }

        public static void Reset()
        {
            directForceEnabled = false;
            pullToPointEnabled = false;
            pullToPlaneEnabled = false;
            HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, new double[3]);
        }
    }
}