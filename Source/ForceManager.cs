using System;

namespace ghoh
{
    public static class ForceManager
    {
        // Flags for enabled forces
        private static bool directForceEnabled;
        private static bool pullToPointEnabled;
        private static bool pullToPlaneEnabled;
        private static bool filteredForceEnabled;

        // Direct force parameters
        private static double[] currentDirectForce = new double[3];

        // Filtered force support
        private static UKF forceFilter;
        private static double[] lastFilteredForce = new double[3];
        private static double processNoise = 0.05;
        private static double measurementNoise = 0.3;

        // Pull to point parameters
        private static DeviceManager.Vector3D targetPoint;
        private static DeviceManager.Vector3D currentSmoothedTarget;
        private static double maxForceValuePoint = 1.0;
        private static double maxDistanceValuePoint = 1.0;
        private static bool interpolationEnabled;
        private static double maxStepSize = 5.0;  // Maximum distance to move per update

        // Pull to plane parameters
        private static DeviceManager.Vector3D planeOrigin;
        private static DeviceManager.Vector3D planeNormal;
        private static double maxForceValuePlane = 1.0;
        private static double maxDistanceValuePlane = 1.0;

        public static void SetDirectForce(double[] force, bool enable, bool useFilter = false)
        {
            if (useFilter)
            {
                if (forceFilter == null)
                {
                    forceFilter = new UKF(3);
                    forceFilter.SetNoiseParams(processNoise, measurementNoise);
                }

                if (enable)
                {
                    forceFilter.Update(force);
                    lastFilteredForce = forceFilter.getState();
                }
                else
                {
                    forceFilter.Reset();
                }

                filteredForceEnabled = enable;
                directForceEnabled = false;  // Disable unfiltered direct force
            }
            else
            {
                currentDirectForce = force;
                directForceEnabled = enable;
                filteredForceEnabled = false;  // Disable filtered force
            }
        }

        public static void SetFilterParams(double q, double r)
        {
            processNoise = Math.Max(0.001, Math.Min(q, 1.0));
            measurementNoise = Math.Max(0.001, Math.Min(r, 1.0));
            if (forceFilter != null)
            {
                forceFilter.SetNoiseParams(processNoise, measurementNoise);
            }
        }

        public static void SetPullToPoint(
            DeviceManager.Vector3D target,
            bool enable,
            double maxForce,
            double maxDistance,
            bool useInterpolation = false,
            double stepSize = 5.0
        )
        {
            // Initialize smoothed target on first enable
            if (!pullToPointEnabled && enable)
            {
                currentSmoothedTarget = target;
            }

            pullToPointEnabled = enable;
            targetPoint = target;
            maxForceValuePoint = maxForce;
            maxDistanceValuePoint = maxDistance;
            interpolationEnabled = useInterpolation;
            maxStepSize = stepSize;
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
        }

        private static void UpdateSmoothedTarget(DeviceManager.Vector3D currentPosition)
        {
            if (!interpolationEnabled)
            {
                currentSmoothedTarget = targetPoint;
                return;
            }

            // Calculate direction and distance to actual target
            double dx = targetPoint.X - currentSmoothedTarget.X;
            double dy = targetPoint.Y - currentSmoothedTarget.Y;
            double dz = targetPoint.Z - currentSmoothedTarget.Z;

            double distanceToTarget = Math.Sqrt(dx * dx + dy * dy + dz * dz);

            if (distanceToTarget < 0.001)
            {
                return;  // Already close enough to target
            }

            // Calculate how far to move this update (distance-based)
            double stepSize = Math.Min(distanceToTarget, maxStepSize);

            // Update smoothed target position
            double scale = stepSize / distanceToTarget;
            currentSmoothedTarget = new DeviceManager.Vector3D(
                currentSmoothedTarget.X + dx * scale,
                currentSmoothedTarget.Y + dy * scale,
                currentSmoothedTarget.Z + dz * scale
            );
        }

        public static void UpdateForces()
        {
            var totalForce = new double[3];
            var position = new double[3];
            HDdll.hdGetDoublev(HDdll.HD_CURRENT_POSITION, position);

            var devicePos = new DeviceManager.Vector3D(
                -position[0],  // Negate X for coordinate system match
                position[2],   // Z becomes Y
                position[1]    // Y becomes Z
            );

            // Apply direct force (either filtered or unfiltered)
            if (filteredForceEnabled && forceFilter != null)
            {
                forceFilter.Predict();
                lastFilteredForce = forceFilter.getState();
                for (int i = 0; i < 3; i++)
                    totalForce[i] += lastFilteredForce[i];
            }
            else if (directForceEnabled)
            {
                for (int i = 0; i < 3; i++)
                    totalForce[i] += currentDirectForce[i];
            }

            // Apply pull-to-point force
            if (pullToPointEnabled)
            {
                UpdateSmoothedTarget(devicePos);
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

            // Use smoothed target for force calculation
            var targetToUse = interpolationEnabled ? currentSmoothedTarget : targetPoint;

            // Calculate direction and distance to target
            var dx = targetToUse.X - devicePos.X;
            var dy = targetToUse.Y - devicePos.Y;
            var dz = targetToUse.Z - devicePos.Z;

            var distance = Math.Sqrt(dx * dx + dy * dy + dz * dz);

            if (distance < 0.001)
            {
                return new double[] { 0, 0, 0 };
            }

            // Normalize direction and calculate force magnitude
            var scale = distance > maxDistanceValuePoint ?
                maxForceValuePoint :
                maxForceValuePoint * (distance / maxDistanceValuePoint);

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
            filteredForceEnabled = false;
            interpolationEnabled = false;
            if (forceFilter != null)
            {
                forceFilter.Reset();
            }
            HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, new double[3]);
        }
    }
}