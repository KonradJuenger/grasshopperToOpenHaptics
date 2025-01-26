using System;

namespace ghoh
{
    public static class ForceManager
    {
        // Force types that can be enabled/disabled independently
        private static bool directForceEnabled;
        private static bool pullToPointEnabled;

        // Direct force parameters
        private static double[] currentDirectForce = new double[] { 0, 0, 0 };

        // Pull to point parameters
        private static DeviceManager.Vector3D targetPoint;
        private static double maxForceValue = 1.0;
        private static double maxDistanceValue = 1.0;
        private static bool interpolationEnabled;
        private static double interpolationTimeWindow = 30.0;
        private static DeviceManager.Vector3D previousTarget;
        private static DeviceManager.Vector3D currentInterpolatedTarget;
        private static DateTime lastTargetUpdateTime = DateTime.MinValue;

        public static void SetDirectForce(double[] force, bool enable)
        {
            currentDirectForce = force;
            directForceEnabled = enable;
            UpdateForces();
        }

        public static void SetPullToPoint(DeviceManager.Vector3D target, bool enable, double maxForce,
            double maxDistance, bool useInterpolation = false, double interpWindow = 30.0)
        {
            pullToPointEnabled = enable;
            interpolationEnabled = useInterpolation;
            interpolationTimeWindow = Math.Max(1.0, interpWindow);

            if (interpolationEnabled)
            {
                previousTarget = currentInterpolatedTarget;
                lastTargetUpdateTime = DateTime.Now;
            }
            else
            {
                currentInterpolatedTarget = target;
            }

            targetPoint = target;
            maxForceValue = maxForce;
            maxDistanceValue = maxDistance;

            UpdateForces();
        }

        public static void UpdateForces()
        {
            if (!directForceEnabled && !pullToPointEnabled)
            {
                HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, new double[] { 0, 0, 0 });
                return;
            }

            var position = new double[3];
            HDdll.hdGetDoublev(HDdll.HD_CURRENT_POSITION, position);

            var totalForce = new double[] { 0, 0, 0 };

            // Add direct force if enabled
            if (directForceEnabled)
            {
                for (int i = 0; i < 3; i++)
                {
                    totalForce[i] += currentDirectForce[i];
                }
            }

            // Add pull to point force if enabled
            if (pullToPointEnabled)
            {
                var pullForce = CalculatePullForce(position);
                for (int i = 0; i < 3; i++)
                {
                    totalForce[i] += pullForce[i];
                }
            }

            HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, totalForce);
        }

        private static double[] CalculatePullForce(double[] position)
        {
            UpdateInterpolatedTarget();
            var target = currentInterpolatedTarget;

            // Convert position to our coordinate system
            var devicePos = new DeviceManager.Vector3D(
                -position[0],  // Negate X for coordinate system match
                position[2],   // Z becomes Y
                position[1]    // Y becomes Z
            );

            // Calculate direction and distance to target
            var dx = target.X - devicePos.X;
            var dy = target.Y - devicePos.Y;
            var dz = target.Z - devicePos.Z;

            var distance = Math.Sqrt(dx * dx + dy * dy + dz * dz);

            if (distance < 0.001)
            {
                return new double[] { 0, 0, 0 };
            }

            // Normalize direction and calculate force magnitude
            var scale = distance > maxDistanceValue ? maxForceValue : maxForceValue * (distance / maxDistanceValue);
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

        private static void UpdateInterpolatedTarget()
        {
            if (!interpolationEnabled || lastTargetUpdateTime == DateTime.MinValue)
            {
                currentInterpolatedTarget = targetPoint;
                return;
            }

            double elapsedMs = (DateTime.Now - lastTargetUpdateTime).TotalMilliseconds;
            double t = Math.Min(Math.Max(elapsedMs / interpolationTimeWindow, 0.0), 1.0);

            currentInterpolatedTarget = new DeviceManager.Vector3D(
                previousTarget.X + (targetPoint.X - previousTarget.X) * t,
                previousTarget.Y + (targetPoint.Y - previousTarget.Y) * t,
                previousTarget.Z + (targetPoint.Z - previousTarget.Z) * t
            );

            if (t >= 1.0)
            {
                interpolationEnabled = false;
            }
        }

        public static void Reset()
        {
            directForceEnabled = false;
            pullToPointEnabled = false;
            currentDirectForce = new double[] { 0, 0, 0 };
            HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, currentDirectForce);
        }
    }
}