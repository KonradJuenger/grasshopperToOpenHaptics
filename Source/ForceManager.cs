using System;
using System.Diagnostics;
using Rhino.Geometry; // Assuming Curve is Rhino.Geometry.Curve
using System.Collections.Generic; // Required for List

// Ensure HDdll and UCManager are accessible via correct using statements if not in this namespace.
// using YourOpenHapticsWrapperNamespace; // For HDdll
// using YourMicrocontrollerWrapperNamespace; // For UCManager

namespace ghoh
{
    public static class ForceManager
    {
        // General state
        private static double[] currentTotalForce = new double[3]; // Stores the last calculated total force for external query.

        // Direct force parameters
        private static bool directForceEnabled;
        private static double[] currentDirectForce = new double[3];

        // Filtered force support (UKF)
        private static bool filteredForceEnabled;
        private static UKF forceFilter; // Unscented Kalman Filter for smoothing forces.
        private static double[] lastFilteredForce = new double[3];
        private static double processNoise = 0.05;
        private static double measurementNoise = 0.3;

        // Tool Center Point (TCP) Offset
        private static DeviceManager.Vector3D tcpOffset = new DeviceManager.Vector3D(0, 0, 0);

        // Pull to point parameters
        private static bool pullToPointEnabled;
        private static DeviceManager.Vector3D targetPoint;
        private static DeviceManager.Vector3D currentSmoothedTarget;
        private static double maxForceValuePoint = 1.0;
        private static double maxDistanceValuePoint = 1.0;
        private static bool interpolationEnabled;
        private static double maxStepSize = 5.0;

        // Pull to plane parameters
        private static bool pullToPlaneEnabled;
        private static DeviceManager.Vector3D planeOrigin;
        private static DeviceManager.Vector3D planeNormal;
        private static double maxForceValuePlane = 1.0;
        private static double maxDistanceValuePlane = 1.0;

        // Damping parameters
        public enum DampingMethod { ExponentialSmoothing, ForceDerivative, Both }
        private static bool dampingEnabled = false;
        private static double dampingCoefficient = 0.5;
        private static double derivativeDampingCoefficient = 0.0;
        private static DampingMethod currentDampingMethod = DampingMethod.ExponentialSmoothing;
        private static double[] lastAppliedForce = new double[3];
        private static double[] previousForce = new double[3];
        private static DateTime lastForceUpdateTime = DateTime.Now;

        // Viscous damping parameters
        private static bool viscousDampingEnabled;
        private static double viscousGain = 0.5;
        private static double viscousMaxForce = 3.0;
        private static double velocityDeadband = 10.0;
        private static double deadbandSoftness = 5.0;
        private static double velocityFilterCoefficient = 0.9;
        private static int forceWindowSize = 10;
        private static double[] lastVelocity = new double[3];
        private static double[] filteredVelocity = new double[3];
        private static double[] lastViscousForce = new double[3];
        private static DateTime lastViscousUpdateTime = DateTime.Now;
        private static VectorMovingAverageFilter viscousForceFilter;

        // Plane collision parameters
        private static bool planeCollisionEnabled;
        private static DeviceManager.Vector3D collisionPlaneOrigin;
        private static DeviceManager.Vector3D collisionPlaneNormal;
        private static double maxForceValueCollision = 1.0;
        private static double maxDistanceValueCollision = 1.0;

        // Curve-based vibration parameters
        public static bool vibrationCurveEnabled = false;
        public static Curve vibrationCurve = null;
        private static DeviceManager.Vector3D vibrationCurveDirection = new DeviceManager.Vector3D(0, 0, 1);
        private static double vibrationCurveDeadzone = 0.0;
        private static double vibrationCurveMaxDistance = 10.0;
        private static double vibrationCurveMaxAmplitude = 1.0; // Max amplitude for curve vibration.
        private static double vibrationCurveFrequency = 100.0;
        private static bool vibrationCurveInvertMapping = false;
        private static bool vibrationCurveUseSquareWave = false;
        public static double vibrationCurveDistance = 0.0;
        public static double vibrationCurveAmplitude = 0.0;
        private static Stopwatch curveCalcStopwatch = new Stopwatch();
        public static double vibrationCurveCalcTime = 0.0;

        // Pull to curve parameters
        private static bool pullToCurveEnabled = false;
        private static Curve pullToCurve = null;
        private static double maxForceValueCurve = 1.0;
        private static double maxDistanceValueCurve = 1.0;
        private static bool pullCurveFade = false;
        private static int pullCurveMethod = 0;
        private static bool pullAlongEnabled = false;
        private static double travelingPointSpeed = 10.0;
        private static double travelingPointPosition = 0.0;
        private static double travelingPointLengthPosition = 0.0;
        private static DateTime lastTravelingPointUpdateTime = DateTime.Now;
        private static bool travelingPointActive = false;
        private static double totalCurveLength = 0.0;
        private static double tangentForce = 1.0;
        private static Curve originalPullToCurve = null;
        private static double originalCurveLength = 0.0;

        // Vibration Point Handling (Multiple Points)
        private struct VibrationPointData
        {
            public DeviceManager.Vector3D Target;
            public DeviceManager.Vector3D Direction; // Normalized vibration direction in device orientation space.
            public double Deadzone;
            public double MaxDistance;
            public double MaxAmplitude; // Max amplitude this single point can contribute (set by global param).
            public double Frequency;
            public bool InvertMapping;
            public bool UseSquareWave;
        }
        private static List<VibrationPointData> activeVibrationPoints = new List<VibrationPointData>();
        private static bool globalVibrationPointEnabled = false;
        // Stores the overall maximum amplitude for the combined point vibrations, set by SetVibratePoints.
        private static double currentOverallMaxVibrationAmplitude = 3.0; // Default cap, typically overridden.


        /// <summary>
        /// Retrieves a copy of the last calculated total force vector.
        /// </summary>
        public static double[] GetCurrentForce()
        {
            return (double[])currentTotalForce.Clone();
        }

        /// <summary>
        /// Configures viscous damping parameters.
        /// </summary>
        /// <returns>The last calculated viscous force vector (Rhino coordinates).</returns>
        public static Vector3d SetViscousDamping(bool enable, double gain, double maxForce, double deadbandThreshold = 10.0, double softness = 5.0, double filterCoefficient = 0.9, int windowSize = 10)
        {
            viscousDampingEnabled = enable;
            viscousGain = Math.Max(0.0, Math.Min(1.0, gain));
            viscousMaxForce = Math.Max(0.0, maxForce);
            velocityDeadband = Math.Max(0.0, deadbandThreshold);
            deadbandSoftness = Math.Max(0.0, Math.Min(velocityDeadband, softness));
            velocityFilterCoefficient = Math.Max(0.0, Math.Min(0.99, filterCoefficient));
            forceWindowSize = Math.Max(1, windowSize);
            return new Vector3d(-lastViscousForce[0], lastViscousForce[2], lastViscousForce[1]);
        }

        /// <summary>
        /// Calculates the viscous force based on current device velocity and parameters.
        /// </summary>
        private static double[] CalculateViscousForce()
        {
            if (viscousForceFilter == null)
            {
                viscousForceFilter = new VectorMovingAverageFilter(forceWindowSize);
            }
            else if (viscousForceFilter.GetWindowSize() != forceWindowSize)
            {
                viscousForceFilter.SetWindowSize(forceWindowSize);
            }

            DateTime currentTime = DateTime.Now;
            double timeDelta = Math.Max(0.001, (currentTime - lastViscousUpdateTime).TotalSeconds);
            lastViscousUpdateTime = currentTime;

            var rawVelocity = new double[3];
            HDdll.hdGetDoublev(HDdll.HD_CURRENT_VELOCITY, rawVelocity);

            for (int i = 0; i < 3; i++)
            {
                filteredVelocity[i] = velocityFilterCoefficient * filteredVelocity[i] +
                                     (1 - velocityFilterCoefficient) * rawVelocity[i];
            }

            double velMagnitude = Math.Sqrt(filteredVelocity[0] * filteredVelocity[0] +
                                           filteredVelocity[1] * filteredVelocity[1] +
                                           filteredVelocity[2] * filteredVelocity[2]);

            double[] rawViscousForce = new double[3];
            rawViscousForce[0] = -viscousGain * filteredVelocity[0];
            rawViscousForce[1] = -viscousGain * filteredVelocity[1];
            rawViscousForce[2] = -viscousGain * filteredVelocity[2];

            double forceMag = Math.Sqrt(rawViscousForce[0] * rawViscousForce[0] +
                                       rawViscousForce[1] * rawViscousForce[1] +
                                       rawViscousForce[2] * rawViscousForce[2]);

            if (forceMag > viscousMaxForce && forceMag > 0.0001)
            {
                double scale = viscousMaxForce / forceMag;
                rawViscousForce[0] *= scale;
                rawViscousForce[1] *= scale;
                rawViscousForce[2] *= scale;
            }

            double[] smoothedForce = viscousForceFilter.AddSample(rawViscousForce);

            double[] finalForce = new double[3];
            double scaleFactor = 1.0;

            if (velMagnitude < velocityDeadband - deadbandSoftness)
            {
                scaleFactor = 0.0;
            }
            else if (velMagnitude < velocityDeadband)
            {
                double transitionPosition = velMagnitude - (velocityDeadband - deadbandSoftness);
                scaleFactor = transitionPosition / deadbandSoftness;
            }

            finalForce[0] = smoothedForce[0] * scaleFactor;
            finalForce[1] = smoothedForce[1] * scaleFactor;
            finalForce[2] = smoothedForce[2] * scaleFactor;

            lastVelocity = (double[])filteredVelocity.Clone();
            lastViscousForce = (double[])finalForce.Clone();
            return finalForce;
        }

        /// <summary>
        /// Sets the Tool Center Point (TCP) offset from the device's gimbal point.
        /// </summary>
        public static void SetTCPOffset(DeviceManager.Vector3D offset)
        {
            tcpOffset = offset;
        }

        /// <summary>
        /// Sets a direct force vector to be applied, optionally using a UKF filter.
        /// </summary>
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
                directForceEnabled = false;
            }
            else
            {
                currentDirectForce = force;
                directForceEnabled = enable;
                filteredForceEnabled = false;
            }
        }

        /// <summary>
        /// Sets parameters for the Unscented Kalman Filter (UKF) used with direct forces.
        /// </summary>
        public static void SetFilterParams(double q, double r)
        {
            processNoise = Math.Max(0.001, Math.Min(q, 1.0));
            measurementNoise = Math.Max(0.001, Math.Min(r, 1.0));
            if (forceFilter != null)
            {
                forceFilter.SetNoiseParams(processNoise, measurementNoise);
            }
        }

        /// <summary>
        /// Configures damping parameters for forces.
        /// </summary>
        public static void SetDampingParameters(bool enable, double coefficient, double derivativeCoefficient, DampingMethod method)
        {
            dampingEnabled = enable;
            dampingCoefficient = Math.Max(0, Math.Min(coefficient, 0.99));
            derivativeDampingCoefficient = Math.Max(0, Math.Min(derivativeCoefficient, 1.0));
            currentDampingMethod = method;
        }

        /// <summary>
        /// Sets parameters for pulling the device towards a single point.
        /// </summary>
        public static void SetPullToPoint(DeviceManager.Vector3D target, bool enable, double maxForce, double maxDistance, bool useInterpolation = false, double stepSize = 5.0)
        {
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

        /// <summary>
        /// Calculates the force vector to pull the device towards the target point.
        /// </summary>
        private static double[] CalculatePullToPointForce(DeviceManager.Vector3D devicePos)
        {
            var targetToUse = interpolationEnabled ? currentSmoothedTarget : targetPoint;
            var dx = targetToUse.X - devicePos.X;
            var dy = targetToUse.Y - devicePos.Y;
            var dz = targetToUse.Z - devicePos.Z;
            var distance = Math.Sqrt(dx * dx + dy * dy + dz * dz);

            if (distance < 0.001) return new double[] { 0, 0, 0 };

            var scale = distance > maxDistanceValuePoint ? maxForceValuePoint : maxForceValuePoint * (distance / maxDistanceValuePoint);
            var fx = (dx / distance) * scale;
            var fy = (dy / distance) * scale;
            var fz = (dz / distance) * scale;
            return new double[] { -fx, fz, fy }; // Rhino-like direction to Device Force components.
        }

        /// <summary>
        /// Updates the smoothed target position for interpolated pull-to-point.
        /// </summary>
        private static void UpdateSmoothedTarget(DeviceManager.Vector3D currentPosition)
        {
            if (!interpolationEnabled)
            {
                currentSmoothedTarget = targetPoint;
                return;
            }
            double dx = targetPoint.X - currentSmoothedTarget.X;
            double dy = targetPoint.Y - currentSmoothedTarget.Y;
            double dz = targetPoint.Z - currentSmoothedTarget.Z;
            double distanceToTarget = Math.Sqrt(dx * dx + dy * dy + dz * dz);

            if (distanceToTarget < 0.001) return;

            double step = Math.Min(distanceToTarget, maxStepSize);
            double scaleFactor = step / distanceToTarget;
            currentSmoothedTarget = new DeviceManager.Vector3D(
                currentSmoothedTarget.X + dx * scaleFactor,
                currentSmoothedTarget.Y + dy * scaleFactor,
                currentSmoothedTarget.Z + dz * scaleFactor
            );
        }

        /// <summary>
        /// Sets parameters for pulling the device towards a plane.
        /// </summary>
        public static void SetPullToPlane(DeviceManager.Vector3D origin, DeviceManager.Vector3D normal, bool enable, double maxForce, double maxDistance)
        {
            pullToPlaneEnabled = enable;
            planeOrigin = origin;
            planeNormal = normal;
            maxForceValuePlane = maxForce;
            maxDistanceValuePlane = maxDistance;
        }

        /// <summary>
        /// Calculates the force vector to pull the device towards the defined plane.
        /// </summary>
        private static double[] CalculatePullToPlaneForce(DeviceManager.Vector3D devicePos)
        {
            double dx = devicePos.X - planeOrigin.X;
            double dy = devicePos.Y - planeOrigin.Y;
            double dz = devicePos.Z - planeOrigin.Z;
            double distance = dx * planeNormal.X + dy * planeNormal.Y + dz * planeNormal.Z;
            double absDistance = Math.Abs(distance);
            double forceMagnitude;

            if (absDistance > maxDistanceValuePlane) forceMagnitude = maxForceValuePlane;
            else forceMagnitude = (absDistance / maxDistanceValuePlane) * maxForceValuePlane;

            double directionSign = distance > 0 ? -1 : 1;
            double fx = planeNormal.X * directionSign * forceMagnitude;
            double fy = planeNormal.Y * directionSign * forceMagnitude;
            double fz = planeNormal.Z * directionSign * forceMagnitude;
            return new double[] { -fx, fz, fy }; // Rhino-like direction to Device Force components.
        }

        /// <summary>
        /// Sets parameters for plane collision.
        /// </summary>
        public static void SetPlaneCollision(DeviceManager.Vector3D origin, DeviceManager.Vector3D normal, bool enable, double maxForce, double maxDistance)
        {
            planeCollisionEnabled = enable;
            collisionPlaneOrigin = origin;
            collisionPlaneNormal = normal;
            maxForceValueCollision = maxForce;
            maxDistanceValueCollision = maxDistance;
        }

        /// <summary>
        /// Calculates the collision force if the device penetrates the collision plane.
        /// </summary>
        private static double[] CalculatePlaneCollisionForce(DeviceManager.Vector3D devicePos)
        {
            double dx = devicePos.X - collisionPlaneOrigin.X;
            double dy = devicePos.Y - collisionPlaneOrigin.Y;
            double dz = devicePos.Z - collisionPlaneOrigin.Z;
            double distance = dx * collisionPlaneNormal.X + dy * collisionPlaneNormal.Y + dz * collisionPlaneNormal.Z;

            if (distance >= 0) return new double[] { 0, 0, 0 };

            double penetrationDepth = -distance;
            double forceMagnitude;

            if (penetrationDepth > maxDistanceValueCollision) forceMagnitude = maxForceValueCollision;
            else forceMagnitude = (penetrationDepth / maxDistanceValueCollision) * maxForceValueCollision;

            double fx = collisionPlaneNormal.X * forceMagnitude;
            double fy = collisionPlaneNormal.Y * forceMagnitude;
            double fz = collisionPlaneNormal.Z * forceMagnitude;
            return new double[] { -fx, fz, fy }; // Rhino-like direction to Device Force components.
        }

        /// <summary>
        /// Applies damping to a given force vector.
        /// </summary>
        private static double[] ApplyDamping(double[] currentForce, double timeDelta)
        {
            if (!dampingEnabled || (dampingCoefficient < 0.01 && derivativeDampingCoefficient < 0.01))
            {
                previousForce = (double[])lastAppliedForce.Clone();
                lastAppliedForce = (double[])currentForce.Clone();
                return currentForce;
            }
            double[] dampedForce = new double[3];
            switch (currentDampingMethod)
            {
                case DampingMethod.ExponentialSmoothing:
                    double alpha = 1.0 - dampingCoefficient;
                    for (int i = 0; i < 3; i++) dampedForce[i] = alpha * currentForce[i] + dampingCoefficient * lastAppliedForce[i];
                    break;
                case DampingMethod.ForceDerivative:
                    for (int i = 0; i < 3; i++)
                    {
                        double derivative = (currentForce[i] - previousForce[i]) / Math.Max(0.001, timeDelta);
                        dampedForce[i] = currentForce[i] - (derivativeDampingCoefficient * derivative);
                    }
                    break;
                case DampingMethod.Both:
                    double alphaBoth = 1.0 - dampingCoefficient;
                    double[] smoothedForce = new double[3];
                    for (int i = 0; i < 3; i++) smoothedForce[i] = alphaBoth * currentForce[i] + dampingCoefficient * lastAppliedForce[i];
                    for (int i = 0; i < 3; i++)
                    {
                        double derivative = (smoothedForce[i] - previousForce[i]) / Math.Max(0.001, timeDelta);
                        dampedForce[i] = smoothedForce[i] - (derivativeDampingCoefficient * derivative);
                    }
                    break;
                default:
                    Array.Copy(currentForce, dampedForce, 3);
                    break;
            }
            previousForce = (double[])lastAppliedForce.Clone();
            lastAppliedForce = (double[])dampedForce.Clone();
            return dampedForce;
        }

        // --- Multi-Point Vibration Methods ---

        /// <summary>
        /// Clears all currently defined vibration points and disables point vibrations.
        /// </summary>
        public static void ClearVibratePoints()
        {
            activeVibrationPoints.Clear();
            globalVibrationPointEnabled = false;
        }

        /// <summary>
        /// Sets or updates the list of points for vibration.
        /// All points share common behavioral parameters (direction, frequency, etc.).
        /// The overallMaxAmplitude parameter from GH component will be used to cap the final combined vibration.
        /// </summary>
        public static void SetVibratePoints(
            List<DeviceManager.Vector3D> deviceSpaceTargets,
            DeviceManager.Vector3D deviceSpaceDirection,
            bool enable,
            double deadzone,
            double maxDistance,
            double overallMaxAmplitude, // This is the MaxAmplitude from GH, used for capping.
            double frequency,
            bool invertMapping,
            bool useSquareWave)
        {
            ClearVibratePoints();
            globalVibrationPointEnabled = enable;

            if (!enable || deviceSpaceTargets == null || deviceSpaceTargets.Count == 0)
            {
                return;
            }

            DeviceManager.Vector3D normalizedDeviceDirection = deviceSpaceDirection;
            double dirMagSq = deviceSpaceDirection.X * deviceSpaceDirection.X +
                              deviceSpaceDirection.Y * deviceSpaceDirection.Y +
                              deviceSpaceDirection.Z * deviceSpaceDirection.Z;

            if (dirMagSq > 0.000001 && Math.Abs(dirMagSq - 1.0) > 0.000001)
            {
                double mag = Math.Sqrt(dirMagSq);
                normalizedDeviceDirection = new DeviceManager.Vector3D(
                    deviceSpaceDirection.X / mag, deviceSpaceDirection.Y / mag, deviceSpaceDirection.Z / mag);
            }
            else if (dirMagSq < 0.000001)
            {
                normalizedDeviceDirection = new DeviceManager.Vector3D(0, 0, 1); // Default to Z-axis.
            }

            double clampedDeadzone = Math.Max(0.0, deadzone);
            double clampedMaxDistance = Math.Max(clampedDeadzone + 0.001, maxDistance);
            // Store the overallMaxAmplitude for capping the final combined effect.
            currentOverallMaxVibrationAmplitude = Math.Max(0.0, Math.Min(3.0, overallMaxAmplitude));
            double clampedFrequency = Math.Max(1.0, Math.Min(1000.0, frequency));

            foreach (var target in deviceSpaceTargets)
            {
                activeVibrationPoints.Add(new VibrationPointData
                {
                    Target = target,
                    Direction = normalizedDeviceDirection,
                    Deadzone = clampedDeadzone,
                    MaxDistance = clampedMaxDistance,
                    // Each point's individual max contribution is the overall cap.
                    // This means if only one point is active, it can reach currentOverallMaxVibrationAmplitude.
                    MaxAmplitude = currentOverallMaxVibrationAmplitude,
                    Frequency = clampedFrequency,
                    InvertMapping = invertMapping,
                    UseSquareWave = useSquareWave
                });
            }
        }

        /// <summary>
        /// Calculates the combined vibration force from all active points,
        /// capping the effective total amplitude.
        /// </summary>
        private static double[] CalculateCombinedVibrationForce(DeviceManager.Vector3D currentDevicePos)
        {
            // This vector will store the sum of (Direction_force_coords * individual_amplitude_factor_for_this_point).
            // Its magnitude represents the potential combined amplitude before global capping and oscillation.
            double[] summedPotentialForceDirection = new double[] { 0, 0, 0 };

            if (!globalVibrationPointEnabled || activeVibrationPoints.Count == 0)
            {
                return summedPotentialForceDirection; // Returns {0,0,0}
            }

            foreach (var pointData in activeVibrationPoints)
            {
                double dx = currentDevicePos.X - pointData.Target.X;
                double dy = currentDevicePos.Y - pointData.Target.Y;
                double dz = currentDevicePos.Z - pointData.Target.Z;
                double distance = Math.Sqrt(dx * dx + dy * dy + dz * dz);

                // This 'individualAmplitudeFactor' is how much this point *would* contribute (0 to pointData.MaxAmplitude).
                // pointData.MaxAmplitude here is already set to currentOverallMaxVibrationAmplitude.
                double individualAmplitudeFactor = 0.0;
                double usableRange = pointData.MaxDistance - pointData.Deadzone;

                if (pointData.InvertMapping)
                {
                    if (distance >= pointData.MaxDistance) individualAmplitudeFactor = 0.0;
                    else if (distance < pointData.Deadzone) individualAmplitudeFactor = pointData.MaxAmplitude; // Can reach full potential
                    else individualAmplitudeFactor = (1.0 - (distance - pointData.Deadzone) / usableRange) * pointData.MaxAmplitude;
                }
                else
                {
                    if (distance >= pointData.MaxDistance) individualAmplitudeFactor = pointData.MaxAmplitude; // Can reach full potential
                    else if (distance < pointData.Deadzone) individualAmplitudeFactor = 0.0;
                    else individualAmplitudeFactor = ((distance - pointData.Deadzone) / usableRange) * pointData.MaxAmplitude;
                }
                individualAmplitudeFactor = Math.Max(0.0, Math.Min(individualAmplitudeFactor, pointData.MaxAmplitude));

                if (individualAmplitudeFactor < 0.0001) continue;

                // Accumulate the potential force contribution (direction * factor) for this point.
                // Oscillation is applied once at the end to the capped sum.
                // The Direction components are from device orientation space, convert to device force space.
                summedPotentialForceDirection[0] += -pointData.Direction.X * individualAmplitudeFactor;
                summedPotentialForceDirection[1] += pointData.Direction.Z * individualAmplitudeFactor;
                summedPotentialForceDirection[2] += pointData.Direction.Y * individualAmplitudeFactor;
            }

            // Calculate the magnitude of the summed potential force vector.
            double currentTotalPotentialMagnitude = Math.Sqrt(
                summedPotentialForceDirection[0] * summedPotentialForceDirection[0] +
                summedPotentialForceDirection[1] * summedPotentialForceDirection[1] +
                summedPotentialForceDirection[2] * summedPotentialForceDirection[2]
            );

            // Determine scaling factor to cap the magnitude to currentOverallMaxVibrationAmplitude.
            double capScaleFactor = 1.0;
            if (currentTotalPotentialMagnitude > currentOverallMaxVibrationAmplitude && currentTotalPotentialMagnitude > 0.0001)
            {
                capScaleFactor = currentOverallMaxVibrationAmplitude / currentTotalPotentialMagnitude;
            }

            // Get the global oscillation multiplier (using parameters from the first active point as representative,
            // as frequency and wave type are shared among all points in a single SetVibratePoints call).
            double representativeFrequency = 100.0; // Default if no points (though list shouldn't be empty here).
            bool representativeSquareWave = false;
            if (activeVibrationPoints.Count > 0)
            {
                representativeFrequency = activeVibrationPoints[0].Frequency;
                representativeSquareWave = activeVibrationPoints[0].UseSquareWave;
            }
            double oscillation = Vibration.GetVibrationMultiplier(representativeFrequency, representativeSquareWave);

            // Apply capping scale factor and oscillation to the summed potential force.
            double[] finalCombinedForce = new double[3];
            finalCombinedForce[0] = summedPotentialForceDirection[0] * capScaleFactor * oscillation;
            finalCombinedForce[1] = summedPotentialForceDirection[1] * capScaleFactor * oscillation;
            finalCombinedForce[2] = summedPotentialForceDirection[2] * capScaleFactor * oscillation;

            return finalCombinedForce;
        }


        // --- Curve-based forces and vibrations (Placeholders for brevity, ensure your full implementations are here) ---
        public static void SetPullToCurve(Curve curve, bool enable, double maxForce, double maxDistance, bool fade, int method, double tangentForceValue, bool pullAlong, double speed, bool reset)
        {
            pullToCurveEnabled = enable; pullToCurve = curve; maxForceValueCurve = maxForce; /* ... more params ... */
        }
        private static double[] CalculatePullToCurveForce(DeviceManager.Vector3D devicePos) { /* ... Your complex logic ... */ return new double[3]; }

        public static void SetVibrateCurve(Curve curve, DeviceManager.Vector3D direction, bool enable, double deadzone, double maxDistance, double maxAmplitude, double frequency, bool invertMapping, bool useSquareWave)
        {
            vibrationCurveEnabled = enable; vibrationCurve = curve; vibrationCurveDirection = direction; /* ... more params ... */
        }
        private static double[] CalculateVibrationCurveForce(DeviceManager.Vector3D devicePos) { /* ... Your complex logic ... */ return new double[3]; }
        private static void ResetTravelingPoint() { /* ... */ }
        private static double FindParameterAtLength(Curve curve, double targetLength) { /* ... */ return 0.0; }
        private static void UpdateTravelingPointPosition() { /* ... */ }


        // --- Main Force Calculation Loop (Called by DeviceManager's servo loop) ---
        public static void UpdateForces()
        {
            DateTime currentTime = DateTime.Now; // For time-dependent calculations like damping.
            double[] totalForceForFrame = new double[3]; // Initialize total force for this frame to zero.

            var transformMatrix = new double[16]; // To store device's current transform.
            HDdll.hdGetDoublev(HDdll.HD_CURRENT_TRANSFORM, transformMatrix);

            // Determine current device position in device coordinates.
            var currentDevicePos = new DeviceManager.Vector3D(
                -transformMatrix[12],
                 transformMatrix[14],
                 transformMatrix[13]
            );

            // Apply TCP offset if configured.
            if (tcpOffset.X != 0 || tcpOffset.Y != 0 || tcpOffset.Z != 0)
            {
                var xDir = new DeviceManager.Vector3D(-transformMatrix[0], transformMatrix[2], transformMatrix[1]);
                var yDir = new DeviceManager.Vector3D(-transformMatrix[4], transformMatrix[6], transformMatrix[5]);
                var zDir = new DeviceManager.Vector3D(-transformMatrix[8], transformMatrix[10], transformMatrix[9]);
                currentDevicePos.X += tcpOffset.X * xDir.X + tcpOffset.Y * yDir.X + tcpOffset.Z * zDir.X;
                currentDevicePos.Y += tcpOffset.X * xDir.Y + tcpOffset.Y * yDir.Y + tcpOffset.Z * zDir.Y;
                currentDevicePos.Z += tcpOffset.X * zDir.X + tcpOffset.Y * zDir.Y + tcpOffset.Z * zDir.Z;
            }

            // --- Accumulate all active forces ---
            if (viscousDampingEnabled)
            {
                var f = CalculateViscousForce(); for (int i = 0; i < 3; i++) totalForceForFrame[i] += f[i];
            }
            if (planeCollisionEnabled)
            {
                var f = CalculatePlaneCollisionForce(currentDevicePos); for (int i = 0; i < 3; i++) totalForceForFrame[i] += f[i];
            }
            // Combined Point Vibration
            if (globalVibrationPointEnabled && activeVibrationPoints.Count > 0)
            {
                var f = CalculateCombinedVibrationForce(currentDevicePos); for (int i = 0; i < 3; i++) totalForceForFrame[i] += f[i];
            }
            if (vibrationCurveEnabled && vibrationCurve != null)
            {
                var f = CalculateVibrationCurveForce(currentDevicePos); for (int i = 0; i < 3; i++) totalForceForFrame[i] += f[i];
            }
            if (pullToPointEnabled)
            {
                UpdateSmoothedTarget(currentDevicePos);
                var f = CalculatePullToPointForce(currentDevicePos); for (int i = 0; i < 3; i++) totalForceForFrame[i] += f[i];
            }
            if (pullToCurveEnabled && pullToCurve != null)
            {
                var f = CalculatePullToCurveForce(currentDevicePos); for (int i = 0; i < 3; i++) totalForceForFrame[i] += f[i];
            }
            if (pullToPlaneEnabled)
            {
                var f = CalculatePullToPlaneForce(currentDevicePos); for (int i = 0; i < 3; i++) totalForceForFrame[i] += f[i];
            }
            // Direct forces (filtered or raw)
            if (filteredForceEnabled && forceFilter != null)
            {
                for (int i = 0; i < 3; i++) totalForceForFrame[i] += lastFilteredForce[i];
            }
            else if (directForceEnabled)
            {
                for (int i = 0; i < 3; i++) totalForceForFrame[i] += currentDirectForce[i];
            }

            // Microcontroller force input (example using reflection for robustness)
            try
            {
                var ucManagerType = Type.GetType("ghoh.UCManager"); // Attempt to get UCManager type.
                if (ucManagerType != null)
                {
                    var isConnectedProp = ucManagerType.GetProperty("IsConnected");
                    var forceEnabledProp = ucManagerType.GetProperty("ForceEnabled");
                    var getMappedForceMethod = ucManagerType.GetMethod("GetMappedForceValue");

                    if (isConnectedProp != null && forceEnabledProp != null && getMappedForceMethod != null &&
                        (bool)isConnectedProp.GetValue(null) && (bool)forceEnabledProp.GetValue(null))
                    {
                        var zDirDevice = new DeviceManager.Vector3D(-transformMatrix[8], transformMatrix[10], transformMatrix[9]);
                        double rawMCVal = (double)getMappedForceMethod.Invoke(null, null);
                        if (rawMCVal > 0.001)
                        {
                            double fx = zDirDevice.X * rawMCVal;
                            double fy = zDirDevice.Y * rawMCVal;
                            double fz = zDirDevice.Z * rawMCVal;
                            double[] mcForce = new double[3] { -fx, fz, fy };
                            double timeDelta = (currentTime - lastForceUpdateTime).TotalSeconds;
                            lastForceUpdateTime = currentTime;
                            double[] dampedMCForce = ApplyDamping(mcForce, timeDelta);
                            for (int i = 0; i < 3; i++) totalForceForFrame[i] += dampedMCForce[i];
                        }
                    }
                }
            }
            catch (Exception) { /* Silently ignore if UCManager reflection fails, or log error if preferred. */ }
            // --- End Force Accumulation ---

            currentTotalForce = (double[])totalForceForFrame.Clone();
            HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, totalForceForFrame);
        }

        /// <summary>
        /// Resets all force states to their defaults and clears forces on the device.
        /// </summary>
        public static void Reset()
        {
            directForceEnabled = false;
            pullToPointEnabled = false;
            pullToPlaneEnabled = false;
            filteredForceEnabled = false;
            interpolationEnabled = false;
            dampingEnabled = false;
            viscousDampingEnabled = false;
            planeCollisionEnabled = false;
            vibrationCurveEnabled = false;
            pullToCurveEnabled = false;

            tcpOffset = new DeviceManager.Vector3D(0, 0, 0);
            ClearVibratePoints();
            currentOverallMaxVibrationAmplitude = 3.0; // Reset to a default max amplitude.

            vibrationCurve = null;
            pullToCurve = null;
            originalPullToCurve = null;
            pullCurveFade = false;
            pullCurveMethod = 0;
            pullAlongEnabled = false;
            travelingPointSpeed = 10.0;
            travelingPointPosition = 0.0;
            travelingPointLengthPosition = 0.0;
            lastTravelingPointUpdateTime = DateTime.Now;
            travelingPointActive = false;
            totalCurveLength = 0.0;
            originalCurveLength = 0.0;
            tangentForce = 1.0;

            Vibration.Reset();
            if (forceFilter != null) forceFilter.Reset();

            lastAppliedForce = new double[3];
            previousForce = new double[3];
            lastViscousForce = new double[3];
            lastVelocity = new double[3];
            filteredVelocity = new double[3];
            currentTotalForce = new double[3];

            HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, new double[3]);
        }
    }
}
