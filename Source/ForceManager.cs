using System;
using System.Diagnostics;
using Rhino.Geometry;
using System.Collections.Generic;
using System.Linq; // For .ToList()

namespace ghoh
{
    public static class ForceManager
    {
        // General state
        private static double[] currentTotalForce = new double[3];

        // Direct force parameters
        private static bool directForceEnabled;
        private static double[] currentDirectForce = new double[3];

        // Filtered force support (UKF)
        private static bool filteredForceEnabled;
        private static UKF forceFilter;
        private static double[] lastFilteredForce = new double[3];
        private static double processNoise = 0.05;
        private static double measurementNoise = 0.3;

        // Tool Center Point (TCP) Offset
        private static DeviceManager.Vector3D tcpOffset = new DeviceManager.Vector3D(0, 0, 0);

        // --- Old Single Pull to point parameters (Commented out / To be removed) ---
        // private static bool pullToPointEnabled; 
        // private static DeviceManager.Vector3D targetPoint;
        // private static DeviceManager.Vector3D currentSmoothedTarget;
        // private static double maxForceValuePoint = 1.0;
        // private static double maxDistanceValuePoint = 1.0;
        // private static bool interpolationEnabled; // Equivalent to smoothingEnabledMultiPoint
        // private static double maxStepSize = 5.0; // Equivalent to maxStepMultiPoint

        // --- New Multi Pull to point parameters ---
        private static bool multiPullToPointEnabled;
        private static List<DeviceManager.Vector3D> currentTargetPoints_RhinoCoords = new List<DeviceManager.Vector3D>();
        private static List<DeviceManager.Vector3D> currentSmoothedTargetPoints_RhinoCoords = new List<DeviceManager.Vector3D>();
        private static double maxForceValueMultiPoint = 1.0;
        private static double maxDistanceValueMultiPoint = 1.0;
        private static double falloffDistanceMultiPoint = 0.0; // New
        private static bool smoothingEnabledMultiPoint;
        private static double maxStepMultiPoint = 5.0;


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
        private static double vibrationCurveMaxAmplitude = 1.0;
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
        private static double pullCurveFalloffDistance = 0.0;
        private static bool pullCurveFade = false;
        private static int pullCurveMethod = 0;
        private static bool pullAlongEnabled = false;
        private static double travelingPointSpeed = 10.0;
        private static double travelingPointLengthPosition = 0.0;
        private static DateTime lastTravelingPointUpdateTime = DateTime.Now;
        private static bool travelingPointActive = false;
        private static double totalCurveLength = 0.0;
        private static double tangentForce = 1.0;

        // Vibration Point Handling (Multiple Points)
        private struct VibrationPointData
        {
            public DeviceManager.Vector3D Target;
            public DeviceManager.Vector3D Direction;
            public double Deadzone;
            public double MaxDistance;
            public double MaxAmplitude;
            public double Frequency;
            public bool InvertMapping;
            public bool UseSquareWave;
        }
        private static List<VibrationPointData> activeVibrationPoints = new List<VibrationPointData>();
        private static bool globalVibrationPointEnabled = false;
        private static double currentOverallMaxVibrationAmplitude = 3.0;


        public static double[] GetCurrentForce()
        {
            return (double[])currentTotalForce.Clone();
        }

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

        public static void SetTCPOffset(DeviceManager.Vector3D offset)
        {
            tcpOffset = offset;
        }

        public static void SetDirectForce(double[] force_deviceCoords, bool enable, bool useFilter = false)
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
                    forceFilter.Update(force_deviceCoords);
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
                currentDirectForce = force_deviceCoords;
                directForceEnabled = enable;
                filteredForceEnabled = false;
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

        public static void SetDampingParameters(bool enable, double coefficient, double derivativeCoefficient, DampingMethod method)
        {
            dampingEnabled = enable;
            dampingCoefficient = Math.Max(0, Math.Min(coefficient, 0.99));
            derivativeDampingCoefficient = Math.Max(0, Math.Min(derivativeCoefficient, 1.0));
            currentDampingMethod = method;
        }

        // --- Old SetPullToPoint (Single Point) - Commented out ---
        /*
        public static void SetPullToPoint(DeviceManager.Vector3D target_RhinoCoords, bool enable, double maxForce, double maxDistance, bool useInterpolation = false, double stepSize = 5.0)
        {
            if (!pullToPointEnabled && enable)
            {
                currentSmoothedTarget = target_RhinoCoords;
            }
            pullToPointEnabled = enable;
            targetPoint = target_RhinoCoords;
            maxForceValuePoint = maxForce;
            maxDistanceValuePoint = maxDistance;
            interpolationEnabled = useInterpolation;
            maxStepSize = stepSize;
        }
        */

        // --- Old CalculatePullToPointForce (Single Point) - Commented out ---
        /*
        private static double[] CalculatePullToPointForce(DeviceManager.Vector3D devicePos_RhinoCoords)
        {
            var targetToUse_RhinoCoords = interpolationEnabled ? currentSmoothedTarget : targetPoint;
            var dx_rhino = targetToUse_RhinoCoords.X - devicePos_RhinoCoords.X;
            var dy_rhino = targetToUse_RhinoCoords.Y - devicePos_RhinoCoords.Y;
            var dz_rhino = targetToUse_RhinoCoords.Z - devicePos_RhinoCoords.Z;
            var distance = Math.Sqrt(dx_rhino * dx_rhino + dy_rhino * dy_rhino + dz_rhino * dz_rhino);

            if (distance < 0.001) return new double[] { 0, 0, 0 };

            var scale = distance > maxDistanceValuePoint ? maxForceValuePoint : maxForceValuePoint * (distance / maxDistanceValuePoint);
            
            var fx_rhino = (dx_rhino / distance) * scale;
            var fy_rhino = (dy_rhino / distance) * scale;
            var fz_rhino = (dz_rhino / distance) * scale;

            return new double[] { -fx_rhino, fz_rhino, fy_rhino }; 
        }
        */

        // --- Old UpdateSmoothedTarget (Single Point) - Commented out ---
        /*
        private static void UpdateSmoothedTarget(DeviceManager.Vector3D currentPosition_RhinoCoords)
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
        */

        // --- NEW Multi-Point Pull Methods ---
        public static void SetMultiPullToPoints(
            List<DeviceManager.Vector3D> targets_RhinoCoords,
            bool enable,
            double maxForce,
            double maxDistance,
            double falloffDist, // New parameter
            bool useSmoothing,
            double stepSize)
        {
            multiPullToPointEnabled = enable;
            maxForceValueMultiPoint = Math.Max(0, maxForce);
            maxDistanceValueMultiPoint = Math.Max(0.001, maxDistance);
            falloffDistanceMultiPoint = Math.Max(0, falloffDist); // Store new parameter
            smoothingEnabledMultiPoint = useSmoothing;
            maxStepMultiPoint = Math.Max(0.1, stepSize);

            if (!enable || targets_RhinoCoords == null || targets_RhinoCoords.Count == 0)
            {
                currentTargetPoints_RhinoCoords.Clear();
                currentSmoothedTargetPoints_RhinoCoords.Clear();
                return;
            }

            bool listStructureChanged = currentTargetPoints_RhinoCoords.Count != targets_RhinoCoords.Count;

            // Always update raw targets with a copy
            currentTargetPoints_RhinoCoords = new List<DeviceManager.Vector3D>(targets_RhinoCoords);

            if (listStructureChanged || !smoothingEnabledMultiPoint)
            {
                // If list structure changed (e.g. point count) or smoothing is off,
                // smoothed targets are just a direct copy of raw targets.
                currentSmoothedTargetPoints_RhinoCoords = new List<DeviceManager.Vector3D>(currentTargetPoints_RhinoCoords);
            }
            else
            {
                // Smoothing is on, and point count is the same.
                // We assume the order corresponds. The UpdateSmoothedTargets will handle moving them.
                // If currentSmoothedTargetPoints_RhinoCoords was somehow desynced in count, fix it.
                if (currentSmoothedTargetPoints_RhinoCoords.Count != currentTargetPoints_RhinoCoords.Count)
                {
                    currentSmoothedTargetPoints_RhinoCoords = new List<DeviceManager.Vector3D>(currentTargetPoints_RhinoCoords);
                }
                // Otherwise, existing smoothed points will be updated towards new raw targets in UpdateSmoothedTargets.
            }
        }

        private static void UpdateSmoothedTargets()
        {
            if (!multiPullToPointEnabled || !smoothingEnabledMultiPoint ||
                currentTargetPoints_RhinoCoords.Count == 0 ||
                currentSmoothedTargetPoints_RhinoCoords.Count != currentTargetPoints_RhinoCoords.Count)
            {
                return;
            }

            for (int i = 0; i < currentTargetPoints_RhinoCoords.Count; i++)
            {
                DeviceManager.Vector3D rawTarget = currentTargetPoints_RhinoCoords[i];
                DeviceManager.Vector3D smoothedTarget = currentSmoothedTargetPoints_RhinoCoords[i];

                double dx = rawTarget.X - smoothedTarget.X;
                double dy = rawTarget.Y - smoothedTarget.Y;
                double dz = rawTarget.Z - smoothedTarget.Z;
                double distanceToRawTarget = Math.Sqrt(dx * dx + dy * dy + dz * dz);

                if (distanceToRawTarget < 0.001) // Effectively at the target
                {
                    if (!smoothedTarget.Equals(rawTarget)) // Only assign if different to avoid unnecessary list modification
                        currentSmoothedTargetPoints_RhinoCoords[i] = rawTarget;
                    continue;
                }

                double step = Math.Min(distanceToRawTarget, maxStepMultiPoint);
                double scaleFactor = step / distanceToRawTarget;

                currentSmoothedTargetPoints_RhinoCoords[i] = new DeviceManager.Vector3D(
                    smoothedTarget.X + dx * scaleFactor,
                    smoothedTarget.Y + dy * scaleFactor,
                    smoothedTarget.Z + dz * scaleFactor
                );
            }
        }

        private static double[] CalculateMultiPullToPointForce(DeviceManager.Vector3D devicePos_RhinoCoords)
        {
            if (!multiPullToPointEnabled || currentTargetPoints_RhinoCoords.Count == 0)
            {
                return new double[] { 0, 0, 0 };
            }

            double strongestForceMag = -1.0; // Use -1 to ensure any positive force is chosen first
            Vector3d dominantForceVec_Rhino = Vector3d.Zero;

            List<DeviceManager.Vector3D> pointsToConsider = (smoothingEnabledMultiPoint &&
                                                              currentSmoothedTargetPoints_RhinoCoords.Count == currentTargetPoints_RhinoCoords.Count) ?
                                                             currentSmoothedTargetPoints_RhinoCoords :
                                                             currentTargetPoints_RhinoCoords;

            foreach (var target_Rhino_Vec3D in pointsToConsider)
            {
                Point3d target_Rhino_Pt = new Point3d(target_Rhino_Vec3D.X, target_Rhino_Vec3D.Y, target_Rhino_Vec3D.Z);
                Point3d device_Rhino_Pt = new Point3d(devicePos_RhinoCoords.X, devicePos_RhinoCoords.Y, devicePos_RhinoCoords.Z);

                Vector3d vecToTarget_Rhino = target_Rhino_Pt - device_Rhino_Pt;
                double distance = vecToTarget_Rhino.Length;
                double currentForceMag = 0;

                if (distance < 0.001)
                {
                    // At the point, no force.
                }
                else if (falloffDistanceMultiPoint > 0.001) // Snapping with falloff enabled
                {
                    double totalAttractionRange = maxDistanceValueMultiPoint + falloffDistanceMultiPoint;
                    if (distance >= totalAttractionRange)
                    {
                        currentForceMag = 0;
                    }
                    else if (distance > maxDistanceValueMultiPoint) // In falloff zone
                    {
                        double falloffProgress = (distance - maxDistanceValueMultiPoint) / falloffDistanceMultiPoint;
                        currentForceMag = maxForceValueMultiPoint * (1.0 - falloffProgress);
                    }
                    else // In proportional zone
                    {
                        currentForceMag = maxForceValueMultiPoint * (distance / maxDistanceValueMultiPoint);
                    }
                }
                else // Original behavior (no falloff, or falloff <= 0.001)
                {
                    if (distance > maxDistanceValueMultiPoint)
                    {
                        currentForceMag = maxForceValueMultiPoint;
                    }
                    else // distance <= maxDistanceValueMultiPoint
                    {
                        currentForceMag = maxForceValueMultiPoint * (distance / maxDistanceValueMultiPoint);
                    }
                }

                currentForceMag = Math.Max(0, Math.Min(currentForceMag, maxForceValueMultiPoint));

                if (currentForceMag > strongestForceMag)
                {
                    strongestForceMag = currentForceMag;
                    if (distance > 0.001) vecToTarget_Rhino.Unitize();
                    dominantForceVec_Rhino = vecToTarget_Rhino * strongestForceMag;
                }
            }

            if (strongestForceMag <= 0.0) return new double[] { 0, 0, 0 };

            return new double[] { -dominantForceVec_Rhino.X, dominantForceVec_Rhino.Z, dominantForceVec_Rhino.Y };
        }


        public static void SetPullToPlane(DeviceManager.Vector3D origin_RhinoCoords, DeviceManager.Vector3D normal_RhinoCoords, bool enable, double maxForce, double maxDistance)
        {
            pullToPlaneEnabled = enable;
            planeOrigin = origin_RhinoCoords;
            double mag = Math.Sqrt(normal_RhinoCoords.X * normal_RhinoCoords.X + normal_RhinoCoords.Y * normal_RhinoCoords.Y + normal_RhinoCoords.Z * normal_RhinoCoords.Z);
            if (mag > 0.0001)
            {
                planeNormal = new DeviceManager.Vector3D(normal_RhinoCoords.X / mag, normal_RhinoCoords.Y / mag, normal_RhinoCoords.Z / mag);
            }
            else
            {
                planeNormal = new DeviceManager.Vector3D(0, 1, 0);
            }
            maxForceValuePlane = maxForce;
            maxDistanceValuePlane = maxDistance;
        }

        private static double[] CalculatePullToPlaneForce(DeviceManager.Vector3D devicePos_RhinoCoords)
        {
            double dx_rhino = devicePos_RhinoCoords.X - planeOrigin.X;
            double dy_rhino = devicePos_RhinoCoords.Y - planeOrigin.Y;
            double dz_rhino = devicePos_RhinoCoords.Z - planeOrigin.Z;

            double distance = dx_rhino * planeNormal.X + dy_rhino * planeNormal.Y + dz_rhino * planeNormal.Z;
            double absDistance = Math.Abs(distance);
            double forceMagnitude;

            if (absDistance > maxDistanceValuePlane) forceMagnitude = maxForceValuePlane;
            else forceMagnitude = (absDistance / maxDistanceValuePlane) * maxForceValuePlane;

            double directionSign = distance > 0 ? -1 : 1;

            double fx_rhino = planeNormal.X * directionSign * forceMagnitude;
            double fy_rhino = planeNormal.Y * directionSign * forceMagnitude;
            double fz_rhino = planeNormal.Z * directionSign * forceMagnitude;

            return new double[] { -fx_rhino, fz_rhino, fy_rhino };
        }

        public static void SetPlaneCollision(DeviceManager.Vector3D origin_RhinoCoords, DeviceManager.Vector3D normal_RhinoCoords, bool enable, double maxForce, double maxDistance)
        {
            planeCollisionEnabled = enable;
            collisionPlaneOrigin = origin_RhinoCoords;
            double mag = Math.Sqrt(normal_RhinoCoords.X * normal_RhinoCoords.X + normal_RhinoCoords.Y * normal_RhinoCoords.Y + normal_RhinoCoords.Z * normal_RhinoCoords.Z);
            if (mag > 0.0001)
            {
                collisionPlaneNormal = new DeviceManager.Vector3D(normal_RhinoCoords.X / mag, normal_RhinoCoords.Y / mag, normal_RhinoCoords.Z / mag);
            }
            else { collisionPlaneNormal = new DeviceManager.Vector3D(0, 1, 0); }
            maxForceValueCollision = maxForce;
            maxDistanceValueCollision = maxDistance;
        }

        private static double[] CalculatePlaneCollisionForce(DeviceManager.Vector3D devicePos_RhinoCoords)
        {
            double dx_rhino = devicePos_RhinoCoords.X - collisionPlaneOrigin.X;
            double dy_rhino = devicePos_RhinoCoords.Y - collisionPlaneOrigin.Y;
            double dz_rhino = devicePos_RhinoCoords.Z - collisionPlaneOrigin.Z;
            double distance = dx_rhino * collisionPlaneNormal.X + dy_rhino * collisionPlaneNormal.Y + dz_rhino * collisionPlaneNormal.Z;

            if (distance >= 0) return new double[] { 0, 0, 0 };

            double penetrationDepth = -distance;
            double forceMagnitude;

            if (penetrationDepth > maxDistanceValueCollision) forceMagnitude = maxForceValueCollision;
            else forceMagnitude = (penetrationDepth / maxDistanceValueCollision) * maxForceValueCollision;

            double fx_rhino = collisionPlaneNormal.X * forceMagnitude;
            double fy_rhino = collisionPlaneNormal.Y * forceMagnitude;
            double fz_rhino = collisionPlaneNormal.Z * forceMagnitude;
            return new double[] { -fx_rhino, fz_rhino, fy_rhino };
        }

        private static double[] ApplyDamping(double[] currentForce_deviceCoords, double timeDelta)
        {
            if (!dampingEnabled || (dampingCoefficient < 0.01 && derivativeDampingCoefficient < 0.01))
            {
                previousForce = (double[])lastAppliedForce.Clone();
                lastAppliedForce = (double[])currentForce_deviceCoords.Clone();
                return currentForce_deviceCoords;
            }
            double[] dampedForce = new double[3];
            switch (currentDampingMethod)
            {
                case DampingMethod.ExponentialSmoothing:
                    double alpha = 1.0 - dampingCoefficient;
                    for (int i = 0; i < 3; i++) dampedForce[i] = alpha * currentForce_deviceCoords[i] + dampingCoefficient * lastAppliedForce[i];
                    break;
                case DampingMethod.ForceDerivative:
                    for (int i = 0; i < 3; i++)
                    {
                        double derivative = (currentForce_deviceCoords[i] - previousForce[i]) / Math.Max(0.001, timeDelta);
                        dampedForce[i] = currentForce_deviceCoords[i] - (derivativeDampingCoefficient * derivative);
                    }
                    break;
                case DampingMethod.Both:
                    double alphaBoth = 1.0 - dampingCoefficient;
                    double[] smoothedForce = new double[3];
                    for (int i = 0; i < 3; i++) smoothedForce[i] = alphaBoth * currentForce_deviceCoords[i] + dampingCoefficient * lastAppliedForce[i];
                    for (int i = 0; i < 3; i++)
                    {
                        double derivative = (smoothedForce[i] - previousForce[i]) / Math.Max(0.001, timeDelta);
                        dampedForce[i] = smoothedForce[i] - (derivativeDampingCoefficient * derivative);
                    }
                    break;
                default:
                    Array.Copy(currentForce_deviceCoords, dampedForce, 3);
                    break;
            }
            previousForce = (double[])lastAppliedForce.Clone();
            lastAppliedForce = (double[])dampedForce.Clone();
            return dampedForce;
        }

        public static void ClearVibratePoints()
        {
            activeVibrationPoints.Clear();
            globalVibrationPointEnabled = false;
        }

        public static void SetVibratePoints(
            List<DeviceManager.Vector3D> deviceSpaceTargets,
            DeviceManager.Vector3D deviceSpaceDirection,
            bool enable,
            double deadzone,
            double maxDistance,
            double overallMaxAmplitude,
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
                normalizedDeviceDirection = new DeviceManager.Vector3D(0, 0, 1);
            }

            double clampedDeadzone = Math.Max(0.0, deadzone);
            double clampedMaxDistance = Math.Max(clampedDeadzone + 0.001, maxDistance);
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
                    MaxAmplitude = currentOverallMaxVibrationAmplitude,
                    Frequency = clampedFrequency,
                    InvertMapping = invertMapping,
                    UseSquareWave = useSquareWave
                });
            }
        }

        private static double[] CalculateCombinedVibrationForce(DeviceManager.Vector3D currentDevicePos_native)
        {
            double[] summedPotentialForceDirection_native = new double[] { 0, 0, 0 };

            if (!globalVibrationPointEnabled || activeVibrationPoints.Count == 0)
            {
                return summedPotentialForceDirection_native;
            }

            foreach (var pointData in activeVibrationPoints)
            {
                double dx = currentDevicePos_native.X - pointData.Target.X;
                double dy = currentDevicePos_native.Y - pointData.Target.Y;
                double dz = currentDevicePos_native.Z - pointData.Target.Z;
                double distance = Math.Sqrt(dx * dx + dy * dy + dz * dz);

                double individualAmplitudeFactor = 0.0;
                double usableRange = pointData.MaxDistance - pointData.Deadzone;
                if (usableRange < 0.001) usableRange = 0.001;


                if (pointData.InvertMapping)
                {
                    if (distance >= pointData.MaxDistance) individualAmplitudeFactor = 0.0;
                    else if (distance < pointData.Deadzone) individualAmplitudeFactor = pointData.MaxAmplitude;
                    else individualAmplitudeFactor = (1.0 - (distance - pointData.Deadzone) / usableRange) * pointData.MaxAmplitude;
                }
                else
                {
                    if (distance >= pointData.MaxDistance) individualAmplitudeFactor = pointData.MaxAmplitude;
                    else if (distance < pointData.Deadzone) individualAmplitudeFactor = 0.0;
                    else individualAmplitudeFactor = ((distance - pointData.Deadzone) / usableRange) * pointData.MaxAmplitude;
                }
                individualAmplitudeFactor = Math.Max(0.0, Math.Min(individualAmplitudeFactor, pointData.MaxAmplitude));

                if (individualAmplitudeFactor < 0.0001) continue;

                summedPotentialForceDirection_native[0] += pointData.Direction.X * individualAmplitudeFactor;
                summedPotentialForceDirection_native[1] += pointData.Direction.Y * individualAmplitudeFactor;
                summedPotentialForceDirection_native[2] += pointData.Direction.Z * individualAmplitudeFactor;
            }

            double currentTotalPotentialMagnitude = Math.Sqrt(
                summedPotentialForceDirection_native[0] * summedPotentialForceDirection_native[0] +
                summedPotentialForceDirection_native[1] * summedPotentialForceDirection_native[1] +
                summedPotentialForceDirection_native[2] * summedPotentialForceDirection_native[2]
            );

            double capScaleFactor = 1.0;
            if (currentTotalPotentialMagnitude > currentOverallMaxVibrationAmplitude && currentTotalPotentialMagnitude > 0.0001)
            {
                capScaleFactor = currentOverallMaxVibrationAmplitude / currentTotalPotentialMagnitude;
            }

            double representativeFrequency = 100.0;
            bool representativeSquareWave = false;
            if (activeVibrationPoints.Count > 0)
            {
                representativeFrequency = activeVibrationPoints[0].Frequency;
                representativeSquareWave = activeVibrationPoints[0].UseSquareWave;
            }
            double oscillation = Vibration.GetVibrationMultiplier(representativeFrequency, representativeSquareWave);

            double[] finalCombinedForce_native = new double[3];
            finalCombinedForce_native[0] = summedPotentialForceDirection_native[0] * capScaleFactor * oscillation;
            finalCombinedForce_native[1] = summedPotentialForceDirection_native[1] * capScaleFactor * oscillation;
            finalCombinedForce_native[2] = summedPotentialForceDirection_native[2] * capScaleFactor * oscillation;

            return finalCombinedForce_native;
        }

        public static void SetPullToCurve(Curve curve_RhinoCoords, bool enable, double maxForce, double maxDistance,
                                        double falloffDist, bool fade, int method, double tangentForceValue,
                                        bool pullAlong, double speed, bool resetInput)
        {
            bool previousEnableState = pullToCurveEnabled;
            Curve previousCurve = pullToCurve;

            pullToCurveEnabled = enable;

            if (enable)
            {
                if (curve_RhinoCoords == null)
                {
                    pullToCurveEnabled = false;
                    pullToCurve = null;
                    totalCurveLength = 0;
                }
                else if (pullToCurve != curve_RhinoCoords || (pullToCurve != null && curve_RhinoCoords != null && pullToCurve.ToNurbsCurve().Points.Count != curve_RhinoCoords.ToNurbsCurve().Points.Count)) // Basic check for curve change, more robust needed for content
                {
                    pullToCurve = curve_RhinoCoords; // Should be a copy from GH component
                    totalCurveLength = pullToCurve.GetLength();
                    ResetTravelingPoint();
                }
                else if (!previousEnableState)
                {
                    lastTravelingPointUpdateTime = DateTime.Now;
                }
            }

            maxForceValueCurve = maxForce;
            maxDistanceValueCurve = Math.Max(0.001, maxDistance);
            pullCurveFalloffDistance = Math.Max(0.0, falloffDist);
            pullCurveFade = fade;
            pullCurveMethod = method;
            tangentForce = tangentForceValue;
            pullAlongEnabled = pullAlong;
            travelingPointSpeed = speed;

            if (resetInput)
            {
                ResetTravelingPoint();
            }
        }

        private static double[] CalculatePullToCurveForce(DeviceManager.Vector3D devicePos_RhinoCoords)
        {
            if (!pullToCurveEnabled || pullToCurve == null)
            {
                travelingPointActive = false;
                return new double[] { 0, 0, 0 };
            }

            Point3d rhinoDevicePt = new Point3d(devicePos_RhinoCoords.X, devicePos_RhinoCoords.Y, devicePos_RhinoCoords.Z);
            double curveParam;
            if (!pullToCurve.ClosestPoint(rhinoDevicePt, out curveParam))
            {
                return new double[] { 0, 0, 0 };
            }

            Point3d closestPointOnCurve_Rhino = pullToCurve.PointAt(curveParam);
            Vector3d vectorToCurve_Rhino = closestPointOnCurve_Rhino - rhinoDevicePt;
            double distanceToCurve = vectorToCurve_Rhino.Length;

            double currentEffectiveMaxForce = maxForceValueCurve;

            if (pullCurveFade && totalCurveLength > 0.001)
            {
                double lengthAtParam = pullToCurve.GetLength(new Interval(pullToCurve.Domain.Min, curveParam));
                double progress = Math.Max(0.0, Math.Min(1.0, lengthAtParam / totalCurveLength));
                currentEffectiveMaxForce *= progress;
            }

            double pullMagnitude = 0;
            if (distanceToCurve < 0.001)
            {
                pullMagnitude = 0;
            }
            else if (pullCurveFalloffDistance > 0.001)
            {
                double totalAttractionRange = maxDistanceValueCurve + pullCurveFalloffDistance;
                if (distanceToCurve >= totalAttractionRange)
                {
                    pullMagnitude = 0;
                }
                else if (distanceToCurve > maxDistanceValueCurve)
                {
                    double falloffProgress = (distanceToCurve - maxDistanceValueCurve) / pullCurveFalloffDistance;
                    pullMagnitude = currentEffectiveMaxForce * (1.0 - falloffProgress);
                }
                else
                {
                    pullMagnitude = currentEffectiveMaxForce * (distanceToCurve / maxDistanceValueCurve);
                }
            }
            else
            {
                if (distanceToCurve > maxDistanceValueCurve)
                {
                    pullMagnitude = currentEffectiveMaxForce;
                }
                else
                {
                    pullMagnitude = currentEffectiveMaxForce * (distanceToCurve / maxDistanceValueCurve);
                }
            }

            pullMagnitude = Math.Max(0, Math.Min(pullMagnitude, currentEffectiveMaxForce));

            Vector3d totalForceVector_Rhino = Vector3d.Zero;
            if (pullMagnitude > 0.001 && distanceToCurve > 0.001)
            {
                vectorToCurve_Rhino.Unitize();
                totalForceVector_Rhino = vectorToCurve_Rhino * pullMagnitude;
            }

            if (pullAlongEnabled && pullMagnitude > 0.001)
            {
                travelingPointActive = true;
                UpdateTravelingPointPosition();

                Vector3d alongCurveDirection_Rhino = Vector3d.Zero;
                double alongCurveForceMagnitude = 0;

                if (pullCurveMethod == 0)
                {
                    alongCurveDirection_Rhino = pullToCurve.TangentAt(curveParam);
                    alongCurveForceMagnitude = tangentForce * currentEffectiveMaxForce;
                }
                else if (pullCurveMethod == 1)
                {
                    if (totalCurveLength > 0.001)
                    {
                        double travelingParam = FindParameterAtLength(pullToCurve, travelingPointLengthPosition);
                        Point3d targetPtOnCurve_Rhino = pullToCurve.PointAt(travelingParam);
                        alongCurveDirection_Rhino = targetPtOnCurve_Rhino - closestPointOnCurve_Rhino;
                        alongCurveForceMagnitude = tangentForce;
                    }
                }

                if (alongCurveDirection_Rhino.SquareLength > 0.00001 && alongCurveForceMagnitude > 0.001)
                {
                    alongCurveDirection_Rhino.Unitize();
                    totalForceVector_Rhino += alongCurveDirection_Rhino * alongCurveForceMagnitude;
                }
            }
            else
            {
                travelingPointActive = false;
            }

            return new double[] { -totalForceVector_Rhino.X, totalForceVector_Rhino.Z, totalForceVector_Rhino.Y };
        }

        private static void ResetTravelingPoint()
        {
            travelingPointLengthPosition = 0.0;
            lastTravelingPointUpdateTime = DateTime.Now;
            travelingPointActive = false;
        }

        private static void UpdateTravelingPointPosition()
        {
            if (!pullToCurveEnabled || !pullAlongEnabled || pullCurveMethod != 1 || !travelingPointActive || pullToCurve == null || totalCurveLength < 0.001)
            {
                return;
            }

            DateTime currentTime = DateTime.Now;
            double timeDelta = (currentTime - lastTravelingPointUpdateTime).TotalSeconds;
            lastTravelingPointUpdateTime = currentTime;

            if (timeDelta <= 0) return;

            double distanceToTravel = travelingPointSpeed * timeDelta;
            travelingPointLengthPosition += distanceToTravel;

            if (pullToCurve.IsClosed)
            {
                travelingPointLengthPosition = (travelingPointLengthPosition % totalCurveLength + totalCurveLength) % totalCurveLength;
            }
            else
            {
                travelingPointLengthPosition = Math.Max(0, Math.Min(travelingPointLengthPosition, totalCurveLength));
            }
        }

        private static double FindParameterAtLength(Curve curve, double targetLength)
        {
            if (curve == null) return 0.0;
            double curveTotalLength = curve.GetLength();
            if (curveTotalLength < 0.001) return curve.Domain.Min;

            double clampedLength = targetLength;
            if (curve.IsClosed)
            {
                clampedLength = (targetLength % curveTotalLength + curveTotalLength) % curveTotalLength;
            }
            else
            {
                clampedLength = Math.Max(0, Math.Min(targetLength, curveTotalLength));
            }

            double t;
            // Use Curve.LengthParameter which finds parameter at a given length from start of curve.
            if (curve.LengthParameter(clampedLength, out t))
            {
                return t;
            }

            // Fallback for safety / if LengthParameter fails (shouldn't for valid inputs)
            if (clampedLength <= 0.0) return curve.Domain.Min;
            if (clampedLength >= curveTotalLength && !curve.IsClosed) return curve.Domain.Max;

            return curve.Domain.Min;
        }

        public static void SetVibrateCurve(Curve curve_RhinoCoords, DeviceManager.Vector3D direction_RhinoCoords, bool enable, double deadzone, double maxDistance, double maxAmplitude, double frequency, bool invertMapping, bool useSquareWave)
        {
            vibrationCurveEnabled = enable;
            vibrationCurve = curve_RhinoCoords;
            vibrationCurveDirection = direction_RhinoCoords;
            vibrationCurveDeadzone = deadzone;
            vibrationCurveMaxDistance = maxDistance;
            vibrationCurveMaxAmplitude = maxAmplitude;
            vibrationCurveFrequency = frequency;
            vibrationCurveInvertMapping = invertMapping;
            vibrationCurveUseSquareWave = useSquareWave;
        }

        private static double[] CalculateVibrationCurveForce(DeviceManager.Vector3D devicePos_RhinoCoords)
        {
            if (!vibrationCurveEnabled || vibrationCurve == null)
            {
                vibrationCurveAmplitude = 0; vibrationCurveDistance = -1; return new double[3];
            }
            curveCalcStopwatch.Restart();

            Point3d rhinoDevicePt = new Point3d(devicePos_RhinoCoords.X, devicePos_RhinoCoords.Y, devicePos_RhinoCoords.Z);
            double curveParam;
            vibrationCurve.ClosestPoint(rhinoDevicePt, out curveParam);
            Point3d closestPt = vibrationCurve.PointAt(curveParam);

            vibrationCurveDistance = rhinoDevicePt.DistanceTo(closestPt);

            double amplitudeFactor = 0.0;
            double usableRange = vibrationCurveMaxDistance - vibrationCurveDeadzone;
            if (usableRange < 0.001) usableRange = 0.001;

            if (vibrationCurveInvertMapping)
            {
                if (vibrationCurveDistance >= vibrationCurveMaxDistance) amplitudeFactor = 0.0;
                else if (vibrationCurveDistance < vibrationCurveDeadzone) amplitudeFactor = 1.0;
                else amplitudeFactor = 1.0 - ((vibrationCurveDistance - vibrationCurveDeadzone) / usableRange);
            }
            else
            {
                if (vibrationCurveDistance >= vibrationCurveMaxDistance) amplitudeFactor = 1.0;
                else if (vibrationCurveDistance < vibrationCurveDeadzone) amplitudeFactor = 0.0;
                else amplitudeFactor = (vibrationCurveDistance - vibrationCurveDeadzone) / usableRange;
            }
            amplitudeFactor = Math.Max(0.0, Math.Min(1.0, amplitudeFactor));
            vibrationCurveAmplitude = vibrationCurveMaxAmplitude * amplitudeFactor;

            double oscillation = Vibration.GetVibrationMultiplier(vibrationCurveFrequency, vibrationCurveUseSquareWave);
            double currentVibMag = vibrationCurveAmplitude * oscillation;

            Vector3d vibDir_Rhino = new Vector3d(vibrationCurveDirection.X, vibrationCurveDirection.Y, vibrationCurveDirection.Z);
            if (!vibDir_Rhino.Unitize()) vibDir_Rhino = new Vector3d(0, 0, 1);

            Vector3d force_Rhino = vibDir_Rhino * currentVibMag;

            curveCalcStopwatch.Stop();
            vibrationCurveCalcTime = curveCalcStopwatch.Elapsed.TotalMilliseconds;

            return new double[] { -force_Rhino.X, force_Rhino.Z, force_Rhino.Y };
        }

        public static void UpdateForces()
        {
            DateTime currentTime = DateTime.Now;
            double[] totalForceForFrame_DeviceCoords = new double[3];

            var transformMatrix = new double[16];
            HDdll.hdGetDoublev(HDdll.HD_CURRENT_TRANSFORM, transformMatrix);

            var currentDevicePos_RhinoCoords = new DeviceManager.Vector3D(
                -transformMatrix[12],
                 transformMatrix[14],
                 transformMatrix[13]
            );

            var currentDevicePos_NativeDeviceCoords = new DeviceManager.Vector3D(
                transformMatrix[12],
                transformMatrix[13],
                transformMatrix[14]
            );

            if (tcpOffset.X != 0 || tcpOffset.Y != 0 || tcpOffset.Z != 0)
            {
                var devXaxis_inRhino = new DeviceManager.Vector3D(-transformMatrix[0], transformMatrix[2], transformMatrix[1]);
                var devYaxis_inRhino = new DeviceManager.Vector3D(-transformMatrix[4], transformMatrix[6], transformMatrix[5]);
                var devZaxis_inRhino = new DeviceManager.Vector3D(-transformMatrix[8], transformMatrix[10], transformMatrix[9]);

                currentDevicePos_RhinoCoords.X += tcpOffset.X * devXaxis_inRhino.X + tcpOffset.Y * devYaxis_inRhino.X + tcpOffset.Z * devZaxis_inRhino.X;
                currentDevicePos_RhinoCoords.Y += tcpOffset.X * devXaxis_inRhino.Y + tcpOffset.Y * devYaxis_inRhino.Y + tcpOffset.Z * devZaxis_inRhino.Y;
                currentDevicePos_RhinoCoords.Z += tcpOffset.X * devXaxis_inRhino.Z + tcpOffset.Y * devYaxis_inRhino.Z + tcpOffset.Z * devZaxis_inRhino.Z;
            }

            if (viscousDampingEnabled)
            {
                var f = CalculateViscousForce(); for (int i = 0; i < 3; i++) totalForceForFrame_DeviceCoords[i] += f[i];
            }
            if (planeCollisionEnabled)
            {
                var f = CalculatePlaneCollisionForce(currentDevicePos_RhinoCoords); for (int i = 0; i < 3; i++) totalForceForFrame_DeviceCoords[i] += f[i];
            }
            if (globalVibrationPointEnabled && activeVibrationPoints.Count > 0)
            {
                var f = CalculateCombinedVibrationForce(currentDevicePos_NativeDeviceCoords); for (int i = 0; i < 3; i++) totalForceForFrame_DeviceCoords[i] += f[i];
            }
            if (vibrationCurveEnabled && vibrationCurve != null)
            {
                var f = CalculateVibrationCurveForce(currentDevicePos_RhinoCoords); for (int i = 0; i < 3; i++) totalForceForFrame_DeviceCoords[i] += f[i];
            }

            // --- Updated PullToPoint Logic ---
            if (multiPullToPointEnabled)
            {
                if (smoothingEnabledMultiPoint) UpdateSmoothedTargets(); // Update smoothed positions if enabled
                var f = CalculateMultiPullToPointForce(currentDevicePos_RhinoCoords);
                for (int i = 0; i < 3; i++) totalForceForFrame_DeviceCoords[i] += f[i];
            }
            // --- End Updated PullToPoint Logic ---

            if (pullToCurveEnabled && pullToCurve != null)
            {
                var f = CalculatePullToCurveForce(currentDevicePos_RhinoCoords); for (int i = 0; i < 3; i++) totalForceForFrame_DeviceCoords[i] += f[i];
            }
            if (pullToPlaneEnabled)
            {
                var f = CalculatePullToPlaneForce(currentDevicePos_RhinoCoords); for (int i = 0; i < 3; i++) totalForceForFrame_DeviceCoords[i] += f[i];
            }
            if (filteredForceEnabled && forceFilter != null)
            {
                for (int i = 0; i < 3; i++) totalForceForFrame_DeviceCoords[i] += lastFilteredForce[i];
            }
            else if (directForceEnabled)
            {
                for (int i = 0; i < 3; i++) totalForceForFrame_DeviceCoords[i] += currentDirectForce[i];
            }

            try
            {
                var ucManagerType = Type.GetType("ghoh.UCManager");
                if (ucManagerType != null)
                {
                    var isConnectedProp = ucManagerType.GetProperty("IsConnected");
                    var forceEnabledProp = ucManagerType.GetProperty("ForceEnabled");
                    var getMappedForceMethod = ucManagerType.GetMethod("GetMappedForceValue");

                    if (isConnectedProp != null && forceEnabledProp != null && getMappedForceMethod != null &&
                        (bool)isConnectedProp.GetValue(null) && (bool)forceEnabledProp.GetValue(null))
                    {
                        var devZaxis_inRhino = new DeviceManager.Vector3D(-transformMatrix[8], transformMatrix[10], transformMatrix[9]);
                        double rawMCVal = (double)getMappedForceMethod.Invoke(null, null);

                        if (Math.Abs(rawMCVal) > 0.001)
                        {
                            double fx_rhino = devZaxis_inRhino.X * rawMCVal;
                            double fy_rhino = devZaxis_inRhino.Y * rawMCVal;
                            double fz_rhino = devZaxis_inRhino.Z * rawMCVal;

                            double[] mcForce_deviceCoords = new double[3] { -fx_rhino, fz_rhino, fy_rhino };

                            double timeDelta = (currentTime - lastForceUpdateTime).TotalSeconds;
                            double[] dampedMCForce = ApplyDamping(mcForce_deviceCoords, timeDelta);
                            for (int i = 0; i < 3; i++) totalForceForFrame_DeviceCoords[i] += dampedMCForce[i];
                        }
                    }
                }
            }
            catch (Exception) { /* Silently ignore */ }
            lastForceUpdateTime = currentTime;

            currentTotalForce = (double[])totalForceForFrame_DeviceCoords.Clone();
            HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, totalForceForFrame_DeviceCoords);
        }

        public static void Reset()
        {
            directForceEnabled = false;

            // Reset new multi-point pull parameters
            multiPullToPointEnabled = false;
            currentTargetPoints_RhinoCoords.Clear();
            currentSmoothedTargetPoints_RhinoCoords.Clear();
            maxForceValueMultiPoint = 1.0;
            maxDistanceValueMultiPoint = 1.0;
            falloffDistanceMultiPoint = 0.0;
            smoothingEnabledMultiPoint = false;
            maxStepMultiPoint = 5.0;

            pullToPlaneEnabled = false;
            filteredForceEnabled = false;
            dampingEnabled = false;
            viscousDampingEnabled = false;
            planeCollisionEnabled = false;

            vibrationCurveEnabled = false;
            vibrationCurve = null;

            pullToCurveEnabled = false;
            pullToCurve = null;
            pullCurveFalloffDistance = 0.0;
            pullCurveFade = false;
            pullCurveMethod = 0;
            pullAlongEnabled = false;
            travelingPointSpeed = 10.0;
            ResetTravelingPoint();
            totalCurveLength = 0.0;
            tangentForce = 1.0;

            tcpOffset = new DeviceManager.Vector3D(0, 0, 0);
            ClearVibratePoints();
            currentOverallMaxVibrationAmplitude = 3.0;

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