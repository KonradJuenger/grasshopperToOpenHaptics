// --- START OF FILE ForceManager.cs ---
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace ghoh
{
    public static class ForceManager
    {
        #region Existing Force Properties
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

        // Multi Pull to point parameters
        private static bool multiPullToPointEnabled;
        private static List<DeviceManager.Vector3D> currentTargetPoints_RhinoCoords = new List<DeviceManager.Vector3D>();
        private static List<DeviceManager.Vector3D> currentSmoothedTargetPoints_RhinoCoords = new List<DeviceManager.Vector3D>();
        private static double maxForceValueMultiPoint = 1.0;
        private static double maxDistanceValueMultiPoint = 1.0;
        private static double falloffDistanceMultiPoint = 0.0;
        private static bool smoothingEnabledMultiPoint;
        private static double maxStepMultiPoint = 5.0;

        // Pull to plane parameters
        private static bool pullToPlaneEnabled;
        private static DeviceManager.Vector3D planeOrigin_RhinoCoords;
        private static DeviceManager.Vector3D planeNormal_RhinoCoords;
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

        // Viscous damping parameters
        private static bool viscousDampingEnabled;
        private static double viscousGain = 0.5;
        private static double viscousMaxForce = 3.0;
        private static double velocityDeadband = 10.0;
        private static double deadbandSoftness = 5.0;
        private static double velocityFilterCoefficient = 0.9;
        private static int forceWindowSize = 10;
        private static double[] lastVelocity_Native = new double[3];
        private static double[] filteredVelocity_Native = new double[3];
        private static double[] lastViscousForce_Native = new double[3];
        private static DateTime lastViscousUpdateTime = DateTime.Now;
        private static VectorMovingAverageFilter viscousForceFilter;

        // Plane collision parameters
        private static bool planeCollisionEnabled;
        private static DeviceManager.Vector3D collisionPlaneOrigin_RhinoCoords;
        private static DeviceManager.Vector3D collisionPlaneNormal_RhinoCoords;
        private static double maxForceValueCollision = 1.0;
        private static double maxDistanceValueCollision = 1.0;

        // Curve-based vibration parameters (now with pulsing)
        public static bool vibrationCurveEnabled = false;
        public static Curve vibrationCurve_RhinoCoords = null;
        private static DeviceManager.Vector3D vibrationCurveDirection_RhinoCoords = new DeviceManager.Vector3D(0, 0, 1);
        private static double vibrationCurveDeadzone = 0.0;
        private static double vibrationCurveMaxDistance = 10.0;
        private static double vibrationCurveMaxAmplitude = 1.0;
        private static double vibrationCurveFrequency = 100.0;
        private static bool vibrationCurveInvertMapping = false;
        private static Vibration.VibrationMode vibrationCurveMode = Vibration.VibrationMode.Sine;
        private static double vibrationCurvePulseDuration = 0.1;
        private static double vibrationCurveMinPause = 0.1;
        private static double vibrationCurveMaxPause = 1.0;
        public static double vibrationCurveDistance = 0.0;
        public static double vibrationCurveAmplitude = 0.0;
        private static Stopwatch curveCalcStopwatch = new Stopwatch();
        public static double vibrationCurveCalcTime = 0.0;


        public static bool vibrationPointSystemEnabled = false;
        private static List<DeviceManager.Vector3D> vibrationPointTargets_RhinoCoords = new List<DeviceManager.Vector3D>();
        private static DeviceManager.Vector3D vibrationPointDirection_RhinoCoords = new DeviceManager.Vector3D(0, 0, 1);
        private static double vibrationPointDeadzone = 0.0;
        private static double vibrationPointMaxDistance = 10.0;
        private static double vibrationPointMaxAmplitude = 1.0;
        private static double vibrationPointFrequency = 100.0;
        private static bool vibrationPointInvertMapping = false;
        private static bool vibrationPointUseSquareWave = false;

        // Pull to curve parameters
        private static bool pullToCurveEnabled = false;
        private static Curve pullToCurve_RhinoCoords = null;
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
        private static double tangentForceMagnitudeFactor = 1.0;

        private static DateTime lastForceUpdateTime = DateTime.Now;

        #endregion Existing Force Properties

        // --- "Lazy String" / Pulled String parameters ---
        private static bool lazyStringEnabled = false;
        private static double stringLength = 20.0;
        private static double returnForce = 1.5;
        private static DeviceManager.Vector3D tooltipPosition_Native = new DeviceManager.Vector3D(0, 0, 0);
        private static bool isFirstLazyStringFrame = true;

        // --- "Dog on a Leash" parameters ---
        private static bool dogOnLeashEnabled = false;
        private static Curve dogLeashCurve_RhinoCoords = null;
        private static double leashLength = 20.0;
        private static double leashSpringRange = 20.0;
        private static double leashMaxForce = 1.5;
        private static double dogSpeed = 10.0;
        private static double dogCurrentLengthPosition = 0.0;
        private static double totalDogCurveLength = 0.0;
        private static DateTime lastDogLeashUpdateTime = DateTime.Now;
        public static Point3d DogWorldPosition_RhinoCoords { get; private set; } = Point3d.Unset;
        private static bool wasButton1PressedLastFrame = false; // NEW: State tracking for button press
        private static Point3d dogFreeSpacePosition_RhinoCoords = Point3d.Unset; // NEW: Position when dragging
        // --- End of New Parameters ---

        public static DeviceManager.Vector3D CurrentDeviceTCP_NativeCoords { get; private set; } = new DeviceManager.Vector3D();

        public static DeviceManager.Vector3D TooltipPosition_Native { get; private set; } = new DeviceManager.Vector3D();

        public static void SetLazyStringEffect(bool enable, double length, double force)
        {
            if (enable && !lazyStringEnabled)
            {
                isFirstLazyStringFrame = true;
            }
            lazyStringEnabled = enable;
            stringLength = Math.Max(0, length);
            returnForce = Math.Max(0, force);
        }

        public static void SetDogOnLeash(bool enable, Curve curve, double length, double springRange, double force, double speed, bool reset)
        {
            dogOnLeashEnabled = enable;

            if (!enable)
            {
                dogLeashCurve_RhinoCoords = null;
                DogWorldPosition_RhinoCoords = Point3d.Unset;
                return;
            }

            if (curve == null || !curve.IsValid)
            {
                dogOnLeashEnabled = false;
                dogLeashCurve_RhinoCoords = null;
                DogWorldPosition_RhinoCoords = Point3d.Unset;
                return;
            }

            leashLength = Math.Max(0, length);
            leashSpringRange = Math.Max(0.001, springRange);
            leashMaxForce = Math.Max(0, force);
            dogSpeed = Math.Max(0, speed);

            if (dogLeashCurve_RhinoCoords != curve || reset)
            {
                dogLeashCurve_RhinoCoords = curve;
                totalDogCurveLength = curve.GetLength();
                dogCurrentLengthPosition = 0.0;
                lastDogLeashUpdateTime = DateTime.Now;
                DogWorldPosition_RhinoCoords = dogLeashCurve_RhinoCoords.PointAtLength(0);
            }
        }

        public static double[] GetCurrentForce()
        {
            return (double[])currentTotalForce.Clone();
        }

        #region Existing Force Calculation Methods
        public static Vector3d SetViscousDamping(bool enable, double gain, double maxForce, double deadbandThreshold = 10.0, double softness = 5.0, double filterCoefficient = 0.9, int windowSize = 10)
        {
            viscousDampingEnabled = enable;
            viscousGain = Math.Max(0.0, gain);
            viscousMaxForce = Math.Max(0.0, maxForce);
            velocityDeadband = Math.Max(0.0, deadbandThreshold);
            deadbandSoftness = Math.Max(0.0, Math.Min(velocityDeadband, softness));
            velocityFilterCoefficient = Math.Max(0.0, Math.Min(0.99, filterCoefficient));
            forceWindowSize = Math.Max(1, windowSize);
            return new Vector3d(-lastViscousForce_Native[0], lastViscousForce_Native[2], lastViscousForce_Native[1]);
        }

        private static double[] CalculateViscousForce_Native()
        {
            if (!viscousDampingEnabled) return new double[] { 0, 0, 0 };

            if (viscousForceFilter == null) viscousForceFilter = new VectorMovingAverageFilter(forceWindowSize);
            else if (viscousForceFilter.GetWindowSize() != forceWindowSize) viscousForceFilter.SetWindowSize(forceWindowSize);

            DateTime currentTime = DateTime.Now;
            lastViscousUpdateTime = currentTime;

            var rawVelocity_native = new double[3];
            HDdll.hdGetDoublev(HDdll.HD_CURRENT_VELOCITY, rawVelocity_native);

            for (int i = 0; i < 3; i++)
            {
                filteredVelocity_Native[i] = velocityFilterCoefficient * filteredVelocity_Native[i] +
                                     (1 - velocityFilterCoefficient) * rawVelocity_native[i];
            }
            lastVelocity_Native = (double[])filteredVelocity_Native.Clone();

            double velMagnitude_native = Math.Sqrt(filteredVelocity_Native[0] * filteredVelocity_Native[0] +
                                           filteredVelocity_Native[1] * filteredVelocity_Native[1] +
                                           filteredVelocity_Native[2] * filteredVelocity_Native[2]);

            double[] rawViscousForce_native = new double[3];
            rawViscousForce_native[0] = -viscousGain * filteredVelocity_Native[0];
            rawViscousForce_native[1] = -viscousGain * filteredVelocity_Native[1];
            rawViscousForce_native[2] = -viscousGain * filteredVelocity_Native[2];

            double forceMag_native = Math.Sqrt(rawViscousForce_native[0] * rawViscousForce_native[0] +
                                       rawViscousForce_native[1] * rawViscousForce_native[1] +
                                       rawViscousForce_native[2] * rawViscousForce_native[2]);

            if (forceMag_native > viscousMaxForce && forceMag_native > 0.0001)
            {
                double scale = viscousMaxForce / forceMag_native;
                rawViscousForce_native[0] *= scale;
                rawViscousForce_native[1] *= scale;
                rawViscousForce_native[2] *= scale;
            }

            double[] smoothedForce_native = viscousForceFilter.AddSample(rawViscousForce_native);
            double[] finalForce_native = new double[3];
            double scaleFactor = 1.0;

            if (velMagnitude_native < velocityDeadband - deadbandSoftness) scaleFactor = 0.0;
            else if (velMagnitude_native < velocityDeadband)
            {
                if (deadbandSoftness > 0.001)
                {
                    double transitionPosition = velMagnitude_native - (velocityDeadband - deadbandSoftness);
                    scaleFactor = transitionPosition / deadbandSoftness;
                }
                else scaleFactor = (velMagnitude_native >= velocityDeadband) ? 1.0 : 0.0;
            }
            scaleFactor = Math.Max(0.0, Math.Min(1.0, scaleFactor));

            finalForce_native[0] = smoothedForce_native[0] * scaleFactor;
            finalForce_native[1] = smoothedForce_native[1] * scaleFactor;
            finalForce_native[2] = smoothedForce_native[2] * scaleFactor;

            lastViscousForce_Native = (double[])finalForce_native.Clone();
            return finalForce_native;
        }

        public static void SetTCPOffset(DeviceManager.Vector3D offset_deviceLocal)
        {
            tcpOffset = offset_deviceLocal;
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
                else forceFilter.Reset();
                filteredForceEnabled = enable;
                directForceEnabled = false;
            }
            else
            {
                currentDirectForce = (double[])force_deviceCoords.Clone();
                directForceEnabled = enable;
                filteredForceEnabled = false;
            }
        }

        public static void SetFilterParams(double q, double r)
        {
            processNoise = Math.Max(0.001, Math.Min(q, 1.0));
            measurementNoise = Math.Max(0.001, Math.Min(r, 1.0));
            if (forceFilter != null) forceFilter.SetNoiseParams(processNoise, measurementNoise);
        }

        public static void SetDampingParameters(bool enable, double coefficient, double derivativeCoeff, DampingMethod method)
        {
            dampingEnabled = enable;
            dampingCoefficient = Math.Max(0, Math.Min(coefficient, 0.99));
            derivativeDampingCoefficient = Math.Max(0, Math.Min(derivativeCoeff, 1.0));
            currentDampingMethod = method;
        }

        public static void SetMultiPullToPoints(
            List<DeviceManager.Vector3D> targets_Rhino, bool enable,
            double maxForce, double maxDist, double falloffDist, bool useSmoothing, double stepSize)
        {
            multiPullToPointEnabled = enable;
            maxForceValueMultiPoint = Math.Max(0, maxForce);
            maxDistanceValueMultiPoint = Math.Max(0.001, maxDist);
            falloffDistanceMultiPoint = Math.Max(0, falloffDist);
            smoothingEnabledMultiPoint = useSmoothing;
            maxStepMultiPoint = Math.Max(0.1, stepSize);

            if (!enable || targets_Rhino == null || targets_Rhino.Count == 0)
            {
                currentTargetPoints_RhinoCoords.Clear();
                currentSmoothedTargetPoints_RhinoCoords.Clear();
                return;
            }
            bool listStructureChanged = currentTargetPoints_RhinoCoords.Count != targets_Rhino.Count;
            currentTargetPoints_RhinoCoords = new List<DeviceManager.Vector3D>(targets_Rhino);
            if (listStructureChanged || !smoothingEnabledMultiPoint || currentSmoothedTargetPoints_RhinoCoords.Count != targets_Rhino.Count)
            {
                currentSmoothedTargetPoints_RhinoCoords = new List<DeviceManager.Vector3D>(currentTargetPoints_RhinoCoords);
            }
        }
        private static void UpdateSmoothedTargets()
        {
            if (!multiPullToPointEnabled || !smoothingEnabledMultiPoint ||
                currentTargetPoints_RhinoCoords.Count == 0 ||
                currentSmoothedTargetPoints_RhinoCoords.Count != currentTargetPoints_RhinoCoords.Count) return;

            for (int i = 0; i < currentTargetPoints_RhinoCoords.Count; i++)
            {
                DeviceManager.Vector3D rawTarget = currentTargetPoints_RhinoCoords[i];
                DeviceManager.Vector3D smoothedTarget = currentSmoothedTargetPoints_RhinoCoords[i];
                double dx = rawTarget.X - smoothedTarget.X;
                double dy = rawTarget.Y - smoothedTarget.Y;
                double dz = rawTarget.Z - smoothedTarget.Z;
                double distanceToRawTarget = Math.Sqrt(dx * dx + dy * dy + dz * dz);

                if (distanceToRawTarget < 0.001)
                {
                    if (!smoothedTarget.Equals(rawTarget)) currentSmoothedTargetPoints_RhinoCoords[i] = rawTarget;
                    continue;
                }
                double step = Math.Min(distanceToRawTarget, maxStepMultiPoint);
                double scaleFactor = step / distanceToRawTarget;
                currentSmoothedTargetPoints_RhinoCoords[i] = new DeviceManager.Vector3D(
                    smoothedTarget.X + dx * scaleFactor, smoothedTarget.Y + dy * scaleFactor, smoothedTarget.Z + dz * scaleFactor);
            }
        }
        private static double[] CalculateMultiPullToPointForce_Native(DeviceManager.Vector3D devicePos_Rhino)
        {
            if (!multiPullToPointEnabled || currentTargetPoints_RhinoCoords.Count == 0) return new double[] { 0, 0, 0 };

            double strongestForceMag_Rhino = -1.0;
            Vector3d dominantForceVec_Rhino = Vector3d.Zero;
            List<DeviceManager.Vector3D> pointsToConsider = (smoothingEnabledMultiPoint &&
                currentSmoothedTargetPoints_RhinoCoords.Count == currentTargetPoints_RhinoCoords.Count) ?
                currentSmoothedTargetPoints_RhinoCoords : currentTargetPoints_RhinoCoords;
            Point3d device_Rhino_Pt = new Point3d(devicePos_Rhino.X, devicePos_Rhino.Y, devicePos_Rhino.Z);

            foreach (var target_Rhino_Vec3D in pointsToConsider)
            {
                Point3d target_Rhino_Pt = new Point3d(target_Rhino_Vec3D.X, target_Rhino_Vec3D.Y, target_Rhino_Vec3D.Z);
                Vector3d vecToTarget_Rhino = target_Rhino_Pt - device_Rhino_Pt;
                double distance_Rhino = vecToTarget_Rhino.Length;
                double currentForceMag_Rhino = 0;

                if (distance_Rhino < 0.001) { /* on point */ }
                else if (falloffDistanceMultiPoint > 0.001)
                {
                    double totalAttractionRange = maxDistanceValueMultiPoint + falloffDistanceMultiPoint;
                    if (distance_Rhino >= totalAttractionRange) currentForceMag_Rhino = 0;
                    else if (distance_Rhino > maxDistanceValueMultiPoint)
                    {
                        double falloffProgress = (distance_Rhino - maxDistanceValueMultiPoint) / falloffDistanceMultiPoint;
                        currentForceMag_Rhino = maxForceValueMultiPoint * (1.0 - falloffProgress);
                    }
                    else currentForceMag_Rhino = maxForceValueMultiPoint * (distance_Rhino / maxDistanceValueMultiPoint);
                }
                else
                {
                    if (distance_Rhino > maxDistanceValueMultiPoint) currentForceMag_Rhino = maxForceValueMultiPoint;
                    else currentForceMag_Rhino = maxForceValueMultiPoint * (distance_Rhino / maxDistanceValueMultiPoint);
                }
                currentForceMag_Rhino = Math.Max(0, Math.Min(currentForceMag_Rhino, maxForceValueMultiPoint));
                if (currentForceMag_Rhino > strongestForceMag_Rhino)
                {
                    strongestForceMag_Rhino = currentForceMag_Rhino;
                    if (distance_Rhino > 0.001) vecToTarget_Rhino.Unitize();
                    dominantForceVec_Rhino = vecToTarget_Rhino * strongestForceMag_Rhino;
                }
            }
            if (strongestForceMag_Rhino <= 0.0) return new double[] { 0, 0, 0 };
            return new double[] { -dominantForceVec_Rhino.X, dominantForceVec_Rhino.Z, dominantForceVec_Rhino.Y };
        }
        public static void SetPullToPlane(DeviceManager.Vector3D origin_Rhino, DeviceManager.Vector3D normal_Rhino, bool enable, double maxForce, double maxDistance)
        {
            pullToPlaneEnabled = enable;
            planeOrigin_RhinoCoords = origin_Rhino;
            double mag = Math.Sqrt(normal_Rhino.X * normal_Rhino.X + normal_Rhino.Y * normal_Rhino.Y + normal_Rhino.Z * normal_Rhino.Z);
            if (mag > 0.0001) planeNormal_RhinoCoords = new DeviceManager.Vector3D(normal_Rhino.X / mag, normal_Rhino.Y / mag, normal_Rhino.Z / mag);
            else planeNormal_RhinoCoords = new DeviceManager.Vector3D(0, 1, 0);
            maxForceValuePlane = maxForce;
            maxDistanceValuePlane = Math.Max(0.001, maxDistance);
        }
        private static double[] CalculatePullToPlaneForce_Native(DeviceManager.Vector3D devicePos_Rhino)
        {
            if (!pullToPlaneEnabled) return new double[] { 0, 0, 0 };
            double dx_r = devicePos_Rhino.X - planeOrigin_RhinoCoords.X;
            double dy_r = devicePos_Rhino.Y - planeOrigin_RhinoCoords.Y;
            double dz_r = devicePos_Rhino.Z - planeOrigin_RhinoCoords.Z;
            double dist_r = dx_r * planeNormal_RhinoCoords.X + dy_r * planeNormal_RhinoCoords.Y + dz_r * planeNormal_RhinoCoords.Z;
            double absDist_r = Math.Abs(dist_r);
            double forceMag_r = (absDist_r > maxDistanceValuePlane) ? maxForceValuePlane : (absDist_r / maxDistanceValuePlane) * maxForceValuePlane;
            double dirSign = (dist_r > 0) ? -1.0 : 1.0;
            if (absDist_r < 0.0001) dirSign = 0;
            double fx_r = planeNormal_RhinoCoords.X * dirSign * forceMag_r;
            double fy_r = planeNormal_RhinoCoords.Y * dirSign * forceMag_r;
            double fz_r = planeNormal_RhinoCoords.Z * dirSign * forceMag_r;
            return new double[] { -fx_r, fz_r, fy_r };
        }
        public static void SetPlaneCollision(DeviceManager.Vector3D origin_Rhino, DeviceManager.Vector3D normal_Rhino, bool enable, double maxForce, double maxDistanceAsPenetration)
        {
            planeCollisionEnabled = enable;
            collisionPlaneOrigin_RhinoCoords = origin_Rhino;
            double mag = Math.Sqrt(normal_Rhino.X * normal_Rhino.X + normal_Rhino.Y * normal_Rhino.Y + normal_Rhino.Z * normal_Rhino.Z);
            if (mag > 0.0001) collisionPlaneNormal_RhinoCoords = new DeviceManager.Vector3D(normal_Rhino.X / mag, normal_Rhino.Y / mag, normal_Rhino.Z / mag);
            else collisionPlaneNormal_RhinoCoords = new DeviceManager.Vector3D(0, 1, 0);
            maxForceValueCollision = maxForce;
            maxDistanceValueCollision = Math.Max(0.001, maxDistanceAsPenetration);
        }
        private static double[] CalculatePlaneCollisionForce_Native(DeviceManager.Vector3D devicePos_Rhino)
        {
            if (!planeCollisionEnabled) return new double[] { 0, 0, 0 };
            double dx_r = devicePos_Rhino.X - collisionPlaneOrigin_RhinoCoords.X;
            double dy_r = devicePos_Rhino.Y - collisionPlaneOrigin_RhinoCoords.Y;
            double dz_r = devicePos_Rhino.Z - collisionPlaneOrigin_RhinoCoords.Z;
            double dist_r = dx_r * collisionPlaneNormal_RhinoCoords.X + dy_r * collisionPlaneNormal_RhinoCoords.Y + dz_r * collisionPlaneNormal_RhinoCoords.Z;
            if (dist_r >= 0) return new double[] { 0, 0, 0 };
            double penetration_r = -dist_r;
            double forceMag_r = (penetration_r > maxDistanceValueCollision) ? maxForceValueCollision : (penetration_r / maxDistanceValueCollision) * maxForceValueCollision;
            double fx_r = collisionPlaneNormal_RhinoCoords.X * forceMag_r;
            double fy_r = collisionPlaneNormal_RhinoCoords.Y * forceMag_r;
            double fz_r = collisionPlaneNormal_RhinoCoords.Z * forceMag_r;
            return new double[] { -fx_r, fz_r, fy_r };
        }
        public static void SetPullToCurve(
            Curve curve_Rhino, bool enable, double maxForce, double maxDistance,
            double falloffDist, bool fadeAtEnd, int method, double tangentForceFactor,
            bool pullAlong, double speed, bool resetTravelingPoint)
        {
            bool previousEnableState = pullToCurveEnabled;
            pullToCurveEnabled = enable;
            if (enable)
            {
                if (curve_Rhino == null || !curve_Rhino.IsValid)
                {
                    pullToCurveEnabled = false; pullToCurve_RhinoCoords = null; totalCurveLength = 0; ResetTravelingPointInternal();
                }
                else if (pullToCurve_RhinoCoords != curve_Rhino || (pullToCurve_RhinoCoords == null && curve_Rhino != null))
                {
                    pullToCurve_RhinoCoords = curve_Rhino; totalCurveLength = pullToCurve_RhinoCoords.GetLength(); ResetTravelingPointInternal();
                }
                else if (!previousEnableState && pullToCurve_RhinoCoords != null) lastTravelingPointUpdateTime = DateTime.Now;
            }
            maxForceValueCurve = maxForce; maxDistanceValueCurve = Math.Max(0.001, maxDistance);
            pullCurveFalloffDistance = Math.Max(0.0, falloffDist); pullCurveFade = fadeAtEnd;
            pullCurveMethod = method; tangentForceMagnitudeFactor = tangentForceFactor;
            pullAlongEnabled = pullAlong; travelingPointSpeed = speed;
            if (resetTravelingPoint) ResetTravelingPointInternal();
        }
        private static double[] CalculatePullToCurveForce_Native(DeviceManager.Vector3D devicePos_Rhino)
        {
            if (!pullToCurveEnabled || pullToCurve_RhinoCoords == null || !pullToCurve_RhinoCoords.IsValid)
            {
                travelingPointActive = false; return new double[] { 0, 0, 0 };
            }
            Point3d rhinoDevicePt = new Point3d(devicePos_Rhino.X, devicePos_Rhino.Y, devicePos_Rhino.Z);
            double curveParam;
            if (!pullToCurve_RhinoCoords.ClosestPoint(rhinoDevicePt, out curveParam))
            {
                travelingPointActive = false; return new double[] { 0, 0, 0 };
            }
            Point3d closestPt_r = pullToCurve_RhinoCoords.PointAt(curveParam);
            Vector3d vecToCurve_r = closestPt_r - rhinoDevicePt;
            double distToCurve_r = vecToCurve_r.Length;
            double effectiveMaxForce = maxForceValueCurve;
            if (pullCurveFade && totalCurveLength > 0.001)
            {
                double lenAtParam = pullToCurve_RhinoCoords.GetLength(new Interval(pullToCurve_RhinoCoords.Domain.Min, curveParam));
                effectiveMaxForce *= Math.Max(0.0, Math.Min(1.0, lenAtParam / totalCurveLength));
            }
            double pullMag_r = 0;
            if (distToCurve_r < 0.001) { /* on curve */ }
            else if (pullCurveFalloffDistance > 0.001)
            {
                double totalRange = maxDistanceValueCurve + pullCurveFalloffDistance;
                if (distToCurve_r >= totalRange) pullMag_r = 0;
                else if (distToCurve_r > maxDistanceValueCurve)
                    pullMag_r = effectiveMaxForce * (1.0 - (distToCurve_r - maxDistanceValueCurve) / pullCurveFalloffDistance);
                else pullMag_r = effectiveMaxForce * (distToCurve_r / maxDistanceValueCurve);
            }
            else
            {
                if (distToCurve_r > maxDistanceValueCurve) pullMag_r = effectiveMaxForce;
                else pullMag_r = effectiveMaxForce * (distToCurve_r / maxDistanceValueCurve);
            }
            pullMag_r = Math.Max(0, Math.Min(pullMag_r, effectiveMaxForce));
            Vector3d totalForce_r = Vector3d.Zero;
            if (pullMag_r > 0.001 && distToCurve_r > 0.001)
            {
                vecToCurve_r.Unitize(); totalForce_r = vecToCurve_r * pullMag_r;
            }
            if (pullAlongEnabled && pullMag_r > 0.0001 * effectiveMaxForce)
            {
                travelingPointActive = true; UpdateTravelingPointPositionInternal();
                Vector3d alongDir_r = Vector3d.Zero; double alongMag_r = 0;
                if (pullCurveMethod == 0)
                {
                    alongDir_r = pullToCurve_RhinoCoords.TangentAt(curveParam);
                    alongMag_r = tangentForceMagnitudeFactor * effectiveMaxForce;
                }
                else if (pullCurveMethod == 1 && totalCurveLength > 0.001)
                {
                    double travelParam = FindParameterAtLengthInternal(pullToCurve_RhinoCoords, travelingPointLengthPosition);
                    Point3d targetPt_r = pullToCurve_RhinoCoords.PointAt(travelParam);
                    alongDir_r = targetPt_r - closestPt_r;

                    double distToTarget = alongDir_r.Length;

                    if (distToTarget < 0.001)
                    {
                        alongMag_r = 0;
                    }
                    else
                    {
                        double forceScale = Math.Min(1.0, distToTarget / maxDistanceValueCurve);
                        alongMag_r = tangentForceMagnitudeFactor * forceScale;
                    }
                }
                if (alongDir_r.SquareLength > 1e-6 && alongMag_r > 0.001)
                {
                    alongDir_r.Unitize(); totalForce_r += alongDir_r * alongMag_r;
                }
            }
            else travelingPointActive = false;
            return new double[] { -totalForce_r.X, totalForce_r.Z, totalForce_r.Y };
        }
        private static void ResetTravelingPointInternal()
        {
            travelingPointLengthPosition = 0.0; lastTravelingPointUpdateTime = DateTime.Now; travelingPointActive = false;
        }
        private static void UpdateTravelingPointPositionInternal()
        {
            if (!pullToCurveEnabled || !pullAlongEnabled || pullCurveMethod != 1 || !travelingPointActive ||
                pullToCurve_RhinoCoords == null || totalCurveLength < 0.001) return;
            DateTime now = DateTime.Now; double dt = (now - lastTravelingPointUpdateTime).TotalSeconds;
            lastTravelingPointUpdateTime = now; if (dt <= 0) return;
            travelingPointLengthPosition += travelingPointSpeed * dt;
            if (pullToCurve_RhinoCoords.IsClosed) travelingPointLengthPosition = (travelingPointLengthPosition % totalCurveLength + totalCurveLength) % totalCurveLength;
            else travelingPointLengthPosition = Math.Max(0, Math.Min(travelingPointLengthPosition, totalCurveLength));
        }
        private static double FindParameterAtLengthInternal(Curve curve, double targetLength)
        {
            if (curve == null || !curve.IsValid) return 0.0;
            double curveLen = curve.GetLength(); if (curveLen < 0.001) return curve.Domain.Min;
            double clampedLen = targetLength;
            if (curve.IsClosed) clampedLen = (targetLength % curveLen + curveLen) % curveLen;
            else clampedLen = Math.Max(0, Math.Min(targetLength, curveLen));
            double t; if (curve.LengthParameter(clampedLen, out t)) return t;
            if (clampedLen <= 0.0) return curve.Domain.Min; if (clampedLen >= curveLen) return curve.IsClosed ? curve.Domain.Min : curve.Domain.Max;
            return curve.Domain.Min + (curve.Domain.Length * (clampedLen / curveLen));
        }

        public static void SetVibrateCurve(
            Curve curve_Rhino, DeviceManager.Vector3D direction_Rhino, bool enable,
            double deadzone, double maxDistance, double maxAmplitude, double frequency,
            bool invertMapping, Vibration.VibrationMode mode, double pulseDuration, double minPause, double maxPause)
        {
            bool previousState = vibrationCurveEnabled;
            vibrationCurveEnabled = enable;
            if (enable)
            {
                vibrationCurve_RhinoCoords = curve_Rhino;
                vibrationCurveDirection_RhinoCoords = direction_Rhino;
                vibrationCurveDeadzone = deadzone;
                vibrationCurveMaxDistance = maxDistance;
                vibrationCurveMaxAmplitude = maxAmplitude;
                vibrationCurveFrequency = frequency;
                vibrationCurveInvertMapping = invertMapping;
                vibrationCurveMode = mode;
                vibrationCurvePulseDuration = pulseDuration;
                vibrationCurveMinPause = minPause;
                vibrationCurveMaxPause = maxPause;
                if (!previousState || mode != vibrationCurveMode) Vibration.Reset();
            }
            else
            {
                vibrationCurve_RhinoCoords = null;
            }
        }

        private static double[] CalculateVibrationCurveForce_Native(DeviceManager.Vector3D devicePos_Rhino)
        {
            if (!vibrationCurveEnabled || vibrationCurve_RhinoCoords == null)
            {
                vibrationCurveAmplitude = 0; vibrationCurveDistance = -1; vibrationCurveCalcTime = 0; return new double[] { 0, 0, 0 };
            }

            curveCalcStopwatch.Restart();
            Point3d rhinoDevicePt = new Point3d(devicePos_Rhino.X, devicePos_Rhino.Y, devicePos_Rhino.Z);
            double curveParam;

            if (!vibrationCurve_RhinoCoords.IsValid || vibrationCurve_RhinoCoords.Domain.IsSingleton)
            {
                vibrationCurveAmplitude = 0; vibrationCurveDistance = -1; vibrationCurveCalcTime = 0;
                curveCalcStopwatch.Stop(); return new double[] { 0, 0, 0 };
            }

            vibrationCurve_RhinoCoords.ClosestPoint(rhinoDevicePt, out curveParam);
            Point3d closestPt_Rhino = vibrationCurve_RhinoCoords.PointAt(curveParam);
            vibrationCurveDistance = rhinoDevicePt.DistanceTo(closestPt_Rhino);

            double ampFactor = 0.0;
            double usableRange = vibrationCurveMaxDistance - vibrationCurveDeadzone;
            if (usableRange < 0.001) usableRange = 0.001;

            if (vibrationCurveInvertMapping)
            {
                if (vibrationCurveDistance >= vibrationCurveMaxDistance) ampFactor = 0.0;
                else if (vibrationCurveDistance < vibrationCurveDeadzone) ampFactor = 1.0;
                else ampFactor = 1.0 - ((vibrationCurveDistance - vibrationCurveDeadzone) / usableRange);
            }
            else
            {
                if (vibrationCurveDistance >= vibrationCurveMaxDistance) ampFactor = 1.0;
                else if (vibrationCurveDistance < vibrationCurveDeadzone) ampFactor = 0.0;
                else ampFactor = (vibrationCurveDistance - vibrationCurveDeadzone) / usableRange;
            }
            ampFactor = Math.Max(0.0, Math.Min(1.0, ampFactor));

            vibrationCurveAmplitude = vibrationCurveMaxAmplitude * ampFactor;

            if (vibrationCurveMode != Vibration.VibrationMode.Pulse && vibrationCurveAmplitude < 0.0001)
            {
                curveCalcStopwatch.Stop();
                vibrationCurveCalcTime = curveCalcStopwatch.Elapsed.TotalMilliseconds;
                return new double[] { 0, 0, 0 };
            }

            if (vibrationCurveMode != Vibration.VibrationMode.Pulse && ampFactor < 0.0001)
            {
                vibrationCurveAmplitude = 0;
                curveCalcStopwatch.Stop();
                vibrationCurveCalcTime = curveCalcStopwatch.Elapsed.TotalMilliseconds;
                return new double[] { 0, 0, 0 };
            }

            double currentPause = vibrationCurveMinPause + (vibrationCurveMaxPause - vibrationCurveMinPause) * (1.0 - ampFactor);

            double osc = Vibration.GetVibrationMultiplier(
                vibrationCurveMode,
                vibrationCurveFrequency,
                vibrationCurvePulseDuration,
                currentPause
            );

            double effectiveAmplitude;
            if (vibrationCurveMode == Vibration.VibrationMode.Pulse)
            {
                effectiveAmplitude = vibrationCurveMaxAmplitude;
            }
            else
            {
                effectiveAmplitude = vibrationCurveAmplitude;
            }

            double vibMag = effectiveAmplitude * osc;

            Vector3d forceVec_Rhino = new Vector3d(
                vibrationCurveDirection_RhinoCoords.X,
                vibrationCurveDirection_RhinoCoords.Y,
                vibrationCurveDirection_RhinoCoords.Z
            );
            forceVec_Rhino.Unitize();
            forceVec_Rhino *= vibMag;

            curveCalcStopwatch.Stop();
            vibrationCurveCalcTime = curveCalcStopwatch.Elapsed.TotalMilliseconds;

            return new double[] { -forceVec_Rhino.X, forceVec_Rhino.Z, forceVec_Rhino.Y };
        }

        public static void SetVibratePoints(
            List<DeviceManager.Vector3D> targets_Rhino, DeviceManager.Vector3D direction_Rhino, bool enable,
            double deadzone, double maxDistance, double maxAmplitude, double frequency,
            bool invertMapping, bool useSquareWave)
        {
            bool previousState = vibrationPointSystemEnabled;
            vibrationPointSystemEnabled = enable;
            if (enable)
            {
                vibrationPointTargets_RhinoCoords = new List<DeviceManager.Vector3D>(targets_Rhino);
                vibrationPointDirection_RhinoCoords = direction_Rhino;
                vibrationPointDeadzone = deadzone; vibrationPointMaxDistance = maxDistance;
                vibrationPointMaxAmplitude = maxAmplitude; vibrationPointFrequency = frequency;
                vibrationPointInvertMapping = invertMapping; vibrationPointUseSquareWave = useSquareWave;
                if (!previousState) Vibration.Reset();
            }
            else vibrationPointTargets_RhinoCoords.Clear();
        }

        private static double[] CalculateVibrationPointForce_Native(DeviceManager.Vector3D devicePos_Rhino)
        {
            if (!vibrationPointSystemEnabled || vibrationPointTargets_RhinoCoords == null || vibrationPointTargets_RhinoCoords.Count == 0)
                return new double[] { 0, 0, 0 };

            double effectiveAmpFactor = vibrationPointInvertMapping ? 0.0 : 1.0;
            bool considered = false;

            for (int i = 0; i < vibrationPointTargets_RhinoCoords.Count; i++)
            {
                DeviceManager.Vector3D target_rhino_dm = vibrationPointTargets_RhinoCoords[i];
                double dx = devicePos_Rhino.X - target_rhino_dm.X;
                double dy = devicePos_Rhino.Y - target_rhino_dm.Y;
                double dz = devicePos_Rhino.Z - target_rhino_dm.Z;
                double dist_rhino = Math.Sqrt(dx * dx + dy * dy + dz * dz);

                double currentAmpFactor;
                double usableRange = vibrationPointMaxDistance - vibrationPointDeadzone;
                if (usableRange < 0.001) usableRange = 0.001;

                if (vibrationPointInvertMapping)
                {
                    if (dist_rhino >= vibrationPointMaxDistance) currentAmpFactor = 0.0;
                    else if (dist_rhino < vibrationPointDeadzone) currentAmpFactor = 1.0;
                    else currentAmpFactor = 1.0 - ((dist_rhino - vibrationPointDeadzone) / usableRange);
                }
                else
                {
                    if (dist_rhino >= vibrationPointMaxDistance) currentAmpFactor = 1.0;
                    else if (dist_rhino < vibrationPointDeadzone) currentAmpFactor = 0.0;
                    else currentAmpFactor = (dist_rhino - vibrationPointDeadzone) / usableRange;
                }
                currentAmpFactor = Math.Max(0.0, Math.Min(1.0, currentAmpFactor));

                if (vibrationPointInvertMapping) effectiveAmpFactor = Math.Max(effectiveAmpFactor, currentAmpFactor);
                else effectiveAmpFactor = Math.Min(effectiveAmpFactor, currentAmpFactor);
                considered = true;
            }

            if (!considered) return new double[] { 0, 0, 0 };
            double resultantAmp = vibrationPointMaxAmplitude * effectiveAmpFactor;
            if (resultantAmp < (0.0001 * vibrationPointMaxAmplitude) || resultantAmp < 1e-5) return new double[] { 0, 0, 0 };

            double osc = Vibration.GetVibrationMultiplier(vibrationPointUseSquareWave ? Vibration.VibrationMode.Square : Vibration.VibrationMode.Sine, vibrationPointFrequency, 0, 0);
            double vibMag = resultantAmp * osc;

            Vector3d forceVec_Rhino = new Vector3d(
                vibrationPointDirection_RhinoCoords.X,
                vibrationPointDirection_RhinoCoords.Y,
                vibrationPointDirection_RhinoCoords.Z
            );
            forceVec_Rhino.Unitize();
            forceVec_Rhino *= vibMag;

            return new double[] { -forceVec_Rhino.X, forceVec_Rhino.Z, forceVec_Rhino.Y };
        }

        #endregion Existing Force Calculation Methods

        // --- UPDATED: Dog on a Leash Force Calculation ---
        private static double[] CalculateDogOnLeashForce_Native(DeviceManager.Vector3D devicePos_Rhino, bool button1Pressed)
        {
            if (!dogOnLeashEnabled || dogLeashCurve_RhinoCoords == null || !dogLeashCurve_RhinoCoords.IsValid)
            {
                DogWorldPosition_RhinoCoords = Point3d.Unset;
                return new double[] { 0, 0, 0 };
            }

            DateTime now = DateTime.Now;
            double dt = (now - lastDogLeashUpdateTime).TotalSeconds;
            lastDogLeashUpdateTime = now;

            Point3d devicePoint_Rhino = new Point3d(devicePos_Rhino.X, devicePos_Rhino.Y, devicePos_Rhino.Z);

            // --- Update Dog's Position ---
            if (button1Pressed)
            {
                // On the first frame the button is pressed, "pick up" the dog from the curve.
                if (!wasButton1PressedLastFrame)
                {
                    dogFreeSpacePosition_RhinoCoords = dogLeashCurve_RhinoCoords.PointAtLength(dogCurrentLengthPosition);
                }

                // Apply lazy string logic to the free-space position.
                Vector3d vecToUser = devicePoint_Rhino - dogFreeSpacePosition_RhinoCoords;
                double distToUser = vecToUser.Length;

                if (distToUser > leashLength)
                {
                    double overshoot = distToUser - leashLength;
                    vecToUser.Unitize();
                    dogFreeSpacePosition_RhinoCoords += vecToUser * overshoot;
                }

                DogWorldPosition_RhinoCoords = dogFreeSpacePosition_RhinoCoords;
            }
            else
            {
                // On the first frame the button is released, snap the dog back to the curve.
                if (wasButton1PressedLastFrame && dogFreeSpacePosition_RhinoCoords.IsValid)
                {
                    double closestT;
                    dogLeashCurve_RhinoCoords.ClosestPoint(dogFreeSpacePosition_RhinoCoords, out closestT);
                    dogCurrentLengthPosition = dogLeashCurve_RhinoCoords.GetLength(new Interval(dogLeashCurve_RhinoCoords.Domain.Min, closestT));
                    dogFreeSpacePosition_RhinoCoords = Point3d.Unset; // Invalidate the free-space position.
                }

                // Autonomous movement logic when not dragging.
                double movementIncrement = dogSpeed * dt;
                dogCurrentLengthPosition += movementIncrement;

                if (totalDogCurveLength > 0.001)
                {
                    if (dogLeashCurve_RhinoCoords.IsClosed)
                    {
                        dogCurrentLengthPosition = (dogCurrentLengthPosition % totalDogCurveLength + totalDogCurveLength) % totalDogCurveLength;
                    }
                    else
                    {
                        dogCurrentLengthPosition = Math.Max(0, Math.Min(dogCurrentLengthPosition, totalDogCurveLength));
                    }
                }
                DogWorldPosition_RhinoCoords = dogLeashCurve_RhinoCoords.PointAtLength(dogCurrentLengthPosition);
            }

            // --- Calculate Haptic Force ---
            double finalDistToUser = devicePoint_Rhino.DistanceTo(DogWorldPosition_RhinoCoords);
            double forceScale = 0.0;
            if (finalDistToUser > leashLength)
            {
                double overshoot = finalDistToUser - leashLength;
                forceScale = Math.Min(1.0, overshoot / leashSpringRange);
            }

            Vector3d forceVec_Rhino = Vector3d.Zero;
            if (forceScale > 0.001)
            {
                double forceMagnitude = leashMaxForce * forceScale;
                forceVec_Rhino = DogWorldPosition_RhinoCoords - devicePoint_Rhino;
                forceVec_Rhino.Unitize();
                forceVec_Rhino *= forceMagnitude;
            }

            return new double[] { -forceVec_Rhino.X, forceVec_Rhino.Z, forceVec_Rhino.Y };
        }
        // --- End of Updated Method ---


        public static void UpdateForces()
        {
            DateTime currentTime = DateTime.Now;
            double timeDeltaSinceLastForceUpdate = (currentTime - lastForceUpdateTime).TotalSeconds;
            lastForceUpdateTime = currentTime;

            double[] totalForceForFrame_NativeDeviceCoords = new double[3] { 0, 0, 0 };

            var state = DeviceManager.GetCurrentState();
            if (state.Transform == null)
            {
                state.ReturnArrays();
                return;
            }
            var hdTransformMatrix = state.Transform;
            bool button1Pressed = (state.Buttons & 0x01) != 0;


            DeviceManager.Vector3D rawGimbalPos_Native = new DeviceManager.Vector3D(
                hdTransformMatrix[12], hdTransformMatrix[13], hdTransformMatrix[14]);

            DeviceManager.Vector3D realTCP_Native = rawGimbalPos_Native;
            if (tcpOffset.X != 0 || tcpOffset.Y != 0 || tcpOffset.Z != 0)
            {
                double ox = tcpOffset.X * hdTransformMatrix[0] + tcpOffset.Y * hdTransformMatrix[4] + tcpOffset.Z * hdTransformMatrix[8];
                double oy = tcpOffset.X * hdTransformMatrix[1] + tcpOffset.Y * hdTransformMatrix[5] + tcpOffset.Z * hdTransformMatrix[9];
                double oz = tcpOffset.X * hdTransformMatrix[2] + tcpOffset.Y * hdTransformMatrix[6] + tcpOffset.Z * hdTransformMatrix[10];
                realTCP_Native.X += ox;
                realTCP_Native.Y += oy;
                realTCP_Native.Z += oz;
            }

            DeviceManager.Vector3D interactionPosition_Native;

            if (lazyStringEnabled)
            {
                if (isFirstLazyStringFrame)
                {
                    tooltipPosition_Native = realTCP_Native;
                    isFirstLazyStringFrame = false;
                }

                double dx = realTCP_Native.X - tooltipPosition_Native.X;
                double dy = realTCP_Native.Y - tooltipPosition_Native.Y;
                double dz = realTCP_Native.Z - tooltipPosition_Native.Z;
                double distance = Math.Sqrt(dx * dx + dy * dy + dz * dz);

                if (distance > stringLength)
                {
                    double overshoot = distance - stringLength;
                    if (distance > 0.001)
                    {
                        var direction = new DeviceManager.Vector3D(dx / distance, dy / distance, dz / distance);
                        tooltipPosition_Native.X += direction.X * overshoot;
                        tooltipPosition_Native.Y += direction.Y * overshoot;
                        tooltipPosition_Native.Z += direction.Z * overshoot;
                    }

                    double forceDirX = tooltipPosition_Native.X - realTCP_Native.X;
                    double forceDirY = tooltipPosition_Native.Y - realTCP_Native.Y;
                    double forceDirZ = tooltipPosition_Native.Z - realTCP_Native.Z;
                    double forceDirMag = Math.Sqrt(forceDirX * forceDirX + forceDirY * forceDirY + forceDirZ * forceDirZ);

                    if (forceDirMag > 0.001)
                    {
                        forceDirX /= forceDirMag;
                        forceDirY /= forceDirMag;
                        forceDirZ /= forceDirMag;
                    }

                    var hapticForce = new double[3] {
                        forceDirX * returnForce,
                        forceDirY * returnForce,
                        forceDirZ * returnForce
                    };
                    for (int i = 0; i < 3; i++) totalForceForFrame_NativeDeviceCoords[i] += hapticForce[i];
                }
                interactionPosition_Native = tooltipPosition_Native;
            }
            else
            {
                interactionPosition_Native = realTCP_Native;
            }

            CurrentDeviceTCP_NativeCoords = interactionPosition_Native;
            TooltipPosition_Native = tooltipPosition_Native;
            DeviceManager.Vector3D currentTCP_RhinoCoords = new DeviceManager.Vector3D(
                -CurrentDeviceTCP_NativeCoords.X, CurrentDeviceTCP_NativeCoords.Z, CurrentDeviceTCP_NativeCoords.Y);


            // --- Accumulate OTHER forces ---
            if (dogOnLeashEnabled) { var f = CalculateDogOnLeashForce_Native(currentTCP_RhinoCoords, button1Pressed); for (int i = 0; i < 3; i++) totalForceForFrame_NativeDeviceCoords[i] += f[i]; }
            if (viscousDampingEnabled) { var f = CalculateViscousForce_Native(); for (int i = 0; i < 3; i++) totalForceForFrame_NativeDeviceCoords[i] += f[i]; }
            if (vibrationCurveEnabled) { var f = CalculateVibrationCurveForce_Native(currentTCP_RhinoCoords); for (int i = 0; i < 3; i++) totalForceForFrame_NativeDeviceCoords[i] += f[i]; }
            if (vibrationPointSystemEnabled) { var f = CalculateVibrationPointForce_Native(currentTCP_RhinoCoords); for (int i = 0; i < 3; i++) totalForceForFrame_NativeDeviceCoords[i] += f[i]; }
            if (planeCollisionEnabled) { var f = CalculatePlaneCollisionForce_Native(currentTCP_RhinoCoords); for (int i = 0; i < 3; i++) totalForceForFrame_NativeDeviceCoords[i] += f[i]; }
            if (multiPullToPointEnabled) { if (smoothingEnabledMultiPoint) UpdateSmoothedTargets(); var f = CalculateMultiPullToPointForce_Native(currentTCP_RhinoCoords); for (int i = 0; i < 3; i++) totalForceForFrame_NativeDeviceCoords[i] += f[i]; }
            if (pullToCurveEnabled) { var f = CalculatePullToCurveForce_Native(currentTCP_RhinoCoords); for (int i = 0; i < 3; i++) totalForceForFrame_NativeDeviceCoords[i] += f[i]; }
            if (pullToPlaneEnabled) { var f = CalculatePullToPlaneForce_Native(currentTCP_RhinoCoords); for (int i = 0; i < 3; i++) totalForceForFrame_NativeDeviceCoords[i] += f[i]; }
            if (filteredForceEnabled && forceFilter != null) { for (int i = 0; i < 3; i++) totalForceForFrame_NativeDeviceCoords[i] += lastFilteredForce[i]; }
            else if (directForceEnabled) { for (int i = 0; i < 3; i++) totalForceForFrame_NativeDeviceCoords[i] += currentDirectForce[i]; }

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
                        double rawMCVal = (double)getMappedForceMethod.Invoke(null, null);
                        if (Math.Abs(rawMCVal) > 0.001)
                        {
                            totalForceForFrame_NativeDeviceCoords[0] += hdTransformMatrix[8] * rawMCVal;
                            totalForceForFrame_NativeDeviceCoords[1] += hdTransformMatrix[9] * rawMCVal;
                            totalForceForFrame_NativeDeviceCoords[2] += hdTransformMatrix[10] * rawMCVal;
                        }
                    }
                }
            }
            catch (Exception ex) { Debug.WriteLine($"UCManager error: {ex.Message}"); }

            state.ReturnArrays();

            if (dampingEnabled)
            {
                double[] dampedTotalForce_Native = new double[3];
                double alpha = 1.0 - dampingCoefficient;
                switch (currentDampingMethod)
                {
                    case DampingMethod.ExponentialSmoothing:
                        for (int i = 0; i < 3; i++) dampedTotalForce_Native[i] = alpha * totalForceForFrame_NativeDeviceCoords[i] + dampingCoefficient * lastAppliedForce[i];
                        break;
                    case DampingMethod.ForceDerivative:
                        for (int i = 0; i < 3; i++)
                        {
                            double derivative = (totalForceForFrame_NativeDeviceCoords[i] - previousForce[i]) / Math.Max(0.001, timeDeltaSinceLastForceUpdate);
                            dampedTotalForce_Native[i] = totalForceForFrame_NativeDeviceCoords[i] - (derivativeDampingCoefficient * derivative);
                        }
                        break;
                    case DampingMethod.Both:
                        double[] smoothed = new double[3];
                        for (int i = 0; i < 3; i++) smoothed[i] = alpha * totalForceForFrame_NativeDeviceCoords[i] + dampingCoefficient * lastAppliedForce[i];
                        for (int i = 0; i < 3; i++)
                        {
                            double derivative = (smoothed[i] - previousForce[i]) / Math.Max(0.001, timeDeltaSinceLastForceUpdate);
                            dampedTotalForce_Native[i] = smoothed[i] - (derivativeDampingCoefficient * derivative);
                        }
                        break;
                    default: Array.Copy(totalForceForFrame_NativeDeviceCoords, dampedTotalForce_Native, 3); break;
                }
                previousForce = (double[])lastAppliedForce.Clone();
                lastAppliedForce = (double[])dampedTotalForce_Native.Clone();
                currentTotalForce = (double[])dampedTotalForce_Native.Clone();
                HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, dampedTotalForce_Native);
            }
            else
            {
                previousForce = (double[])lastAppliedForce.Clone();
                lastAppliedForce = (double[])totalForceForFrame_NativeDeviceCoords.Clone();
                currentTotalForce = (double[])totalForceForFrame_NativeDeviceCoords.Clone();
                HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, totalForceForFrame_NativeDeviceCoords);
            }

            // Update the button state for the next frame
            wasButton1PressedLastFrame = button1Pressed;
        }

        public static void Reset()
        {
            directForceEnabled = false; filteredForceEnabled = false; if (forceFilter != null) forceFilter.Reset();
            multiPullToPointEnabled = false; currentTargetPoints_RhinoCoords.Clear(); currentSmoothedTargetPoints_RhinoCoords.Clear();
            pullToPlaneEnabled = false;
            dampingEnabled = false; Array.Clear(lastAppliedForce, 0, 3); Array.Clear(previousForce, 0, 3);
            viscousDampingEnabled = false; Array.Clear(lastViscousForce_Native, 0, 3); Array.Clear(lastVelocity_Native, 0, 3); Array.Clear(filteredVelocity_Native, 0, 3); if (viscousForceFilter != null) viscousForceFilter.Reset();
            planeCollisionEnabled = false;
            vibrationCurveEnabled = false; vibrationCurve_RhinoCoords = null;
            vibrationPointSystemEnabled = false; vibrationPointTargets_RhinoCoords.Clear();
            pullToCurveEnabled = false; pullToCurve_RhinoCoords = null; ResetTravelingPointInternal();
            tcpOffset = new DeviceManager.Vector3D(0, 0, 0);
            CurrentDeviceTCP_NativeCoords = new DeviceManager.Vector3D(0, 0, 0);
            Vibration.Reset();

            lazyStringEnabled = false;
            isFirstLazyStringFrame = true;
            tooltipPosition_Native = new DeviceManager.Vector3D(0, 0, 0);
            TooltipPosition_Native = new DeviceManager.Vector3D(0, 0, 0);

            dogOnLeashEnabled = false;
            dogLeashCurve_RhinoCoords = null;
            DogWorldPosition_RhinoCoords = Point3d.Unset;
            dogFreeSpacePosition_RhinoCoords = Point3d.Unset;
            wasButton1PressedLastFrame = false;

            currentTotalForce = new double[3] { 0, 0, 0 };
            HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, currentTotalForce);
        }
    }
}
