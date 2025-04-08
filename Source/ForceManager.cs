using System;
using System.Diagnostics;
using Rhino.Geometry;

namespace ghoh
{
    public static class ForceManager
    {
        private static DateTime lastUpdateTime = DateTime.Now;
        // Flags for enabled forces
        private static bool directForceEnabled;
        private static bool pullToPointEnabled;
        private static bool pullToPlaneEnabled;
        private static bool filteredForceEnabled;

        // Direct force parameters
        private static double[] currentDirectForce = new double[3];
        private static double[] currentTotalForce = new double[3];
        // Filtered force support
        private static UKF forceFilter;
        private static double[] lastFilteredForce = new double[3];
        private static double processNoise = 0.05;
        private static double measurementNoise = 0.3;

        // TCP Offset
        private static DeviceManager.Vector3D tcpOffset = new DeviceManager.Vector3D(0, 0, 0);

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

        // Damping parameters
        private static bool dampingEnabled = false;
        private static double dampingCoefficient = 0.5; // Default value (range 0-1)
        private static double derivativeDampingCoefficient = 0.0; // Default value
        private static DampingMethod currentDampingMethod = DampingMethod.ExponentialSmoothing;
        private static double[] lastAppliedForce = new double[3]; // To track previous force
        private static double[] previousForce = new double[3]; // For derivative calculation
        private static DateTime lastForceUpdateTime = DateTime.Now;
        private static DateTime lastLogTime = DateTime.MinValue;
       
        // Viscous force parameters
        private static bool viscousDampingEnabled;
        private static double viscousGain = 0.5;
        private static double viscousMaxForce = 3.0;
        private static double velocityDeadband = 10.0;
        private static double deadbandSoftness = 5.0; // Transition range for soft deadband
        private static double velocityFilterCoefficient = 0.9;
        private static int forceWindowSize = 10;

        // State tracking variables
        private static double[] lastVelocity = new double[3];
        private static double[] filteredVelocity = new double[3];
        private static double[] lastViscousForce = new double[3];
        private static DateTime lastViscousUpdateTime = DateTime.Now;

        // Moving average filter for viscous forces
        private static VectorMovingAverageFilter viscousForceFilter;

        //plane condtraint parameters
        private static bool planeCollisionEnabled;
        private static DeviceManager.Vector3D collisionPlaneOrigin;
        private static DeviceManager.Vector3D collisionPlaneNormal;
        private static double maxForceValueCollision = 1.0;
        private static double maxDistanceValueCollision = 1.0;

        // Vibration point parameters
        private static bool vibrationPointEnabled = false;
        private static DeviceManager.Vector3D vibrationPointTarget;
        private static DeviceManager.Vector3D vibrationDirection = new DeviceManager.Vector3D(0, 0, 1);
        private static double vibrationDeadzone = 0.0;
        private static double vibrationMaxDistance = 10.0;
        private static double vibrationMaxAmplitude = 1.0;
        private static double vibrationFrequency = 100.0;
        private static bool vibrationInvertMapping = false;
        private static bool vibrationUseSquareWave = false;

        // Curve vibration parameters
        public static bool vibrationCurveEnabled = false;
        public static Curve vibrationCurve = null;
        private static DeviceManager.Vector3D vibrationCurveDirection = new DeviceManager.Vector3D(0, 0, 1);
        private static double vibrationCurveDeadzone = 0.0;
        private static double vibrationCurveMaxDistance = 10.0;
        private static double vibrationCurveMaxAmplitude = 1.0;
        private static double vibrationCurveFrequency = 100.0;
        private static bool vibrationCurveInvertMapping = false;
        private static bool vibrationCurveUseSquareWave = false;
        public static double vibrationCurveDistance = 0.0;  // Made public for component access
        public static double vibrationCurveAmplitude = 0.0; // Made public for component access
        private static Stopwatch curveCalcStopwatch = new Stopwatch();
        public static double vibrationCurveCalcTime = 0.0;  // Made public for component access

        // Curve pull parameters
        private static bool pullToCurveEnabled = false;
        private static Curve pullToCurve = null;
        private static double maxForceValueCurve = 1.0;
        private static double maxDistanceValueCurve = 1.0;
        private static double lookAheadDistance = 0.01;
        public enum DampingMethod
        {
            ExponentialSmoothing,
            ForceDerivative,
            Both
        }
        public static double[] GetCurrentForce()
        {
            // Return a copy of the current force to avoid external modification
            return (double[])currentTotalForce.Clone();
        }
        // Update the SetViscousDamping method to support all parameters
        public static Vector3d SetViscousDamping(bool enable, double gain, double maxForce, double deadbandThreshold = 10.0, double softness = 5.0, double filterCoefficient = 0.9, int windowSize = 10){
            viscousDampingEnabled = enable;
            viscousGain = Math.Max(0.0, Math.Min(1.0, gain));
            viscousMaxForce = Math.Max(0.0, maxForce);
            velocityDeadband = Math.Max(0.0, deadbandThreshold);
            deadbandSoftness = Math.Max(0.0, Math.Min(velocityDeadband, softness)); // Softness can't exceed deadband
            velocityFilterCoefficient = Math.Max(0.0, Math.Min(0.99, filterCoefficient));
            forceWindowSize = Math.Max(1, windowSize);

            // Return the current viscous force for output in device coordinates transformed to Rhino coordinates
            return new Vector3d(-lastViscousForce[0], lastViscousForce[2], lastViscousForce[1]);
        }
        private static double[] CalculateViscousForce()
        {
            // Initialize the force filter if it doesn't exist
            if (viscousForceFilter == null)
            {
                viscousForceFilter = new VectorMovingAverageFilter(forceWindowSize);
            }
            else if (viscousForceFilter.GetWindowSize() != forceWindowSize)
            {
                viscousForceFilter.SetWindowSize(forceWindowSize);
            }

            // Track time between updates
            DateTime currentTime = DateTime.Now;
            double timeDelta = Math.Max(0.001, (currentTime - lastViscousUpdateTime).TotalSeconds);
            lastViscousUpdateTime = currentTime;

            // Get current velocity from device
            var rawVelocity = new double[3];
            HDdll.hdGetDoublev(HDdll.HD_CURRENT_VELOCITY, rawVelocity);

            // Apply low-pass filtering to velocity
            for (int i = 0; i < 3; i++)
            {
                filteredVelocity[i] = velocityFilterCoefficient * filteredVelocity[i] +
                                     (1 - velocityFilterCoefficient) * rawVelocity[i];
            }

            // Calculate velocity magnitude
            double velMagnitude = Math.Sqrt(
                filteredVelocity[0] * filteredVelocity[0] +
                filteredVelocity[1] * filteredVelocity[1] +
                filteredVelocity[2] * filteredVelocity[2]
            );

            // Calculate viscous force: F = -k * V (without deadband)
            double[] rawViscousForce = new double[3];

            // Apply damping in device coordinates
            rawViscousForce[0] = -viscousGain * filteredVelocity[0];
            rawViscousForce[1] = -viscousGain * filteredVelocity[1];
            rawViscousForce[2] = -viscousGain * filteredVelocity[2];

            // Calculate force magnitude
            double forceMag = Math.Sqrt(
                rawViscousForce[0] * rawViscousForce[0] +
                rawViscousForce[1] * rawViscousForce[1] +
                rawViscousForce[2] * rawViscousForce[2]
            );

            // Cap force if needed
            if (forceMag > viscousMaxForce && forceMag > 0.0001)
            {
                double scale = viscousMaxForce / forceMag;
                rawViscousForce[0] *= scale;
                rawViscousForce[1] *= scale;
                rawViscousForce[2] *= scale;
            }

            // Apply moving average filter to smooth force changes
            double[] smoothedForce = viscousForceFilter.AddSample(rawViscousForce);

            // Now apply soft deadband to the smoothed force
            double[] finalForce = new double[3];

            // Calculate a scaling factor based on velocity and deadband
            double scaleFactor = 1.0;

            if (velMagnitude < velocityDeadband - deadbandSoftness)
            {
                // Below the soft transition zone - no force
                scaleFactor = 0.0;
            }
            else if (velMagnitude < velocityDeadband)
            {
                // Inside the soft transition zone - gradually scale up
                double transitionPosition = velMagnitude - (velocityDeadband - deadbandSoftness);
                scaleFactor = transitionPosition / deadbandSoftness;
            }
            // else: Above deadband - full force (scaleFactor = 1.0)

            // Apply scaling to the smoothed force
            finalForce[0] = smoothedForce[0] * scaleFactor;
            finalForce[1] = smoothedForce[1] * scaleFactor;
            finalForce[2] = smoothedForce[2] * scaleFactor;

            // Store current values for next iteration
            lastVelocity = (double[])filteredVelocity.Clone();
            lastViscousForce = (double[])finalForce.Clone();

            return finalForce;
        }
        public static void SetTCPOffset(DeviceManager.Vector3D offset)
        {
            tcpOffset = offset;
        }

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

            UpdateForces();
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

        public static void SetDampingParameters(
            bool enable,
            double coefficient,
            double derivativeCoefficient,
            DampingMethod method)
        {
            dampingEnabled = enable;

            // Clamp coefficients to valid ranges
            dampingCoefficient = Math.Max(0, Math.Min(coefficient, 0.99));
            derivativeDampingCoefficient = Math.Max(0, Math.Min(derivativeCoefficient, 1.0));

            currentDampingMethod = method;

            Logger.Log($"Damping set: enabled={enable}, coefficient={dampingCoefficient:F2}, " +
                       $"derivative={derivativeDampingCoefficient:F2}, method={method}");
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
        public static void SetPullToCurve(
            Curve curve,
            bool enable,
            double maxForce,
            double maxDistance,
            double lookAhead)
        {
            pullToCurve = curve;
            pullToCurveEnabled = enable;
            maxForceValueCurve = Math.Max(0.0, maxForce);
            maxDistanceValueCurve = Math.Max(0.001, maxDistance);
            lookAheadDistance = Math.Max(0.0, lookAhead);
        }

        private static double[] CalculatePullToCurveForce(DeviceManager.Vector3D devicePos)
        {
            double[] force = new double[] { 0, 0, 0 };

            if (!pullToCurveEnabled || pullToCurve == null)
                return force;

            // Create a Point3d from the device position
            var devicePoint = new Point3d(devicePos.X, devicePos.Y, devicePos.Z);

            // Find closest point on curve
            double curveParam;
            Point3d closestPoint;

            if (!pullToCurve.ClosestPoint(devicePoint, out curveParam))
            {
                return force; // If no closest point found, return zero force
            }

            // If look ahead is enabled, move parameter forward on curve
            Point3d targetPoint;
            if (lookAheadDistance > 0.0)
            {
                try
                {
                    // Get the curve's domain
                    Interval domain = pullToCurve.Domain;

                    // Calculate the length from the start of the curve to our current parameter
                    double lengthToClosest = 0;
                    if (domain.Min < curveParam)
                    {
                        lengthToClosest = pullToCurve.GetLength(new Interval(domain.Min, curveParam));
                    }

                    // Add our look ahead distance
                    double targetLength = lengthToClosest + lookAheadDistance;

                    // Get the total curve length
                    double totalLength = pullToCurve.GetLength();

                    // Make sure we don't exceed the curve length
                    targetLength = Math.Min(targetLength, totalLength);

                    // Now find the parameter at this length
                    double newParam = curveParam;
                    // We need to use the NurbsCurve.GetCurveParameterFromLength method or similar approach
                    // For simplicity and reliability, we'll use approximation

                    // Get the curve length interval per parameter unit (approx)
                    double lengthPerParam = totalLength / domain.Length;

                    if (lengthPerParam > 0)
                    {
                        // Calculate parameter increment needed
                        double paramIncrement = lookAheadDistance / lengthPerParam;

                        // Apply increment, staying within domain
                        newParam = Math.Min(domain.Max, curveParam + paramIncrement);

                        // Get the point at this parameter
                        targetPoint = pullToCurve.PointAt(newParam);
                    }
                    else
                    {
                        // Fallback to closest point
                        targetPoint = pullToCurve.PointAt(curveParam);
                    }
                }
                catch
                {
                    // If anything fails, use the closest point
                    targetPoint = pullToCurve.PointAt(curveParam);
                }
            }
            else
            {
                // Just use the closest point if no look ahead
                targetPoint = pullToCurve.PointAt(curveParam);
            }

            // Calculate distance and direction to target point
            double dx = targetPoint.X - devicePoint.X;
            double dy = targetPoint.Y - devicePoint.Y;
            double dz = targetPoint.Z - devicePoint.Z;
            double distance = Math.Sqrt(dx * dx + dy * dy + dz * dz);

            // Calculate force based on distance
            if (distance < 0.001)
            {
                return force; // No force if already at target
            }

            // Scale force based on distance
            double forceMagnitude = distance > maxDistanceValueCurve ?
                maxForceValueCurve :
                maxForceValueCurve * (distance / maxDistanceValueCurve);

            // Calculate force components
            double fx = (dx / distance) * forceMagnitude;
            double fy = (dy / distance) * forceMagnitude;
            double fz = (dz / distance) * forceMagnitude;

            // Return force in device coordinates
            return new double[]
            {
        -fx, // Negate X for device space
        fz,  // Y becomes Z
        fy   // Z becomes Y
            };
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
        public static void SetPlaneCollision(DeviceManager.Vector3D origin, DeviceManager.Vector3D normal,bool enable, double maxForce, double maxDistance)
        {
            planeCollisionEnabled = enable;
            collisionPlaneOrigin = origin;
            collisionPlaneNormal = normal;
            maxForceValueCollision = maxForce;
            maxDistanceValueCollision = maxDistance;
        }

        // Add this method to the ForceManager class:
        private static double[] CalculatePlaneCollisionForce(DeviceManager.Vector3D devicePos)
        {
            double dx = devicePos.X - collisionPlaneOrigin.X;
            double dy = devicePos.Y - collisionPlaneOrigin.Y;
            double dz = devicePos.Z - collisionPlaneOrigin.Z;

            // Distance to the plane (signed: positive is in the direction of the normal)
            double distance = dx * collisionPlaneNormal.X + dy * collisionPlaneNormal.Y + dz * collisionPlaneNormal.Z;

            // Only apply force if the device is on the "wrong" side of the plane (distance < 0)
            if (distance >= 0)
            {
                // Device is on the "free" side, no force needed
                return new double[] { 0, 0, 0 };
            }

            // Calculate force magnitude based on penetration depth
            double absDistance = -distance; // Make positive for calculations
            double forceMagnitude;

            if (absDistance > maxDistanceValueCollision)
            {
                forceMagnitude = maxForceValueCollision;
            }
            else
            {
                forceMagnitude = (absDistance / maxDistanceValueCollision) * maxForceValueCollision;
            }

            // Force direction is along the normal (pushing away from the plane)
            double fx = collisionPlaneNormal.X * forceMagnitude;
            double fy = collisionPlaneNormal.Y * forceMagnitude;
            double fz = collisionPlaneNormal.Z * forceMagnitude;

            // Convert to device coordinates
            return new double[]
            {
        -fx,
        fz,
        fy
            };
        }

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

            if (distanceToTarget < 0.001)
            {
                return;
            }

            double stepSize = Math.Min(distanceToTarget, maxStepSize);
            double scale = stepSize / distanceToTarget;
            currentSmoothedTarget = new DeviceManager.Vector3D(
                currentSmoothedTarget.X + dx * scale,
                currentSmoothedTarget.Y + dy * scale,
                currentSmoothedTarget.Z + dz * scale
            );
        }

        private static double[] ApplyDamping(double[] currentForce, double timeDelta)
        {
            if (!dampingEnabled || (dampingCoefficient < 0.01 && derivativeDampingCoefficient < 0.01))
            {
                // Store for next time but don't modify force
                previousForce = (double[])lastAppliedForce.Clone();
                lastAppliedForce = (double[])currentForce.Clone();
                return currentForce;
            }

            double[] dampedForce = new double[3];

            // Apply damping based on selected method
            switch (currentDampingMethod)
            {
                case DampingMethod.ExponentialSmoothing:
                    // Implement exponential smoothing (1st order low-pass filter)
                    // alpha is the smoothing factor - lower values mean more smoothing
                    double alpha = 1.0 - dampingCoefficient;
                    for (int i = 0; i < 3; i++)
                    {
                        dampedForce[i] = alpha * currentForce[i] + dampingCoefficient * lastAppliedForce[i];
                    }
                    break;

                case DampingMethod.ForceDerivative:
                    // Implement force derivative damping
                    // This applies resistance based on how quickly the force is changing
                    for (int i = 0; i < 3; i++)
                    {
                        double derivative = (currentForce[i] - previousForce[i]) / Math.Max(0.001, timeDelta);
                        dampedForce[i] = currentForce[i] - (derivativeDampingCoefficient * derivative);
                    }
                    break;

                case DampingMethod.Both:
                    // Apply both methods sequentially
                    // First apply exponential smoothing
                    double alphaBoth = 1.0 - dampingCoefficient;
                    for (int i = 0; i < 3; i++)
                    {
                        dampedForce[i] = alphaBoth * currentForce[i] + dampingCoefficient * lastAppliedForce[i];
                    }

                    // Then apply derivative damping
                    double[] tempForce = (double[])dampedForce.Clone();
                    for (int i = 0; i < 3; i++)
                    {
                        double derivative = (tempForce[i] - previousForce[i]) / Math.Max(0.001, timeDelta);
                        dampedForce[i] = tempForce[i] - (derivativeDampingCoefficient * derivative);
                    }
                    break;
            }

            // Store current force as previous for next update
            previousForce = (double[])lastAppliedForce.Clone();
            lastAppliedForce = (double[])dampedForce.Clone();

            return dampedForce;
        }

        public static void UpdateForces()
        {
            DateTime currentTime = DateTime.Now;
            //TimeSpan interval = currentTime - lastUpdateTime;
            //lastUpdateTime = currentTime;
            //Logger.Log($"Time between updates: {interval.TotalMilliseconds} ms");

            var totalForce = new double[3];
            var transform = new double[16];
            HDdll.hdGetDoublev(HDdll.HD_CURRENT_TRANSFORM, transform);

            // Extract device orientation vectors
            var xDirection = new DeviceManager.Vector3D(
                -transform[0],
                transform[2],
                transform[1]
            );

            var yDirection = new DeviceManager.Vector3D(
                -transform[4],
                transform[6],
                transform[5]
            );

            var zDirection = new DeviceManager.Vector3D(
                -transform[8],
                transform[10],
                transform[9]
            );

            // Normalize the Z vector (up direction)
            double length = Math.Sqrt(
                zDirection.X * zDirection.X +
                zDirection.Y * zDirection.Y +
                zDirection.Z * zDirection.Z
            );

            if (length > 0.001)
            {
                zDirection.X /= length;
                zDirection.Y /= length;
                zDirection.Z /= length;
            }

            var devicePos = new DeviceManager.Vector3D(
                -transform[12],
                transform[14],
                transform[13]
            );

            if (tcpOffset.X != 0 || tcpOffset.Y != 0 || tcpOffset.Z != 0)
            {
                devicePos.X += tcpOffset.X * xDirection.X + tcpOffset.Y * yDirection.X + tcpOffset.Z * zDirection.X;
                devicePos.Y += tcpOffset.X * xDirection.Y + tcpOffset.Y * yDirection.Y + tcpOffset.Z * zDirection.Y;
                devicePos.Z += tcpOffset.X * xDirection.Z + tcpOffset.Y * yDirection.Z + tcpOffset.Z * zDirection.Z;
            }

            // Track Z position and forces for logging
            double penZPosition = devicePos.Z;
            double rawForceValue = 0;
            double filteredForceValue = 0;

            // Add viscous damping forces if enabled
            if (viscousDampingEnabled)
            {
                var viscousForce = CalculateViscousForce();
                for (int i = 0; i < 3; i++)
                    totalForce[i] += viscousForce[i];
            }
            if (planeCollisionEnabled)
            {
                var collisionForce = CalculatePlaneCollisionForce(devicePos);
                for (int i = 0; i < 3; i++)
                    totalForce[i] += collisionForce[i];
            }
            if (vibrationPointEnabled)
            {
                var vibrationForce = CalculateVibrationForce(devicePos);
                for (int i = 0; i < 3; i++)
                    totalForce[i] += vibrationForce[i];
            }
            if (vibrationCurveEnabled && vibrationCurve != null)
            {
                var curveVibrationForce = CalculateVibrationCurveForce(devicePos);
                for (int i = 0; i < 3; i++)
                    totalForce[i] += curveVibrationForce[i];
            }
            if (pullToPointEnabled)
            {
                UpdateSmoothedTarget(devicePos);
                var pointForce = CalculatePullToPointForce(devicePos);
                for (int i = 0; i < 3; i++)
                    totalForce[i] += pointForce[i];
            }
            if (pullToCurveEnabled && pullToCurve != null)
            {
                var curveForce = CalculatePullToCurveForce(devicePos);
                for (int i = 0; i < 3; i++)
                    totalForce[i] += curveForce[i];
            }
            if (pullToPlaneEnabled)
            {
                var planeForce = CalculatePullToPlaneForce(devicePos);
                for (int i = 0; i < 3; i++)
                    totalForce[i] += planeForce[i];
            }

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

            // Apply microcontroller force in device Z (up) direction if enabled
            if (UCManager.IsConnected && UCManager.ForceEnabled)
            {
                rawForceValue = UCManager.GetMappedForceValue();

                if (rawForceValue > 0.001) // Only apply if there's a meaningful force
                {
                    // Apply the force along the device's up direction (Z axis)
                    double fx = zDirection.X * rawForceValue;
                    double fy = zDirection.Y * rawForceValue;
                    double fz = zDirection.Z * rawForceValue;

                    // Convert to device coordinates
                    double[] mcForce = new double[3] {
                -fx, // Negate X for device space
                fz,  // Y becomes Z
                fy   // Z becomes Y
            };

                    // Calculate time delta for damping
                    double timeDelta = (currentTime - lastForceUpdateTime).TotalSeconds;
                    lastForceUpdateTime = currentTime;

                    // Apply damping to the microcontroller force
                    double[] dampedMcForce = ApplyDamping(mcForce, timeDelta);

                    // Calculate filtered force magnitude for logging
                    double originalMagnitude = Math.Sqrt(mcForce[0] * mcForce[0] + mcForce[1] * mcForce[1] + mcForce[2] * mcForce[2]);
                    double dampedMagnitude = Math.Sqrt(dampedMcForce[0] * dampedMcForce[0] + dampedMcForce[1] * dampedMcForce[1] + dampedMcForce[2] * dampedMcForce[2]);

                    if (originalMagnitude > 0.0001)
                        filteredForceValue = rawForceValue * (dampedMagnitude / originalMagnitude);
                    else
                        filteredForceValue = 0;

                    // Add the damped force to total force
                    for (int i = 0; i < 3; i++)
                        totalForce[i] += dampedMcForce[i];
                }
            }

            // Log detailed diagnostic information on every update
            // Include milliseconds in timestamp for higher time resolution
            string timestamp = DateTime.Now.ToString("HH:mm:ss.fff");
            //Logger.Log($"[{timestamp}] Z:{penZPosition:F3}, raw:{rawForceValue:F3}, filt:{filteredForceValue:F3}");
            currentTotalForce = (double[])totalForce.Clone();
            HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, totalForce);
        }


        private static double[] CalculateVibrationForce(DeviceManager.Vector3D devicePos)
        {
            double[] force = new double[] { 0, 0, 0 };

            if (!vibrationPointEnabled || vibrationMaxAmplitude < 0.001)
                return force;

            // Calculate distance to target
            double dx = devicePos.X - vibrationPointTarget.X;
            double dy = devicePos.Y - vibrationPointTarget.Y;
            double dz = devicePos.Z - vibrationPointTarget.Z;
            double distance = Math.Sqrt(dx * dx + dy * dy + dz * dz);

            // Map distance to amplitude with deadzone
            double amplitude;

            if (vibrationInvertMapping)
            {
                // Inverted mode:
                // point (constant high) > deadzone (constant high) > distance (proportional less) > max distance (none)
                if (distance >= vibrationMaxDistance)
                {
                    // Beyond max distance - no vibration
                    amplitude = 0.0;
                }
                else if (distance < vibrationDeadzone)
                {
                    // Within deadzone - constant maximum vibration
                    amplitude = vibrationMaxAmplitude;
                }
                else
                {
                    // Between deadzone and max distance - decreasing vibration
                    double usableRange = vibrationMaxDistance - vibrationDeadzone;
                    double relativePosition = distance - vibrationDeadzone;
                    amplitude = (1.0 - (relativePosition / usableRange)) * vibrationMaxAmplitude;
                }
            }
            else
            {
                // Normal mode:
                // point > deadzone (none) > distance (proportional higher) > max distance (constant high)
                if (distance >= vibrationMaxDistance)
                {
                    // Beyond max distance - constant maximum vibration
                    amplitude = vibrationMaxAmplitude;
                }
                else if (distance < vibrationDeadzone)
                {
                    // Within deadzone - no vibration
                    amplitude = 0.0;
                }
                else
                {
                    // Between deadzone and max distance - increasing vibration
                    double usableRange = vibrationMaxDistance - vibrationDeadzone;
                    double relativePosition = distance - vibrationDeadzone;
                    amplitude = (relativePosition / usableRange) * vibrationMaxAmplitude;
                }
            }

            // No vibration if amplitude is effectively zero
            if (amplitude < 0.001)
                return force;

            // Get oscillation multiplier (-1 to 1) from Vibration class
            double oscillation = Vibration.GetVibrationMultiplier(
                vibrationFrequency,
                vibrationUseSquareWave
            );

            // Calculate force components in device coordinates
            force[0] = -vibrationDirection.X * oscillation * amplitude;
            force[1] = vibrationDirection.Z * oscillation * amplitude;
            force[2] = vibrationDirection.Y * oscillation * amplitude;

            return force;
        }
        public static void SetVibratePoint(
            DeviceManager.Vector3D target,
            DeviceManager.Vector3D direction,
            bool enable,
            double deadzone,
            double maxDistance,
            double maxAmplitude,
            double frequency,
            bool invertMapping,
            bool useSquareWave)
        {
            vibrationPointEnabled = enable;
            vibrationPointTarget = target;

            // Normalize direction if it has magnitude
            double magSq = direction.X * direction.X + direction.Y * direction.Y + direction.Z * direction.Z;
            if (magSq > 0.001)
            {
                double mag = Math.Sqrt(magSq);
                vibrationDirection = new DeviceManager.Vector3D(
                    direction.X / mag,
                    direction.Y / mag,
                    direction.Z / mag
                );
            }
            else
            {
                // Default to Z-axis (up) if no direction specified
                vibrationDirection = new DeviceManager.Vector3D(0, 0, 1);
            }

            // Clamp other values to reasonable ranges
            vibrationDeadzone = Math.Max(0.0, deadzone);
            vibrationMaxDistance = Math.Max(vibrationDeadzone + 0.001, maxDistance); // Ensure max distance > deadzone
            vibrationMaxAmplitude = Math.Max(0.0, Math.Min(3.0, maxAmplitude));
            vibrationFrequency = Math.Max(1.0, Math.Min(1000.0, frequency));
            vibrationInvertMapping = invertMapping;
            vibrationUseSquareWave = useSquareWave;
        }

        public static void SetVibrateCurve(
            Curve curve,
            DeviceManager.Vector3D direction,
            bool enable,
            double deadzone,
            double maxDistance,
            double maxAmplitude,
            double frequency,
            bool invertMapping,
            bool useSquareWave)
        {
            vibrationCurve = curve;
            vibrationCurveEnabled = enable;

            // Normalize direction if it has magnitude
            double magSq = direction.X * direction.X + direction.Y * direction.Y + direction.Z * direction.Z;
            if (magSq > 0.001)
            {
                double mag = Math.Sqrt(magSq);
                vibrationCurveDirection = new DeviceManager.Vector3D(
                    direction.X / mag,
                    direction.Y / mag,
                    direction.Z / mag
                );
            }
            else
            {
                // Default to Z-axis (up) if no direction specified
                vibrationCurveDirection = new DeviceManager.Vector3D(0, 0, 1);
            }

            // Clamp other values to reasonable ranges
            vibrationCurveDeadzone = Math.Max(0.0, deadzone);
            vibrationCurveMaxDistance = Math.Max(vibrationCurveDeadzone + 0.001, maxDistance); // Ensure max distance > deadzone
            vibrationCurveMaxAmplitude = Math.Max(0.0, Math.Min(3.0, maxAmplitude));
            vibrationCurveFrequency = Math.Max(1.0, Math.Min(1000.0, frequency));
            vibrationCurveInvertMapping = invertMapping;
            vibrationCurveUseSquareWave = useSquareWave;
        }

        // 3. Add the CalculateVibrationCurveForce method:

        private static double[] CalculateVibrationCurveForce(DeviceManager.Vector3D devicePos)
        {
            double[] force = new double[] { 0, 0, 0 };

            if (!vibrationCurveEnabled || vibrationCurve == null || vibrationCurveMaxAmplitude < 0.001)
                return force;

            // Create a Point3d from the device position
            var devicePoint = new Point3d(devicePos.X, devicePos.Y, devicePos.Z);

            // Start timing the distance calculation
            curveCalcStopwatch.Restart();

            // Calculate distance to curve
            double curveParam;
            Point3d closestPoint;
            double distance;

            if (vibrationCurve.ClosestPoint(devicePoint, out curveParam, vibrationCurveMaxDistance * 2))
            {
                closestPoint = vibrationCurve.PointAt(curveParam);
                distance = devicePoint.DistanceTo(closestPoint);
            }
            else
            {
                // If no closest point found within search range, use max distance
                distance = vibrationCurveMaxDistance;
            }

            // Stop timing and record
            curveCalcStopwatch.Stop();
            vibrationCurveCalcTime = curveCalcStopwatch.Elapsed.TotalMilliseconds;
            vibrationCurveDistance = distance;

            // Log the timing information
            if (vibrationCurveCalcTime > 10.0) // Only log if calculation takes significant time
            {
                Logger.Log($"Curve distance calc: {vibrationCurveCalcTime:F3}ms, Distance: {distance:F3}");
            }

            // Map distance to amplitude with deadzone
            double amplitude;

            if (vibrationCurveInvertMapping)
            {
                // Inverted mode:
                // curve (constant high) > deadzone (constant high) > distance (proportional less) > max distance (none)
                if (distance >= vibrationCurveMaxDistance)
                {
                    // Beyond max distance - no vibration
                    amplitude = 0.0;
                }
                else if (distance < vibrationCurveDeadzone)
                {
                    // Within deadzone - constant maximum vibration
                    amplitude = vibrationCurveMaxAmplitude;
                }
                else
                {
                    // Between deadzone and max distance - decreasing vibration
                    double usableRange = vibrationCurveMaxDistance - vibrationCurveDeadzone;
                    double relativePosition = distance - vibrationCurveDeadzone;
                    amplitude = (1.0 - (relativePosition / usableRange)) * vibrationCurveMaxAmplitude;
                }
            }
            else
            {
                // Normal mode:
                // curve > deadzone (none) > distance (proportional higher) > max distance (constant high)
                if (distance >= vibrationCurveMaxDistance)
                {
                    // Beyond max distance - constant maximum vibration
                    amplitude = vibrationCurveMaxAmplitude;
                }
                else if (distance < vibrationCurveDeadzone)
                {
                    // Within deadzone - no vibration
                    amplitude = 0.0;
                }
                else
                {
                    // Between deadzone and max distance - increasing vibration
                    double usableRange = vibrationCurveMaxDistance - vibrationCurveDeadzone;
                    double relativePosition = distance - vibrationCurveDeadzone;
                    amplitude = (relativePosition / usableRange) * vibrationCurveMaxAmplitude;
                }
            }

            // Store current amplitude for output
            vibrationCurveAmplitude = amplitude;

            // No vibration if amplitude is effectively zero
            if (amplitude < 0.001)
                return force;

            // Get oscillation multiplier (-1 to 1) from Vibration class
            double oscillation = Vibration.GetVibrationMultiplier(
                vibrationCurveFrequency,
                vibrationCurveUseSquareWave
            );

            // Calculate force components in device coordinates
            force[0] = -vibrationCurveDirection.X * oscillation * amplitude;
            force[1] = vibrationCurveDirection.Z * oscillation * amplitude;
            force[2] = vibrationCurveDirection.Y * oscillation * amplitude;

            return force;
        }

        private static double[] CalculatePullToPointForce(DeviceManager.Vector3D devicePos)
        {
            var targetToUse = interpolationEnabled ? currentSmoothedTarget : targetPoint;

            var dx = targetToUse.X - devicePos.X;
            var dy = targetToUse.Y - devicePos.Y;
            var dz = targetToUse.Z - devicePos.Z;

            var distance = Math.Sqrt(dx * dx + dy * dy + dz * dz);

            if (distance < 0.001)
            {
                return new double[] { 0, 0, 0 };
            }

            var scale = distance > maxDistanceValuePoint ?
                maxForceValuePoint :
                maxForceValuePoint * (distance / maxDistanceValuePoint);

            var fx = (dx / distance) * scale;
            var fy = (dy / distance) * scale;
            var fz = (dz / distance) * scale;

            return new double[]
            {
                -fx,
                fz,
                fy
            };
        }

        private static double[] CalculatePullToPlaneForce(DeviceManager.Vector3D devicePos)
        {
            double dx = devicePos.X - planeOrigin.X;
            double dy = devicePos.Y - planeOrigin.Y;
            double dz = devicePos.Z - planeOrigin.Z;

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

            double direction = distance > 0 ? -1 : 1;
            double fx = planeNormal.X * direction * forceMagnitude;
            double fy = planeNormal.Y * direction * forceMagnitude;
            double fz = planeNormal.Z * direction * forceMagnitude;

            return new double[]
            {
                -fx,
                fz,
                fy
            };
        }


        public static void Reset()
        {
            pullToCurveEnabled = false;
            pullToCurve = null;

            directForceEnabled = false;
            pullToPointEnabled = false;
            pullToPlaneEnabled = false;
            filteredForceEnabled = false;
            interpolationEnabled = false;
            dampingEnabled = false;
            viscousDampingEnabled = false;
            tcpOffset = new DeviceManager.Vector3D(0, 0, 0);
            planeCollisionEnabled = false;
            vibrationPointEnabled = false;
            vibrationMaxAmplitude = 0.0;
            Vibration.Reset();
            if (forceFilter != null)
            {
                forceFilter.Reset();
            }

            if (ghoh.ForceManager.forceFilter != null)
            {
                ghoh.ForceManager.forceFilter.Reset();
            }

            // Reset damping state
            lastAppliedForce = new double[3];
            previousForce = new double[3];
            lastViscousForce = new double[3];
            lastVelocity = new double[3];
            filteredVelocity = new double[3];

            vibrationCurveEnabled = false;
            vibrationCurve = null;
            vibrationCurveMaxAmplitude = 0.0;
            vibrationCurveDistance = 0.0;
            vibrationCurveAmplitude = 0.0;
            vibrationCurveCalcTime = 0.0;
            HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, new double[3]);
        }
    }
}