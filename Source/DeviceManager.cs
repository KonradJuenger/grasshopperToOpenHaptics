using System;
using System.Runtime.InteropServices;
using System.Threading;
using System.Buffers;
using Rhino.Geometry;

namespace ghoh
{
    public static class DeviceManager
    {
        private static int deviceHandle = HDdll.HD_INVALID_HANDLE;
        private static readonly object deviceLock = new object();
        private static readonly ArrayPool<double> arrayPool = ArrayPool<double>.Shared;

        // Thread-safe state handling with proper synchronization
        private static DeviceState currentState;
        private static readonly object stateLock = new object();

        // Force parameters with atomic operations
        private static Vector3D targetPoint;
        private static long forceEnabledFlag; // 0 = false, 1 = true
        private static double maxForceValue = 1.0;
        private static double maxDistanceValue = 1.0;
        private static long isRunningFlag; // 0 = false, 1 = true

        // Plane constraint parameters
        private static bool planeConstraintEnabled = false;
        private static Point3d planeOrigin;
        private static Vector3d planeNormal;
        private static Transform worldToDevice = Transform.Identity;
        private static readonly object planeConstraintLock = new object();

        // interpolation parameters
        private static Vector3D previousTarget;
        private static Vector3D currentInterpolatedTarget;
        private static DateTime lastTargetUpdateTime = DateTime.MinValue;
        private static bool interpolationEnabled = false;
        private static double interpolationTimeWindow = 30.0;


        private static bool directForceEnabled = false;
        private static double[] directForce = new double[3];

        // Struct to hold device state
        public struct DeviceState
        {
            public double[] Position;
            public double[] Transform;
            public int Buttons;

            public void ReturnArrays()
            {
                if (Position != null) arrayPool.Return(Position);
                if (Transform != null) arrayPool.Return(Transform);
            }
        }

        public struct Vector3D
        {
            public double X, Y, Z;

            public Vector3D(double x, double y, double z)
            {
                X = x;
                Y = y;
                Z = z;
            }
        }

        private static HDdll.HDSchedulerCallback servoLoopCallback = ServoLoopCallback;

        public static int DeviceHandle
        {
            get
            {
                lock (deviceLock)
                {
                    return deviceHandle;
                }
            }
        }

        public static bool Initialize(out string errorMessage)
        {
            lock (deviceLock)
            {
                if (deviceHandle != HDdll.HD_INVALID_HANDLE)
                {
                    errorMessage = null;
                    return true;
                }

                deviceHandle = HDdll.hdInitDevice(HDdll.HD_DEFAULT_DEVICE);

                if (deviceHandle == HDdll.HD_INVALID_HANDLE)
                {
                    HDdll.HDErrorInfo err = HDdll.hdGetError();
                    IntPtr errPtr = HDdll.hdGetErrorString(err.ErrorCode);
                    errorMessage = Marshal.PtrToStringAnsi(errPtr);
                    return false;
                }

                HDdll.hdMakeCurrentDevice(deviceHandle);
                HDdll.hdEnable(HDdll.HD_FORCE_OUTPUT);

                // Initialize state
                Interlocked.Exchange(ref isRunningFlag, 1);
                lock (stateLock)
                {
                    currentState = new DeviceState
                    {
                        Position = arrayPool.Rent(3),
                        Transform = arrayPool.Rent(16)
                    };
                }

                // Register callback and start the servo loop
                IntPtr callbackHandle = HDdll.hdScheduleAsynchronous(
                    servoLoopCallback,
                    IntPtr.Zero,
                    HDdll.HD_DEFAULT_SCHEDULER_PRIORITY
                );

                if (callbackHandle == IntPtr.Zero)
                {
                    HDdll.HDErrorInfo err = HDdll.hdGetError();
                    IntPtr errPtr = HDdll.hdGetErrorString(err.ErrorCode);
                    errorMessage = Marshal.PtrToStringAnsi(errPtr);
                    return false;
                }

                HDdll.hdStartScheduler();
                Logger.Log("DeviceManager - Servo loop started");

                errorMessage = null;
                return true;
            }
        }

        private static uint ServoLoopCallback(IntPtr userData)
        {
            if (deviceHandle == HDdll.HD_INVALID_HANDLE || Interlocked.Read(ref isRunningFlag) == 0)
                return HDdll.HD_CALLBACK_DONE;

            try
            {
                HDdll.hdBeginFrame(deviceHandle);

                // Update current state
                var position = arrayPool.Rent(3);
                var transform = arrayPool.Rent(16);
                var buttons = arrayPool.Rent(1);

                HDdll.hdGetDoublev(HDdll.HD_CURRENT_POSITION, position);
                HDdll.hdGetDoublev(HDdll.HD_CURRENT_TRANSFORM, transform);
                HDdll.hdGetDoublev(HDdll.HD_CURRENT_BUTTONS, buttons);

                // Calculate forces
                double[] totalForce = new double[] { 0, 0, 0 };

                // Apply plane constraint force if enabled
                if (Interlocked.Read(ref forceEnabledFlag) == 1 && planeConstraintEnabled)
                {
                    var constraintForce = CalculatePlaneConstraintForce(position);
                    totalForce[0] += constraintForce[0];
                    totalForce[1] += constraintForce[1];
                    totalForce[2] += constraintForce[2];
                }

                // Add direct force if enabled
                if (directForceEnabled)
                {
                    lock (deviceLock)
                    {
                        totalForce[0] += directForce[0];
                        totalForce[1] += directForce[1];
                        totalForce[2] += directForce[2];
                    }
                }

                // Apply combined force
                HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, totalForce);

                HDdll.hdEndFrame(deviceHandle);

                // Clean up
                arrayPool.Return(position);
                arrayPool.Return(transform);
                arrayPool.Return(buttons);

                return HDdll.HD_CALLBACK_CONTINUE;
            }
            catch
            {
                return HDdll.HD_CALLBACK_DONE;
            }
        }

        private static double[] CalculatePlaneConstraintForce(double[] position)
        {
            Logger.Log("=== BEGIN CalculatePlaneConstraintForce ===");
            Logger.Log($"Raw device position: [{position[0]:F3}, {position[1]:F3}, {position[2]:F3}]");

            double[] force = new double[] { 0, 0, 0 };

            // Convert device position to our coordinate system
            var devicePos = new Vector3D(
                -position[0],  // Negate X for coordinate system match
                position[2],   // Z becomes Y
                position[1]    // Y becomes Z
            );
            Logger.Log($"Converted device position: X={devicePos.X:F3}, Y={devicePos.Y:F3}, Z={devicePos.Z:F3}");

            // Convert to world space
            Point3d worldPos = new Point3d(devicePos.X, devicePos.Y, devicePos.Z);
            Logger.Log($"Initial world position: X={worldPos.X:F3}, Y={worldPos.Y:F3}, Z={worldPos.Z:F3}");

            Transform deviceToWorld;
            lock (planeConstraintLock)
            {
                Logger.Log($"World to Device Transform: {worldToDevice}");

                if (!worldToDevice.Equals(Transform.Identity))
                {
                    if (!worldToDevice.TryGetInverse(out deviceToWorld))
                    {
                        Logger.Log("ERROR: Could not invert transform");
                        return force;
                    }
                    Logger.Log($"Device to World Transform: {deviceToWorld}");
                    worldPos.Transform(deviceToWorld);
                    Logger.Log($"Transformed world position: X={worldPos.X:F3}, Y={worldPos.Y:F3}, Z={worldPos.Z:F3}");
                }

                Logger.Log($"Plane Origin: X={planeOrigin.X:F3}, Y={planeOrigin.Y:F3}, Z={planeOrigin.Z:F3}");
                Logger.Log($"Plane Normal: X={planeNormal.X:F3}, Y={planeNormal.Y:F3}, Z={planeNormal.Z:F3}");

                // Project point onto plane
                Point3d projectedPoint = new Point3d(worldPos);
                Vector3d toPlane = projectedPoint - planeOrigin;
                double dist = toPlane * planeNormal;
                Logger.Log($"Distance to plane: {dist:F3}");

                projectedPoint -= planeNormal * dist;
                Logger.Log($"Projected Point: X={projectedPoint.X:F3}, Y={projectedPoint.Y:F3}, Z={projectedPoint.Z:F3}");

                // Calculate distance and direction
                Vector3d forceDir = projectedPoint - worldPos;
                double distance = forceDir.Length;
                Logger.Log($"Distance for force calculation: {distance:F3}");

                if (distance < 0.001)
                {
                    Logger.Log("Distance too small, returning zero force");
                    return force;
                }

                forceDir.Unitize();
                Logger.Log($"Normalized Force Direction: X={forceDir.X:F3}, Y={forceDir.Y:F3}, Z={forceDir.Z:F3}");

                // Transform direction back to device space
                if (!worldToDevice.Equals(Transform.Identity))
                {
                    forceDir.Transform(worldToDevice);
                    forceDir.Unitize();
                    Logger.Log($"Force Direction in Device Space: X={forceDir.X:F3}, Y={forceDir.Y:F3}, Z={forceDir.Z:F3}");
                }

                // Calculate force magnitude
                double scale = distance > maxDistanceValue ? maxForceValue : maxForceValue * (distance / maxDistanceValue);
                Logger.Log($"Force scale: {scale:F3} (maxForce={maxForceValue:F3}, maxDistance={maxDistanceValue:F3})");

                // Convert to device coordinates
                force[0] = -forceDir.X * scale;  // Negate X for device space
                force[1] = forceDir.Z * scale;   // Y becomes Z
                force[2] = forceDir.Y * scale;   // Z becomes Y

                Logger.Log($"Final force vector: [{force[0]:F3}, {force[1]:F3}, {force[2]:F3}]");
            }

            Logger.Log("=== END CalculatePlaneConstraintForce ===");
            return force;
        }

        private static void CalculateAndApplyForce(double[] position)
        {
            Logger.Log("=== BEGIN CalculateAndApplyForce ===");
            Logger.Log($"Raw device position: [{position[0]:F3}, {position[1]:F3}, {position[2]:F3}]");

            // Get current target and parameters
            UpdateInterpolatedTarget();
            var target = currentInterpolatedTarget;
            Logger.Log($"Target point: X={target.X:F3}, Y={target.Y:F3}, Z={target.Z:F3}");

            // Convert position to our coordinate system
            var devicePos = new Vector3D(
                -position[0],  // Negate X for coordinate system match
                position[2],   // Z becomes Y
                position[1]    // Y becomes Z
            );
            Logger.Log($"Converted device position: X={devicePos.X:F3}, Y={devicePos.Y:F3}, Z={devicePos.Z:F3}");

            // Calculate direction and distance to target
            var dx = target.X - devicePos.X;
            var dy = target.Y - devicePos.Y;
            var dz = target.Z - devicePos.Z;
            Logger.Log($"Direction vector: dx={dx:F3}, dy={dy:F3}, dz={dz:F3}");

            var distance = Math.Sqrt(dx * dx + dy * dy + dz * dz);
            Logger.Log($"Distance to target: {distance:F3}");

            if (distance < 0.001)
            {
                Logger.Log("Distance too small, applying zero force");
                HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, new double[] { 0, 0, 0 });
                return;
            }

            // Normalize direction and calculate force
            var scale = distance > maxDistanceValue ? maxForceValue : maxForceValue * (distance / maxDistanceValue);
            Logger.Log($"Force scale: {scale:F3} (maxForce={maxForceValue:F3}, maxDistance={maxDistanceValue:F3})");

            var fx = (dx / distance) * scale;
            var fy = (dy / distance) * scale;
            var fz = (dz / distance) * scale;
            Logger.Log($"Scaled force components: fx={fx:F3}, fy={fy:F3}, fz={fz:F3}");

            // Convert back to device coordinates
            var force = new double[]
            {
        -fx,  // Negate X for device space
        fz,   // Y becomes Z
        fy    // Z becomes Y
            };
            Logger.Log($"Final force vector: [{force[0]:F3}, {force[1]:F3}, {force[2]:F3}]");

            HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, force);
            Logger.Log("=== END CalculateAndApplyForce ===");
        }
        public static DeviceState GetCurrentState()
        {
            if (deviceHandle == HDdll.HD_INVALID_HANDLE)
                return new DeviceState();

            // Return copy of current state
            DeviceState state;
            lock (stateLock)
            {
                state = currentState;
                var newPosition = arrayPool.Rent(3);
                var newTransform = arrayPool.Rent(16);

                Array.Copy(state.Position, newPosition, 3);
                Array.Copy(state.Transform, newTransform, 16);

                return new DeviceState
                {
                    Position = newPosition,
                    Transform = newTransform,
                    Buttons = state.Buttons
                };
            }
        }

        public static void UpdateTargetPoint(Vector3D target, bool enable, double maxF, double maxD, bool useInterpolation, double interpolationWindow)
        {
            interpolationEnabled = useInterpolation;
            interpolationTimeWindow = Math.Max(1.0, interpolationWindow);
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
            Interlocked.Exchange(ref forceEnabledFlag, enable ? 1 : 0);
            maxForceValue = maxF;
            maxDistanceValue = maxD;
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

            currentInterpolatedTarget = new Vector3D(
                previousTarget.X + (targetPoint.X - previousTarget.X) * t,
                previousTarget.Y + (targetPoint.Y - previousTarget.Y) * t,
                previousTarget.Z + (targetPoint.Z - previousTarget.Z) * t
            );

            if (t >= 1.0) interpolationEnabled = false;
        }

        public static void UpdatePlaneConstraint(Point3d origin, Vector3d normal, Transform transform, bool enable)
        {
            lock (planeConstraintLock)
            {
                planeConstraintEnabled = enable;
                planeOrigin = origin;
                planeNormal = normal;
                planeNormal.Unitize();
                worldToDevice = transform;
            }
            Logger.Log($"[DeviceManager] Updated plane constraint - Origin: ({(int)origin.X}, {(int)origin.Y}, {(int)origin.Z}), Normal: ({(int)normal.X}, {(int)normal.Y}, {(int)normal.Z})");
        }

        public static void ApplyForce(double[] force, bool enable)
        {
            lock (deviceLock)
            {
                directForceEnabled = enable;
                if (enable && force != null)
                {
                    if (directForce == null) directForce = new double[3];
                    Array.Copy(force, directForce, 3);
                }
                else
                {
                    if (directForce != null)
                        Array.Clear(directForce, 0, directForce.Length);
                }
            }
        }

        public static void Deinitialize()
        {
            lock (deviceLock)
            {
                Interlocked.Exchange(ref isRunningFlag, 0);

                if (deviceHandle != HDdll.HD_INVALID_HANDLE)
                {
                    HDdll.hdStopScheduler();
                    HDdll.hdDisableDevice(deviceHandle);
                    deviceHandle = HDdll.HD_INVALID_HANDLE;
                }

                lock (stateLock)
                {
                    var state = currentState;
                    state.ReturnArrays();
                }
            }
        }
    }
}