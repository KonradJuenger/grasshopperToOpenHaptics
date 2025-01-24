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

                // Update cached state with proper synchronization
                var newState = new DeviceState
                {
                    Position = position,
                    Transform = transform,
                    Buttons = (int)buttons[0]
                };

                DeviceState oldState;
                lock (stateLock)
                {
                    oldState = currentState;
                    currentState = newState;
                }

                if (oldState.Position != null) arrayPool.Return(oldState.Position);
                if (oldState.Transform != null) arrayPool.Return(oldState.Transform);
                arrayPool.Return(buttons);

                // Calculate and apply forces if enabled
                if (Interlocked.Read(ref forceEnabledFlag) == 1)
                {
                    if (planeConstraintEnabled)
                    {
                        CalculateAndApplyPlaneConstraintForce(position);
                    }
                    else
                    {
                        CalculateAndApplyForce(position);
                    }
                }
                else
                {
                    var zeroForce = new double[] { 0, 0, 0 };
                    HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, zeroForce);
                }

                HDdll.hdEndFrame(deviceHandle);

                return HDdll.HD_CALLBACK_CONTINUE;
            }
            catch
            {
                return HDdll.HD_CALLBACK_DONE;
            }
        }

        private static void CalculateAndApplyPlaneConstraintForce(double[] position)
        {
            // Convert device position to our coordinate system
            var devicePos = new Vector3D(
                -position[0],  // Negate X for coordinate system match
                position[2],   // Z becomes Y
                position[1]    // Y becomes Z
            );

            // Convert to world space
            Point3d worldPos = new Point3d(devicePos.X, devicePos.Y, devicePos.Z);
            Transform deviceToWorld;
            lock (planeConstraintLock)
            {
                if (!worldToDevice.Equals(Transform.Identity))
                {
                    if (!worldToDevice.TryGetInverse(out deviceToWorld))
                    {
                        Logger.Log("[DeviceManager] Error: Could not invert transform");
                        return;
                    }
                    worldPos.Transform(deviceToWorld);
                }

                // Project point onto plane
                Point3d projectedPoint = new Point3d(worldPos);
                Vector3d toPlane = projectedPoint - planeOrigin;
                double dist = toPlane * planeNormal;
                projectedPoint -= planeNormal * dist;

                // Calculate distance and direction
                Vector3d forceDir = projectedPoint - worldPos;
                double distance = forceDir.Length;

                if (distance < 0.001)
                {
                    HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, new double[] { 0, 0, 0 });
                    return;
                }

                forceDir.Unitize();

                // Transform direction back to device space
                if (!worldToDevice.Equals(Transform.Identity))
                {
                    forceDir.Transform(worldToDevice);
                }

                // Calculate force magnitude
                double scale = distance > maxDistanceValue ? maxForceValue : maxForceValue * (distance / maxDistanceValue);

                // Apply force in device coordinates
                var force = new double[]
                {
                    -forceDir.X * scale,  // Negate X for device space
                    forceDir.Z * scale,   // Y becomes Z
                    forceDir.Y * scale    // Z becomes Y
                };

                HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, force);
            }
        }

        private static void CalculateAndApplyForce(double[] position)
        {
            // Get current target and parameters
            UpdateInterpolatedTarget();
            var target = currentInterpolatedTarget;
            var maxForce = maxForceValue;
            var maxDistance = maxDistanceValue;

            // Convert position to our coordinate system
            var devicePos = new Vector3D(
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
                HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, new double[] { 0, 0, 0 });
                return;
            }

            // Normalize direction
            var scale = distance > maxDistance ? maxForce : maxForce * (distance / maxDistance);
            var fx = (dx / distance) * scale;
            var fy = (dy / distance) * scale;
            var fz = (dz / distance) * scale;

            // Convert back to device coordinates
            var force = new double[]
            {
                -fx,  // Negate X for device space
                fz,   // Y becomes Z
                fy    // Z becomes Y
            };

            HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, force);
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
            if (enable)
            {
                HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, force);
            }
            else
            {
                HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, new double[] { 0, 0, 0 });
            }
            Interlocked.Exchange(ref forceEnabledFlag, enable ? 1 : 0);
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