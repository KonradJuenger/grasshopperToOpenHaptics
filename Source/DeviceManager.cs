using System;
using System.Runtime.InteropServices;
using System.Threading;
using System.Buffers;

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
                Logger.Log("ServoCallback - Frame started");

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
                    Logger.Log($"ServoCallback - Updated state: Pos({newState.Position[0]}, {newState.Position[1]}, {newState.Position[2]})");
                }

                if (oldState.Position != null) arrayPool.Return(oldState.Position);
                if (oldState.Transform != null) arrayPool.Return(oldState.Transform);
                arrayPool.Return(buttons);

                // Calculate and apply forces if enabled
                if (Interlocked.Read(ref forceEnabledFlag) == 1)
                {
                    CalculateAndApplyForce(position);
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

        private static void CalculateAndApplyForce(double[] position)
        {
            // Get current target and parameters
            var target = targetPoint;
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

        public static void UpdateTargetPoint(Vector3D target, bool enable, double maxF, double maxD)
        {
            targetPoint = target;
            Interlocked.Exchange(ref forceEnabledFlag, enable ? 1 : 0);
            maxForceValue = maxF;
            maxDistanceValue = maxD;
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