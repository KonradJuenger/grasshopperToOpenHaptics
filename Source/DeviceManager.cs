using System;
using System.Runtime.InteropServices;
using System.Threading;
using System.Buffers;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace ghoh
{
    public class ForceParameters
    {
        public bool Enabled { get; set; }
        public Point3d Target { get; set; }
        public double MaxForce { get; set; }
        public double MaxDistance { get; set; }
        public Transform WorldToDevice { get; set; }

        public ForceParameters()
        {
            Enabled = false;
            Target = Point3d.Origin;
            MaxForce = 1.0;
            MaxDistance = 1.0;
            WorldToDevice = Transform.Identity;
        }
    }

    public static class DeviceManager
    {
        private static int deviceHandle = HDdll.HD_INVALID_HANDLE;
        private static readonly ReaderWriterLockSlim deviceLock = new ReaderWriterLockSlim();
        private static readonly object forceParamsLock = new object();
        private static readonly object stateLock = new object();
        private static DeviceState _cachedState = new DeviceState();
        private static ForceParameters currentForceParams = new ForceParameters();

        // Background loop management
        private static CancellationTokenSource cancellationSource;
        private static Task positionUpdateTask;
        private static Task forceUpdateTask;
        private static volatile bool isRunning = false;

        // Keep arrays pooled for reuse
        private static readonly ArrayPool<double> arrayPool = ArrayPool<double>.Shared;

        public class DeviceState
        {
            public double[] Position;
            public double[] Transform;
            public int Buttons;

            public DeviceState()
            {
                Position = null;
                Transform = null;
                Buttons = 0;
            }

            public void ReturnArrays()
            {
                if (Position != null) arrayPool.Return(Position);
                if (Transform != null) arrayPool.Return(Transform);
            }

            public DeviceState MakeCopy()
            {
                var copy = new DeviceState
                {
                    Position = Position != null ? arrayPool.Rent(3) : null,
                    Transform = Transform != null ? arrayPool.Rent(16) : null,
                    Buttons = this.Buttons
                };

                if (this.Position != null) Array.Copy(this.Position, copy.Position, this.Position.Length);
                if (this.Transform != null) Array.Copy(this.Transform, copy.Transform, this.Transform.Length);

                return copy;
            }
        }

        public static int DeviceHandle
        {
            get
            {
                deviceLock.EnterReadLock();
                try
                {
                    return deviceHandle;
                }
                finally
                {
                    deviceLock.ExitReadLock();
                }
            }
        }

        private static void StartBackgroundLoops()
        {
            if (isRunning) return;

            cancellationSource = new CancellationTokenSource();
            var token = cancellationSource.Token;
            isRunning = true;

            // Position update loop
            positionUpdateTask = Task.Run(async () =>
            {
                while (!token.IsCancellationRequested && deviceHandle != HDdll.HD_INVALID_HANDLE)
                {
                    try
                    {
                        UpdateDeviceState();
                        await Task.Delay(1, token); // 1ms delay
                    }
                    catch (Exception ex)
                    {
                        Logger.LogException(ex);
                    }
                }
            }, token);

            // Force update loop
            forceUpdateTask = Task.Run(async () =>
            {
                while (!token.IsCancellationRequested && deviceHandle != HDdll.HD_INVALID_HANDLE)
                {
                    try
                    {
                        UpdateForce();
                        await Task.Delay(1, token); // 1ms delay
                    }
                    catch (Exception ex)
                    {
                        Logger.LogException(ex);
                    }
                }
            }, token);
        }

        private static void StopBackgroundLoops()
        {
            if (!isRunning) return;

            isRunning = false;
            cancellationSource?.Cancel();

            try
            {
                Task.WaitAll(new[] { positionUpdateTask, forceUpdateTask }, 1000); // Wait up to 1 second
            }
            catch (Exception ex)
            {
                Logger.LogException(ex);
            }

            cancellationSource?.Dispose();
            cancellationSource = null;
        }

        private static void UpdateDeviceState()
        {
            deviceLock.EnterReadLock();
            try
            {
                if (deviceHandle == HDdll.HD_INVALID_HANDLE) return;

                var newState = new DeviceState
                {
                    Position = arrayPool.Rent(3),
                    Transform = arrayPool.Rent(16)
                };

                HDdll.hdBeginFrame(deviceHandle);
                HDdll.hdGetDoublev(HDdll.HD_CURRENT_POSITION, newState.Position);
                HDdll.hdGetDoublev(HDdll.HD_CURRENT_TRANSFORM, newState.Transform);

                double[] buttonState = arrayPool.Rent(1);
                try
                {
                    HDdll.hdGetDoublev(HDdll.HD_CURRENT_BUTTONS, buttonState);
                    newState.Buttons = (int)buttonState[0];
                }
                finally
                {
                    arrayPool.Return(buttonState);
                }

                HDdll.hdEndFrame(deviceHandle);

                // Update cached state
                lock (stateLock)
                {
                    var oldState = _cachedState;
                    _cachedState = newState;
                    oldState.ReturnArrays();
                }
            }
            finally
            {
                deviceLock.ExitReadLock();
            }
        }

        private static void UpdateForce()
        {
            ForceParameters parameters;
            lock (forceParamsLock)
            {
                parameters = currentForceParams;
            }

            if (!parameters.Enabled)
            {
                ApplyForce(new double[] { 0, 0, 0 }, false);
                return;
            }

            // Get current position from cached state
            DeviceState currentState;
            lock (stateLock)
            {
                currentState = _cachedState.MakeCopy();
            }

            try
            {
                if (currentState.Transform == null) return;

                // Calculate force based on position and target
                var devicePosition = new Point3d(
                    -currentState.Transform[12],
                    currentState.Transform[14],
                    currentState.Transform[13]
                );

                Point3d deviceInWorldSpace = devicePosition;
                if (!parameters.WorldToDevice.Equals(Transform.Identity))
                {
                    deviceInWorldSpace.Transform(parameters.WorldToDevice);
                }

                Vector3d totalForce = Vector3d.Zero;
                var directionToTarget = parameters.Target - deviceInWorldSpace;
                var distance = directionToTarget.Length;

                if (distance > 0.001)
                {
                    var normalizedDir = directionToTarget / distance;

                    if (distance > parameters.MaxDistance)
                    {
                        totalForce = normalizedDir * parameters.MaxForce;
                    }
                    else
                    {
                        var scale = parameters.MaxForce * (distance / parameters.MaxDistance);
                        totalForce = normalizedDir * scale;
                    }

                    // Transform force back to device space
                    if (!parameters.WorldToDevice.Equals(Transform.Identity))
                    {
                        Transform deviceToWorld = parameters.WorldToDevice;
                        if (deviceToWorld.TryGetInverse(out deviceToWorld))
                        {
                            totalForce.Transform(deviceToWorld);
                        }
                    }
                }

                // Apply force to device
                double[] forceArray = new double[]
                {
                    -totalForce.X,
                    totalForce.Z,
                    totalForce.Y
                };

                ApplyForce(forceArray, true);
            }
            finally
            {
                currentState.ReturnArrays();
            }
        }

        public static bool Initialize(out string errorMessage)
        {
            deviceLock.EnterWriteLock();
            try
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
                HDdll.hdStartScheduler();

                // Start background loops
                StartBackgroundLoops();

                errorMessage = null;
                return true;
            }
            finally
            {
                deviceLock.ExitWriteLock();
            }
        }

        public static void Deinitialize()
        {
            deviceLock.EnterWriteLock();
            try
            {
                StopBackgroundLoops();

                if (deviceHandle != HDdll.HD_INVALID_HANDLE)
                {
                    HDdll.hdStopScheduler();
                    HDdll.hdDisableDevice(deviceHandle);
                    deviceHandle = HDdll.HD_INVALID_HANDLE;
                }

                // Clear cached state
                lock (stateLock)
                {
                    _cachedState.ReturnArrays();
                    _cachedState = new DeviceState();
                }
            }
            finally
            {
                deviceLock.ExitWriteLock();
            }
        }

        public static DeviceState GetCurrentState()
        {
            lock (stateLock)
            {
                return _cachedState.MakeCopy();
            }
        }

        public static void UpdateForceParameters(ForceParameters parameters)
        {
            lock (forceParamsLock)
            {
                currentForceParams = parameters;
            }
        }

        public static void ApplyForce(double[] force, bool enable)
        {
            deviceLock.EnterWriteLock();
            try
            {
                if (deviceHandle == HDdll.HD_INVALID_HANDLE) return;

                HDdll.hdBeginFrame(deviceHandle);
                HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, force);
                HDdll.hdEndFrame(deviceHandle);
            }
            finally
            {
                deviceLock.ExitWriteLock();
            }
        }
    }
}