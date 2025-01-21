using System;
using System.Runtime.InteropServices;
using System.Threading;
using System.Buffers;

namespace ghoh
{
    public static class DeviceManager
    {
        private static int deviceHandle = HDdll.HD_INVALID_HANDLE;
        private static readonly ReaderWriterLockSlim deviceLock = new ReaderWriterLockSlim();
        private static IntPtr forceCallbackHandle = IntPtr.Zero;
        private static double[] currentForce = new double[3];
        private static bool forceEnabled = false;
        private static bool isRunning = false;
        private static readonly ArrayPool<double> arrayPool = ArrayPool<double>.Shared;

        // Keep a reference to the delegate to prevent garbage collection
        private static HDdll.HDSchedulerCallback forceCallbackDelegate = ForceCallback;

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
                    deviceHandle = HDdll.HD_INVALID_HANDLE;
                    return false;
                }

                HDdll.hdMakeCurrentDevice(deviceHandle);
                HDdll.hdEnable(HDdll.HD_FORCE_OUTPUT);
                HDdll.hdStartScheduler();

                isRunning = true;
                Thread forceThread = new Thread(ForceUpdateLoop);
                forceThread.Start();

                errorMessage = null;
                return true;
            }
            finally
            {
                deviceLock.ExitWriteLock();
            }
        }

        public static void ApplyForce(double[] force, bool enable)
        {
            deviceLock.EnterWriteLock();
            try
            {
                forceEnabled = enable;
                if (enable)
                {
                    Array.Copy(force, currentForce, 3);
                }
                else
                {
                    Array.Clear(currentForce, 0, currentForce.Length);
                }
            }
            finally
            {
                deviceLock.ExitWriteLock();
            }
        }

        private static void ForceUpdateLoop()
        {
            var spinWait = new SpinWait();
            while (isRunning)
            {
                try
                {
                    HDdll.hdScheduleSynchronous(ForceCallback, IntPtr.Zero, HDdll.HD_DEFAULT_SCHEDULER_PRIORITY);
                    spinWait.SpinOnce();
                }
                catch
                {
                    // Logging is disabled as per requirement
                }
            }
        }

        private static uint ForceCallback(IntPtr userData)
        {
            deviceLock.EnterReadLock();
            try
            {
                if (deviceHandle == HDdll.HD_INVALID_HANDLE)
                {
                    return HDdll.HD_CALLBACK_DONE;
                }

                HDdll.hdBeginFrame(deviceHandle);

                if (forceEnabled)
                {
                    HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, currentForce);
                }
                else
                {
                    double[] zeroForce = arrayPool.Rent(3);
                    try
                    {
                        Array.Clear(zeroForce, 0, zeroForce.Length);
                        HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, zeroForce);
                    }
                    finally
                    {
                        arrayPool.Return(zeroForce);
                    }
                }

                HDdll.hdEndFrame(deviceHandle);

                HDdll.HDErrorInfo error = HDdll.hdGetError();
                if (error.ErrorCode != HDdll.HD_SUCCESS)
                {
                    return HDdll.HD_CALLBACK_DONE;
                }

                return HDdll.HD_CALLBACK_DONE;
            }
            finally
            {
                deviceLock.ExitReadLock();
            }
        }

        public static DeviceState GetDeviceState()
        {
            var state = new DeviceState
            {
                Position = arrayPool.Rent(3),
                Transform = arrayPool.Rent(16),
                Buttons = 0
            };

            deviceLock.EnterReadLock();
            try
            {
                if (deviceHandle == HDdll.HD_INVALID_HANDLE)
                {
                    state.ReturnArrays();
                    return new DeviceState();
                }

                double[] buttonState = arrayPool.Rent(1);
                try
                {
                    HDdll.hdScheduleSynchronous((_) =>
                    {
                        HDdll.hdBeginFrame(deviceHandle);
                        HDdll.hdGetDoublev(HDdll.HD_CURRENT_POSITION, state.Position);
                        HDdll.hdGetDoublev(HDdll.HD_CURRENT_TRANSFORM, state.Transform);
                        HDdll.hdGetDoublev(HDdll.HD_CURRENT_BUTTONS, buttonState);
                        state.Buttons = (int)buttonState[0];
                        HDdll.hdEndFrame(deviceHandle);

                        return HDdll.HD_CALLBACK_DONE;
                    }, IntPtr.Zero, HDdll.HD_DEFAULT_SCHEDULER_PRIORITY);
                }
                finally
                {
                    arrayPool.Return(buttonState);
                }

                return state;
            }
            finally
            {
                deviceLock.ExitReadLock();
            }
        }

        public static void Deinitialize()
        {
            deviceLock.EnterWriteLock();
            try
            {
                isRunning = false;

                if (deviceHandle != HDdll.HD_INVALID_HANDLE)
                {
                    HDdll.hdStopScheduler();
                    HDdll.hdDisableDevice(deviceHandle);
                    deviceHandle = HDdll.HD_INVALID_HANDLE;
                }
            }
            finally
            {
                deviceLock.ExitWriteLock();
            }
        }
    }
}