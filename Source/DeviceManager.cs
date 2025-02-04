using ghoh;
using System.Buffers;
using System.Runtime.InteropServices;
using System.Threading;
using System;

public static class DeviceManager
{
    private static int deviceHandle = HDdll.HD_INVALID_HANDLE;
    private static readonly object deviceLock = new object();
    private static readonly ArrayPool<double> arrayPool = ArrayPool<double>.Shared;

    private static DeviceState currentState;
    private static readonly object stateLock = new object();
    private static long isRunningFlag; // 0 = false, 1 = true

    private static UKF forceFilter;
    private static bool filterEnabled = true;

    public struct DeviceState
    {
        public double[] Transform;  // 4x4 transform matrix containing position and orientation
        public int Buttons;

        public void ReturnArrays()
        {
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
                    Transform = arrayPool.Rent(16)
                };
            }

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

            // Get transform and buttons
            var transform = arrayPool.Rent(16);
            var buttons = arrayPool.Rent(1);

            HDdll.hdGetDoublev(HDdll.HD_CURRENT_TRANSFORM, transform);
            HDdll.hdGetDoublev(HDdll.HD_CURRENT_BUTTONS, buttons);

            // Update cached state
            var newState = new DeviceState
            {
                Transform = transform,
                Buttons = (int)buttons[0]
            };

            DeviceState oldState;
            lock (stateLock)
            {
                oldState = currentState;
                currentState = newState;
            }

            oldState.ReturnArrays();
            arrayPool.Return(buttons);

            ForceManager.UpdateForces();

            HDdll.hdEndFrame(deviceHandle);

            return HDdll.HD_CALLBACK_CONTINUE;
        }
        catch (Exception ex)
        {
            Logger.Log($"Error in servo loop: {ex.Message}");
            return HDdll.HD_CALLBACK_DONE;
        }
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
            var newTransform = arrayPool.Rent(16);

            Array.Copy(state.Transform, newTransform, 16);

            return new DeviceState
            {
                Transform = newTransform,
                Buttons = state.Buttons
            };
        }
    }

    public static void Deinitialize()
    {
        lock (deviceLock)
        {
            Interlocked.Exchange(ref isRunningFlag, 0);

            if (deviceHandle != HDdll.HD_INVALID_HANDLE)
            {
                ForceManager.Reset();
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