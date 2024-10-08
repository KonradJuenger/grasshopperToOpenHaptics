using System;
using System.Runtime.InteropServices;
using System.Threading;

namespace ghoh
{
    public static class DeviceManager
    {
        private static int deviceHandle = HDdll.HD_INVALID_HANDLE;
        private static readonly object lockObj = new object();
        private static IntPtr forceCallbackHandle = IntPtr.Zero;
        private static double[] currentForce = new double[3];
        private static bool forceEnabled = false;
        private static bool isRunning = false;

        private static bool loggingEnabled = true;

        // Keep a reference to the delegate to prevent garbage collection
        private static HDdll.HDSchedulerCallback forceCallbackDelegate = ForceCallback;

        public struct DeviceState
        {
            public double[] Position;
            public double[] Transform;
            public int Buttons;
        }

        static DeviceManager()
        {
            if (loggingEnabled)
            {
                Logger.EnableLogging(true);
            }
        }

        public static int DeviceHandle
        {
            get
            {
                lock (lockObj)
                {
                    return deviceHandle;
                }
            }
        }

        public static bool Initialize(out string errorMessage)
        {
            lock (lockObj)
            {
                Logger.Log("DeviceManager - Initialize called");

                if (deviceHandle != HDdll.HD_INVALID_HANDLE)
                {
                    Logger.Log("DeviceManager - Device already initialized");
                    errorMessage = null;
                    return true;
                }

                deviceHandle = HDdll.hdInitDevice(HDdll.HD_DEFAULT_DEVICE);
                Logger.Log($"DeviceManager - hdInitDevice returned handle: {deviceHandle}");

                if (deviceHandle == HDdll.HD_INVALID_HANDLE)
                {
                    HDdll.HDErrorInfo err = HDdll.hdGetError();
                    IntPtr errPtr = HDdll.hdGetErrorString(err.ErrorCode);
                    errorMessage = Marshal.PtrToStringAnsi(errPtr);
                    Logger.Log($"Initialization error: {errorMessage}");
                    Logger.Log($"HD Error Code: {err.ErrorCode}, Internal Error: {err.InternalErrorCode}");
                    deviceHandle = HDdll.HD_INVALID_HANDLE;
                    return false;
                }

                // Make the device current
                HDdll.hdMakeCurrentDevice(deviceHandle);

                // Enable force output
                HDdll.hdEnable(HDdll.HD_FORCE_OUTPUT);
                Logger.Log("Force output enabled.");

                HDdll.hdStartScheduler();
                Logger.Log("Scheduler started.");

                // Start the force update loop
                isRunning = true;
                Thread forceThread = new Thread(ForceUpdateLoop);
                forceThread.Start();

                errorMessage = null;
                return true;
            }
        }

        public static void ApplyForce(double[] force, bool enable)
        {
            lock (lockObj)
            {
                Logger.Log($"ApplyForce called with force: [{force[0]}, {force[1]}, {force[2]}], Enable: {enable}");

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
        }

        private static void ForceUpdateLoop()
        {
            while (isRunning)
            {
                try
                {
                    HDdll.hdScheduleSynchronous(ForceCallback, IntPtr.Zero, HDdll.HD_DEFAULT_SCHEDULER_PRIORITY);
                    Thread.Sleep(1); // Small delay to prevent excessive CPU usage
                }
                catch (Exception ex)
                {
                    Logger.LogException(ex);
                }
            }
        }

        private static uint ForceCallback(IntPtr userData)
        {
            try
            {
                Logger.Log("ForceCallback entered");

                if (deviceHandle == HDdll.HD_INVALID_HANDLE)
                {
                    Logger.Log("ForceCallback: Invalid device handle, stopping callback.");
                    return HDdll.HD_CALLBACK_DONE;
                }

                HDdll.hdBeginFrame(deviceHandle);

                if (forceEnabled)
                {
                    HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, currentForce);
                    Logger.Log($"ForceCallback: Force applied: [{currentForce[0]}, {currentForce[1]}, {currentForce[2]}]");
                }
                else
                {
                    double[] zeroForce = new double[3] { 0, 0, 0 };
                    HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, zeroForce);
                    Logger.Log("ForceCallback: Zero force applied.");
                }

                HDdll.hdEndFrame(deviceHandle);

                // Check for errors
                HDdll.HDErrorInfo error = HDdll.hdGetError();
                if (error.ErrorCode != HDdll.HD_SUCCESS)
                {
                    Logger.Log($"ForceCallback: HD Error: {error.ErrorCode}, Internal Error: {error.InternalErrorCode}");
                    return HDdll.HD_CALLBACK_DONE;
                }

                Logger.Log("ForceCallback completed successfully");
                return HDdll.HD_CALLBACK_DONE;
            }
            catch (Exception ex)
            {
                Logger.LogException(ex);
                return HDdll.HD_CALLBACK_DONE;
            }
        }

        public static DeviceState GetDeviceState()
        {
            var state = new DeviceState
            {
                Position = new double[3],
                Transform = new double[16],
                Buttons = 0
            };

            if (deviceHandle == HDdll.HD_INVALID_HANDLE)
            {
                Logger.Log("GetDeviceState called with invalid device handle.");
                return state;
            }

            HDdll.hdScheduleSynchronous((_) =>
            {
                HDdll.hdBeginFrame(deviceHandle);
                HDdll.hdGetDoublev(HDdll.HD_CURRENT_POSITION, state.Position);
                HDdll.hdGetDoublev(HDdll.HD_CURRENT_TRANSFORM, state.Transform);
                double[] buttonState = new double[1];
                HDdll.hdGetDoublev(HDdll.HD_CURRENT_BUTTONS, buttonState);
                state.Buttons = (int)buttonState[0];
                HDdll.hdEndFrame(deviceHandle);

                Logger.Log($"GetDeviceState: Position: [{state.Position[0]}, {state.Position[1]}, {state.Position[2]}], Buttons: {state.Buttons}");

                return HDdll.HD_CALLBACK_DONE;
            }, IntPtr.Zero, HDdll.HD_DEFAULT_SCHEDULER_PRIORITY);

            return state;
        }

        public static void Deinitialize()
        {
            lock (lockObj)
            {
                Logger.Log("DeviceManager - Deinitialize called");

                isRunning = false; // Stop the force update loop

                if (deviceHandle != HDdll.HD_INVALID_HANDLE)
                {
                    HDdll.hdStopScheduler();
                    Logger.Log("Scheduler stopped.");

                    HDdll.hdDisableDevice(deviceHandle);
                    Logger.Log("Device disabled.");

                    deviceHandle = HDdll.HD_INVALID_HANDLE;
                }
                else
                {
                    Logger.Log("DeviceManager - Device already deinitialized");
                }
            }
        }
    }
}