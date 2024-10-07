using System;
using System.Runtime.InteropServices;

namespace ghoh
{
    public static class DeviceManager
    {
        // Changed deviceHandle to int
        private static int deviceHandle = HDdll.HD_INVALID_HANDLE;
        private static readonly object lockObj = new object();
        private static IntPtr forceCallbackHandle = IntPtr.Zero;
        private static double[] currentForce = new double[3];
        private static bool forceEnabled = false;

        private static bool loggingEnabled = true; // Set to 'true' to enable logging, 'false' to disable

        // Static constructor to initialize logging
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
                    // Device is already initialized
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
                    Logger.Log($"HD Error Code: {err.ErrorCode}, Internal Error Code: {err.InternalErrorCode}");
                    deviceHandle = HDdll.HD_INVALID_HANDLE;
                    return false;
                }

                // Enable force output
                HDdll.hdEnable(HDdll.HD_FORCE_OUTPUT);
                Logger.Log("Force output enabled.");

                HDdll.hdStartScheduler();
                Logger.Log("Scheduler started.");

                errorMessage = null;
                return true;
            }
        }

        public static void ApplyForce(double[] force, bool enable)
        {
            lock (lockObj)
            {
                Logger.Log($"Applying force: [{force[0]}, {force[1]}, {force[2]}], Enable: {enable}");

                forceEnabled = enable;



                if (forceEnabled)
                {
                    HDdll.hdBeginFrame(deviceHandle);
                    HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, force);
                    HDdll.hdEndFrame(deviceHandle);
                }
                else
                {
                    HDdll.hdBeginFrame(deviceHandle);
                    HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, new double[] { 0, 0, 0 });
                    HDdll.hdEndFrame(deviceHandle);
                }

            }
        }

        public static void Deinitialize()
        {
            lock (lockObj)
            {
                Logger.Log("DeviceManager - Deinitialize called");

                if (deviceHandle != HDdll.HD_INVALID_HANDLE)
                {
                    if (forceCallbackHandle != IntPtr.Zero)
                    {
                        Logger.Log("Unscheduling force callback during deinitialization.");
                        HDdll.hdUnschedule(forceCallbackHandle);
                        // Wait for the callback to complete
                        HDdll.hdWaitForCompletion(forceCallbackHandle, HDdll.HD_WAIT_INFINITE);
                        forceCallbackHandle = IntPtr.Zero;
                    }

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

        static uint ForceCallback(IntPtr pData)
        {
            try
            {
                int deviceHandleLocal;
                double[] force = new double[3];

                lock (lockObj)
                {
                    deviceHandleLocal = deviceHandle;
                    if (!forceEnabled)
                    {
                        Logger.Log("Force not enabled, stopping callback.");
                        return HDdll.HD_CALLBACK_DONE;
                    }
                    Array.Copy(currentForce, force, 3);
                }

                if (deviceHandleLocal == HDdll.HD_INVALID_HANDLE)
                {
                    Logger.Log("Invalid device handle in callback, stopping callback.");
                    return HDdll.HD_CALLBACK_DONE;
                }

                HDdll.hdBeginFrame(deviceHandleLocal);
                Logger.Log("Begin frame in force callback.");

                // Set the force vector
                HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, force);
                Logger.Log($"Force set to: [{force[0]}, {force[1]}, {force[2]}]");

                HDdll.hdEndFrame(deviceHandleLocal);
                Logger.Log("End frame in force callback.");

                // Check for errors
                HDdll.HDErrorInfo error = HDdll.hdGetError();
                if (error.ErrorCode != HDdll.HD_SUCCESS)
                {
                    Logger.Log($"HD Error in force callback: {error.ErrorCode}, Internal Error: {error.InternalErrorCode}");
                    return HDdll.HD_CALLBACK_DONE;
                }

                // Continue applying the force
                return HDdll.HD_CALLBACK_CONTINUE;
            }
            catch (Exception ex)
            {
                Logger.LogException(ex);
                return HDdll.HD_CALLBACK_DONE;
            }
        }
    }
}
