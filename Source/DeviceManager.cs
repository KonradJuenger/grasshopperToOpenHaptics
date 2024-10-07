using System;
using System.Runtime.InteropServices;

namespace ghoh
{
    public static class DeviceManager
    {
        private static int deviceHandle = HDdll.HD_INVALID_HANDLE;
        private static readonly object lockObj = new object();
        private static IntPtr forceCallbackHandle = IntPtr.Zero;
        private static double[] currentForce = new double[3];
        private static bool forceEnabled = false;

        private static bool loggingEnabled = true;

        // Keep references to delegates to prevent garbage collection
        private static HDdll.HDSchedulerCallback forceCallbackDelegate = ForceCallback;
        private static HDdll.HDSchedulerCallback setZeroForceCallbackDelegate = SetZeroForceCallback;

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
                    Logger.Log($"HD Error Code: {err.ErrorCode}, Internal Error Code: {err.InternalErrorCode}");
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

                errorMessage = null;
                return true;
            }
        }

        public static void ApplyForce(double[] force, bool enable)
        {
            IntPtr callbackHandleToUnschedule = IntPtr.Zero;

            lock (lockObj)
            {
                Logger.Log($"Applying force: [{force[0]}, {force[1]}, {force[2]}], Enable: {enable}");

                if (enable)
                {
                    forceEnabled = true;
                    Array.Copy(force, currentForce, 3);

                    if (forceCallbackHandle == IntPtr.Zero)
                    {
                        // Schedule the asynchronous force callback
                        forceCallbackHandle = HDdll.hdScheduleAsynchronous(forceCallbackDelegate, IntPtr.Zero, HDdll.HD_DEFAULT_SCHEDULER_PRIORITY);
                        Logger.Log("Force callback scheduled.");
                    }
                }
                else
                {
                    forceEnabled = false;

                    // Schedule a synchronous callback to set force to zero
                    HDdll.hdScheduleSynchronous(setZeroForceCallbackDelegate, IntPtr.Zero, HDdll.HD_DEFAULT_SCHEDULER_PRIORITY);
                    Logger.Log("SetZeroForceCallback scheduled.");

                    if (forceCallbackHandle != IntPtr.Zero)
                    {
                        callbackHandleToUnschedule = forceCallbackHandle;
                        forceCallbackHandle = IntPtr.Zero;
                        Logger.Log("Force callback handle reset.");
                    }
                }
            }

            // Unschedule the asynchronous force callback outside the lock
            if (callbackHandleToUnschedule != IntPtr.Zero)
            {
                Logger.Log("Unscheduling force callback.");
                HDdll.hdUnschedule(callbackHandleToUnschedule);
                Logger.Log("Force callback unscheduled.");
            }
        }

        public static void Deinitialize()
        {
            lock (lockObj)
            {
                Logger.Log("DeviceManager - Deinitialize called");

                if (deviceHandle != HDdll.HD_INVALID_HANDLE)
                {
                    // Disable force
                    forceEnabled = false;

                    // Schedule a synchronous callback to set force to zero
                    HDdll.hdScheduleSynchronous(setZeroForceCallbackDelegate, IntPtr.Zero, HDdll.HD_DEFAULT_SCHEDULER_PRIORITY);
                    Logger.Log("SetZeroForceCallback scheduled during deinitialization.");

                    if (forceCallbackHandle != IntPtr.Zero)
                    {
                        HDdll.hdUnschedule(forceCallbackHandle);
                        forceCallbackHandle = IntPtr.Zero;
                        Logger.Log("Force callback unscheduled during deinitialization.");
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

        private static uint ForceCallback(IntPtr pData)
        {
            try
            {
                int deviceHandleLocal = deviceHandle;

                if (!forceEnabled)
                {
                    Logger.Log("Force not enabled, stopping ForceCallback.");
                    return HDdll.HD_CALLBACK_DONE;
                }

                if (deviceHandleLocal == HDdll.HD_INVALID_HANDLE)
                {
                    Logger.Log("Invalid device handle in ForceCallback, stopping callback.");
                    return HDdll.HD_CALLBACK_DONE;
                }

                HDdll.hdBeginFrame(deviceHandleLocal);
                Logger.Log("Begin frame in ForceCallback.");

                // Set the force vector
                double[] force = currentForce;
                HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, force);
                Logger.Log($"Force set to: [{force[0]}, {force[1]}, {force[2]}]");

                HDdll.hdEndFrame(deviceHandleLocal);
                Logger.Log("End frame in ForceCallback.");

                // Check for errors
                HDdll.HDErrorInfo error = HDdll.hdGetError();
                if (error.ErrorCode != HDdll.HD_SUCCESS)
                {
                    Logger.Log($"HD Error in ForceCallback: {error.ErrorCode}, Internal Error: {error.InternalErrorCode}");
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

        private static uint SetZeroForceCallback(IntPtr pData)
        {
            try
            {
                int deviceHandleLocal = deviceHandle;
                if (deviceHandleLocal == HDdll.HD_INVALID_HANDLE)
                {
                    Logger.Log("Invalid device handle in SetZeroForceCallback.");
                    return HDdll.HD_CALLBACK_DONE;
                }

                HDdll.hdBeginFrame(deviceHandleLocal);
                Logger.Log("Begin frame in SetZeroForceCallback.");

                double[] zeroForce = new double[3] { 0, 0, 0 };
                HDdll.hdSetDoublev(HDdll.HD_CURRENT_FORCE, zeroForce);
                Logger.Log("Force set to zero in SetZeroForceCallback.");

                HDdll.hdEndFrame(deviceHandleLocal);
                Logger.Log("End frame in SetZeroForceCallback.");

                return HDdll.HD_CALLBACK_DONE;
            }
            catch (Exception ex)
            {
                Logger.LogException(ex);
                return HDdll.HD_CALLBACK_DONE;
            }
        }
    }
}
