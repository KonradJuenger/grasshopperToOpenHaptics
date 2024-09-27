using System;
using System.Runtime.InteropServices;

namespace ghoh
{
    public static class DeviceManager
    {
        private static uint deviceHandle = HDdll.HD_BAD_HANDLE;
        private static readonly object lockObj = new object();

        public static uint DeviceHandle
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
                if (deviceHandle != HDdll.HD_BAD_HANDLE)
                {
                    // Device is already initialized
                    errorMessage = null;
                    return true;
                }

                deviceHandle = HDdll.hdInitDevice(HDdll.HD_DEFAULT_DEVICE);
                HDdll.HDErrorInfo err = HDdll.hdGetError();
                if (err.ErrorCode != HDdll.HD_SUCCESS)
                {
                    deviceHandle = HDdll.HD_BAD_HANDLE;
                    IntPtr errPtr = HDdll.hdGetErrorString(err.ErrorCode);
                    errorMessage = Marshal.PtrToStringAnsi(errPtr);
                    return false;
                }

                HDdll.hdStartScheduler();
                errorMessage = null;
                return true;
            }
        }

        public static void Deinitialize()
        {
            lock (lockObj)
            {
                if (deviceHandle != HDdll.HD_BAD_HANDLE)
                {
                    HDdll.hdStopScheduler();
                    HDdll.hdDisableDevice(deviceHandle);
                    deviceHandle = HDdll.HD_BAD_HANDLE;
                }
            }
        }
    }
}
