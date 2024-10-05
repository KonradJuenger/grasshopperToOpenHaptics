using System;
using System.Runtime.InteropServices;

namespace ghoh
{
    using HDint = System.Int32;      // int
    using HDuint = System.UInt32;    // unsigned int
    using HDboolean = System.Byte;   // unsigned char (0 or 1)
    using HDulong = System.UInt64;   // unsigned long
    using HDushort = System.UInt16;  // unsigned short
    using HDfloat = System.Single;   // float
    using HDdouble = System.Double;  // double
    using HDlong = System.Int64;     // long
    using HDchar = System.Char;      // char
    using HDerror = System.UInt32;   // unsigned int
    using HDenum = System.UInt32;    // unsigned int
    using HHD = System.Int32;        // Device handle as int

    public static class HDdll
    {
        // Boolean values
        public const HDboolean HD_TRUE = 1;
        public const HDboolean HD_FALSE = 0;

        // Error codes
        public const HDerror HD_SUCCESS = 0x0000;
        public const HHD HD_INVALID_HANDLE = -1;
        public const string HD_DEFAULT_DEVICE = null;

        // Function errors:
        public const HDerror HD_INVALID_ENUM = 0x0100;
        public const HDerror HD_INVALID_VALUE = 0x0101;
        public const HDerror HD_INVALID_OPERATION = 0x0102;
        public const HDerror HD_INVALID_INPUT_TYPE = 0x0103;
        public const HDerror HD_BAD_HANDLE_ERROR = 0x0104;

        // Force errors:
        public const HDerror HD_WARM_MOTORS = 0x0200;
        public const HDerror HD_EXCEEDED_MAX_FORCE = 0x0201;
        public const HDerror HD_EXCEEDED_MAX_FORCE_IMPULSE = 0x0202;
        public const HDerror HD_EXCEEDED_MAX_VELOCITY = 0x0203;
        public const HDerror HD_FORCE_ERROR = 0x0204;

        // Scheduler priority ranges:
        public const HDushort HD_MIN_SCHEDULER_PRIORITY = 0;
        public const HDushort HD_MAX_SCHEDULER_PRIORITY = HDushort.MaxValue;
        public const HDushort HD_DEFAULT_SCHEDULER_PRIORITY = (HDushort)(HD_MAX_SCHEDULER_PRIORITY / 2);

        // Callback codes
        public const uint HD_CALLBACK_DONE = 0;
        public const uint HD_CALLBACK_CONTINUE = 1;

        // Get parameter options:
        public const HDenum HD_CURRENT_BUTTONS = 0x2000;
        public const HDenum HD_CURRENT_SAFETY_SWITCH = 0x2001;
        public const HDenum HD_CURRENT_INKWELL_SWITCH = 0x2002;
        public const HDenum HD_CURRENT_ENCODER_VALUES = 0x2010;
        public const HDenum HD_CURRENT_PINCH_VALUE = 0x2011;
        public const HDenum HD_LAST_PINCH_VALUE = 0x2012;

        // Cartesian space values:
        public const HDenum HD_CURRENT_POSITION = 0x2050;
        public const HDenum HD_CURRENT_VELOCITY = 0x2051;
        public const HDenum HD_CURRENT_TRANSFORM = 0x2052;
        public const HDenum HD_CURRENT_ANGULAR_VELOCITY = 0x2053;
        public const HDenum HD_CURRENT_JACOBIAN = 0x2054;
        public const HDenum HD_CURRENT_FORCE = 0x2700;

        // Enable/Disable capabilities:
        public const HDenum HD_FORCE_OUTPUT = 0x4000;

        // HDErrorInfo struct
        [StructLayout(LayoutKind.Sequential)]
        public struct HDErrorInfo
        {
            public HDerror ErrorCode;
            public HDint InternalErrorCode;
            public HHD hHD;
        }

        // HDSchedulerCallback delegate
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        public delegate uint HDSchedulerCallback(IntPtr pData);

        // DLL Imports with StdCall convention

        // Initializes the haptic device and returns a device handle
        [DllImport("hd.dll", CallingConvention = CallingConvention.StdCall, CharSet = CharSet.Ansi)]
        public static extern HHD hdInitDevice([MarshalAs(UnmanagedType.LPStr)] string pConfigName);

        // Disables the haptic device
        [DllImport("hd.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern void hdDisableDevice(HHD hHD);

        // Gets the current haptic device handle
        [DllImport("hd.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern HHD hdGetCurrentDevice();

        // Retrieves the last error that occurred
        [DllImport("hd.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern HDErrorInfo hdGetError();

        // Gets a string describing the error code
        [DllImport("hd.dll", CallingConvention = CallingConvention.StdCall, CharSet = CharSet.Ansi)]
        public static extern IntPtr hdGetErrorString(HDerror errorCode);

        // Starts the haptic scheduler
        [DllImport("hd.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern void hdStartScheduler();

        // Stops the haptic scheduler
        [DllImport("hd.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern void hdStopScheduler();

        // Schedules an asynchronous callback
        [DllImport("hd.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern IntPtr hdScheduleAsynchronous(HDSchedulerCallback pCallback,
                                                           IntPtr pUserData,
                                                           HDushort nPriority);

        // Schedules a synchronous callback
        [DllImport("hd.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern void hdScheduleSynchronous(HDSchedulerCallback pCallback,
                                                        IntPtr pUserData,
                                                        HDushort nPriority);

        // Unschedules a previously scheduled callback
        [DllImport("hd.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern void hdUnschedule(IntPtr hHandle);

        // Begins a frame for haptic rendering
        [DllImport("hd.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern void hdBeginFrame(HHD hHD);

        // Ends a frame for haptic rendering
        [DllImport("hd.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern void hdEndFrame(HHD hHD);

        // Retrieves double-precision floating-point values from the device
        [DllImport("hd.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern void hdGetDoublev(HDenum pname, [Out] double[] paramsOut);

        // Sets double-precision floating-point values to the device
        [DllImport("hd.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern void hdSetDoublev(HDenum pname, [In] double[] values);

        // Enables a capability on the device
        [DllImport("hd.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern void hdEnable(HDenum cap);

        // Disables a capability on the device
        [DllImport("hd.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern void hdDisable(HDenum cap);

        // Wait options for hdWaitForCompletion
        public const HDulong HD_WAIT_CHECK_STATUS = 0x00000001;
        public const HDulong HD_WAIT_INFINITE = 0xFFFFFFFFFFFFFFFF;

        // Waits for a scheduled callback to complete
        [DllImport("hd.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern HDboolean hdWaitForCompletion(IntPtr hHandle, HDulong timeout);

        // Makes a device the current device for subsequent calls
        [DllImport("hd.dll", CallingConvention = CallingConvention.StdCall)]
        public static extern void hdMakeCurrentDevice(HHD hHD);
    }
}
