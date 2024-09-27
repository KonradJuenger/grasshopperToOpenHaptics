using System;
using System.Runtime.InteropServices;

namespace ghoh
{
    using HDuint = System.UInt32;   // unsigned int
    using HDboolean = System.Byte;  // unsigned char
    using HDulong = System.UInt64;  // unsigned long
    using HDushort = System.UInt16; // unsigned short
    using HDint = System.Int32;     // int
    using HDfloat = System.Single;  // float
    using HDdouble = System.Double; // double
    using HDlong = System.Int64;    // long
    using HDchar = System.Char;     // char
    using HDerror = System.UInt32;  // unsigned int
    using HDenum = System.UInt32;   // unsigned int
    using HHD = System.UInt32;      // unsigned int

    public static unsafe class HDdll
    {
        public const HDboolean HD_TRUE = 1;
        public const HDboolean HD_FALSE = 0;

        public const HDerror HD_SUCCESS = 0x0000;
        public const string HD_DEFAULT_DEVICE = null;

        // Function errors:
        public const HDerror HD_INVALID_ENUM = 0x0100;
        public const HDerror HD_INVALID_VALUE = 0x0101;
        public const HDerror HD_INVALID_OPERATION = 0x0102;
        public const HDerror HD_INVALID_INPUT_TYPE = 0x0103;
        public const HDerror HD_BAD_HANDLE = 0x0104;
        // Force errors:
        public const HDerror HD_WARM_MOTORS = 0x0200;
        public const HDerror HD_EXCEEDED_MAX_FORCE = 0x0201;
        public const HDerror HD_EXCEEDED_MAX_FORCE_IMPULSE = 0x0202;
        public const HDerror HD_EXCEEDED_MAX_VELOCITY = 0x0203;
        public const HDerror HD_FORCE_ERROR = 0x0204;
        // Device errors:
        public const HDerror HD_DEVICE_FAULT = 0x0300;
        public const HDerror HD_DEVICE_ALREADY_INITIATED = 0x0301;
        public const HDerror HD_COMM_ERROR = 0x0302;
        public const HDerror HD_COMM_CONFIG_ERROR = 0x0303;
        public const HDerror HD_TIMER_ERROR = 0x0304;

        // Scheduler priority ranges:
        public const HDushort HD_MIN_SCHEDULER_PRIORITY = 0;
        public const HDushort HD_MAX_SCHEDULER_PRIORITY = HDushort.MaxValue;
        public const HDushort HD_DEFAULT_SCHEDULER_PRIORITY = (HD_MAX_SCHEDULER_PRIORITY + HD_MIN_SCHEDULER_PRIORITY) / 2;

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

        [StructLayout(LayoutKind.Sequential)]
        public struct HDErrorInfo
        {
            public HDerror ErrorCode;
            public int InternalErrorCode;
            public HHD hHD;
        }

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public unsafe delegate uint HDSchedulerCallback(void* pData);

        // DLL Imports with corrected CallingConvention and marshalling
        [DllImport("hd.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern HHD hdInitDevice([MarshalAs(UnmanagedType.LPStr)] string pConfigName);

        [DllImport("hd.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern void hdDisableDevice(HHD hHD);

        [DllImport("hd.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern HHD hdGetCurrentDevice();

        [DllImport("hd.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern HDErrorInfo hdGetError();

        [DllImport("hd.dll", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr hdGetErrorString(HDerror errorCode);

        [DllImport("hd.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern void hdStartScheduler();

        [DllImport("hd.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern void hdStopScheduler();

        [DllImport("hd.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr hdScheduleAsynchronous(HDSchedulerCallback pCallback,
                                                           void* pUserData,
                                                           HDushort nPriority);

        [DllImport("hd.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern void hdScheduleSynchronous(HDSchedulerCallback pCallback,
                                                        void* pUserData,
                                                        HDushort nPriority);

        [DllImport("hd.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern void hdUnschedule(IntPtr hHandle);

        [DllImport("hd.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern void hdBeginFrame(HHD hHD);

        [DllImport("hd.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern void hdEndFrame(HHD hHD);

        [DllImport("hd.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern void hdGetDoublev(HDenum pname, HDdouble* paramsOut);

        [DllImport("hd.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern void hdSetDoublev(HDenum pname, double* values);

        public const HDulong HD_WAIT_INFINITE = 0xFFFFFFFFFFFFFFFF;

        [DllImport("hd.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern HDboolean hdWaitForCompletion(IntPtr hHandle, HDulong waitCode);

    }
}
