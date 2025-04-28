using ghoh;
using System.Buffers;
using System.Runtime.InteropServices;
using System.Threading;
using System;
using System.IO.Ports;
using System.Collections.Concurrent; // Ensure this is present

public static class DeviceManager
{
    private static int deviceHandle = HDdll.HD_INVALID_HANDLE;
    private static readonly object deviceLock = new object();
    private static readonly ArrayPool<double> arrayPool = ArrayPool<double>.Shared;

    private static DeviceState currentState;
    private static readonly object stateLock = new object();
    private static long isRunningFlag; // 0 = false, 1 = true

    // Removed unused filter fields unless they are used elsewhere
    // private static UKF forceFilter;
    // private static bool filterEnabled = true;

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
            Interlocked.Exchange(ref isRunningFlag, 1); // Set flag to running
            lock (stateLock)
            {
                // Initialize only if null or create a new one
                if (currentState.Transform == null)
                {
                    currentState = new DeviceState
                    {
                        Transform = arrayPool.Rent(16),
                        Buttons = 0 // Initialize buttons
                    };
                }
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
                // Cleanup if scheduler fails
                HDdll.hdDisableDevice(deviceHandle);
                deviceHandle = HDdll.HD_INVALID_HANDLE;
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
        // Check run flag first
        if (Interlocked.Read(ref isRunningFlag) == 0)
            return HDdll.HD_CALLBACK_DONE; // Stop if flag is 0

        // Check handle (less likely to change but good practice)
        if (deviceHandle == HDdll.HD_INVALID_HANDLE)
            return HDdll.HD_CALLBACK_DONE;

        try
        {
            HDdll.hdBeginFrame(deviceHandle);

            // Use temporary arrays for HD calls
            var tempTransform = arrayPool.Rent(16);
            var tempButtons = arrayPool.Rent(1); // Use size 1 for button array

            HDdll.hdGetDoublev(HDdll.HD_CURRENT_TRANSFORM, tempTransform);
            HDdll.hdGetDoublev(HDdll.HD_CURRENT_BUTTONS, tempButtons);

            // Update cached state safely
            DeviceState oldState;
            lock (stateLock)
            {
                oldState = currentState; // Get old state to return its array
                                         // Update currentState with the *new* arrays
                currentState = new DeviceState
                {
                    Transform = tempTransform, // Assign the newly rented array
                    Buttons = (int)tempButtons[0]
                };
            }
            // Return the *old* transform array outside the lock
            if (oldState.Transform != null)
            {
                oldState.ReturnArrays();
            }
            arrayPool.Return(tempButtons); // Return the temporary button array


            // Update forces - ensure ForceManager exists and is initialized elsewhere
            ForceManager.UpdateForces();


            // --- Servo Logging ---
            if (ServoLogger.IsRecording)
            {
                // Get the force that was just set
                double[] currentForce = ForceManager.GetCurrentForce();

                var entry = new ServoLogger.LogEntry();

                if (ServoLogger.RecordingStopwatch.IsRunning)
                {
                    entry.TimestampMicroseconds = ServoLogger.RecordingStopwatch.Elapsed.Ticks / (TimeSpan.TicksPerMillisecond / 1000);
                }
                else { entry.TimestampMicroseconds = -1; } // Indicate stopwatch wasn't running

                // Use the transform data *currently* in currentState
                // Need to read inside lock to ensure consistency? Or use tempTransform?
                // Using tempTransform is safer as currentState might change between here and lock below
                entry.PosX = tempTransform[12];
                entry.PosY = tempTransform[13];
                entry.PosZ = tempTransform[14];

                entry.ForceX = currentForce[0];
                entry.ForceY = currentForce[1];
                entry.ForceZ = currentForce[2];

                ServoLogger.LogQueue.Enqueue(entry);
            }
            // --- END: Servo Logging ---

            HDdll.hdEndFrame(deviceHandle);

            return HDdll.HD_CALLBACK_CONTINUE; // Continue the loop
        }
        catch (Exception ex)
        {
            Logger.Log($"Error in servo loop: {ex.Message}");
            // Consider stopping the loop on error
            Interlocked.Exchange(ref isRunningFlag, 0); // Stop loop on error
            ServoLogger.Reset(); // Reset logger state on error
            return HDdll.HD_CALLBACK_DONE; // Stop the loop
        }
    }

    // CORRECTED GetCurrentState
    public static DeviceState GetCurrentState()
    {
        if (deviceHandle == HDdll.HD_INVALID_HANDLE || Interlocked.Read(ref isRunningFlag) == 0)
            return new DeviceState { Transform = null, Buttons = 0 }; // Return empty/invalid state

        DeviceState stateToReturn;
        lock (stateLock)
        {
            // Make a *copy* of the current state data for the caller
            var state = currentState; // Get current state inside lock

            // Check if transform exists before copying
            if (state.Transform == null)
            {
                return new DeviceState { Transform = null, Buttons = state.Buttons };
            }

            var newTransform = arrayPool.Rent(16); // Rent a new array for the copy
            try
            {
                Array.Copy(state.Transform, newTransform, 16);
            }
            catch (Exception ex)
            {
                Logger.Log($"Error copying transform in GetCurrentState: {ex.Message}");
                arrayPool.Return(newTransform); // Return rented array on error
                return new DeviceState { Transform = null, Buttons = state.Buttons };
            }


            stateToReturn = new DeviceState
            {
                Transform = newTransform, // Give the copy to the caller
                Buttons = state.Buttons
            };

            // DO NOT put ServoLogger.Reset() or Interlocked.Exchange here!
        }
        return stateToReturn; // Return the copy
    }

    // CORRECTED Deinitialize
    public static void Deinitialize()
    {
        lock (deviceLock)
        {
            // Reset logger state FIRST
            ServoLogger.Reset();

            // Signal servo loop to stop
            Interlocked.Exchange(ref isRunningFlag, 0);

            if (deviceHandle != HDdll.HD_INVALID_HANDLE)
            {
                // Need to ensure scheduler stops before disabling device
                HDdll.hdStopScheduler();
                ForceManager.Reset(); // Reset forces before disabling
                HDdll.hdDisableDevice(deviceHandle);
                deviceHandle = HDdll.HD_INVALID_HANDLE; // Mark as invalid
                Logger.Log("DeviceManager Deinitialized.");
            }

            // Clean up state arrays outside device handle check
            lock (stateLock)
            {
                if (currentState.Transform != null)
                {
                    var state = currentState;
                    currentState = new DeviceState { Transform = null, Buttons = 0 }; // Clear current state
                    state.ReturnArrays(); // Return the old array
                }
            }
        }
    }
}