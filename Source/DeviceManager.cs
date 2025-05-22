using ghoh; // Needed for Logger, ServoLogger, ForceManager, HDdll
using System.Buffers;
using System.Runtime.InteropServices;
using System.Threading;
using System;
// using System.IO.Ports; // Original code had this, keep if needed elsewhere, but seems unused here
// using System.Collections.Concurrent; // Original code had this, keep if needed elsewhere, but seems unused here

public static class DeviceManager
{
    private static int deviceHandle = HDdll.HD_INVALID_HANDLE;
    private static readonly object deviceLock = new object();
    private static readonly ArrayPool<double> arrayPool = ArrayPool<double>.Shared;

    private static DeviceState currentState;
    private static readonly object stateLock = new object();
    private static long isRunningFlag; // 0 = false, 1 = true // Original used long, assuming Interlocked access

    // Removed unused filter fields unless they are used elsewhere (comment from original)
    // private static UKF forceFilter;
    // private static bool filterEnabled = true;

    public struct DeviceState
    {
        public double[] Transform;  // 4x4 transform matrix containing position and orientation
        public int Buttons;

        public void ReturnArrays()
        {
            // Original code returned array if not null. Keep this logic.
            if (Transform != null)
            {
                arrayPool.Return(Transform);
                // Add null assignment to prevent potential double return if called again
                Transform = null;
            }
        }
    }

    public struct Vector3D // Copied from original
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
            lock (deviceLock) // Original code locked here
            {
                return deviceHandle;
            }
        }
    }

    public static bool Initialize(out string errorMessage)
    {
        lock (deviceLock) // Keep original locking
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
                // Add logging from original if desired
                // Logger.Log($"ERROR: Device initialization failed: {errorMessage}");
                return false;
            }

            HDdll.hdMakeCurrentDevice(deviceHandle);
            HDdll.hdEnable(HDdll.HD_FORCE_OUTPUT);
            // Logger.Log("Device initialized successfully. Handle: " + deviceHandle); // Optional log

            // Initialize state (original logic)
            Interlocked.Exchange(ref isRunningFlag, 1); // Set flag to running
            lock (stateLock)
            {
                // Initialize only if null or create a new one (original logic)
                if (currentState.Transform == null)
                {
                    currentState = new DeviceState
                    {
                        Transform = arrayPool.Rent(16), // Rent array
                        Buttons = 0 // Initialize buttons
                    };
                    // Initialize the rented array to identity or zeros? Original didn't explicitly.
                    // Let's assume first read in callback will populate it.
                }
                // If not null, assume it's already valid from a previous run?
                // Or should we always rent a new one on Initialize?
                // Sticking to original logic: only rent if null.
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
                // Logger.Log($"ERROR: Failed to schedule servo loop: {errorMessage}"); // Optional log

                // Cleanup if scheduler fails (original logic)
                Interlocked.Exchange(ref isRunningFlag, 0); // Reset flag
                HDdll.hdDisableDevice(deviceHandle);
                deviceHandle = HDdll.HD_INVALID_HANDLE;
                // Also clean up state array if rented?
                lock (stateLock)
                {
                    currentState.ReturnArrays(); // Return array if rented above
                    currentState = new DeviceState { Transform = null, Buttons = 0 }; // Reset state
                }
                return false;
            }

            // Call hdStartScheduler without checking return value (original logic)
            HDdll.hdStartScheduler();
            // Check for error *after* calling if needed (original didn't explicitly check here)
            // HDdll.HDErrorInfo startErr = HDdll.hdGetError();
            // if (startErr.ErrorCode != HDdll.HD_SUCCESS) { ... handle error ... }

            Logger.Log("DeviceManager - Servo loop started"); // Keep log message

            errorMessage = null;
            return true;
        }
    }

    // Maintain original structure of ServoLoopCallback, add logging section
    private static uint ServoLoopCallback(IntPtr userData)
    {
        // Check run flag first (original logic)
        if (Interlocked.Read(ref isRunningFlag) == 0)
            return HDdll.HD_CALLBACK_DONE; // Stop if flag is 0

        // Check handle (original logic)
        int currentDeviceHandle = HDdll.HD_INVALID_HANDLE; // Use local var
        lock (deviceLock)
        { // Match original handle access locking
            currentDeviceHandle = deviceHandle;
        }
        if (currentDeviceHandle == HDdll.HD_INVALID_HANDLE)
            return HDdll.HD_CALLBACK_DONE;

        // Rent temporary arrays (needed for reading state, used in original logic for state update)
        var tempTransform = arrayPool.Rent(16);
        var tempButtons = arrayPool.Rent(1); // Original used array pool

        try // Wrap core logic in try block like original seemed to imply (though wasn't explicit)
        {
            HDdll.hdBeginFrame(currentDeviceHandle); // Use local handle var

            // Get current state into temporary arrays
            HDdll.hdGetDoublev(HDdll.HD_CURRENT_TRANSFORM, tempTransform);
            HDdll.hdGetDoublev(HDdll.HD_CURRENT_BUTTONS, tempButtons);
            int currentButtons = (int)tempButtons[0];

            // Update cached state safely (original logic: update shared state *before* force calculation)
            DeviceState oldState;
            lock (stateLock)
            {
                oldState = currentState; // Get old state to return its array later
                // Update currentState with the *newly rented* temp array references
                currentState = new DeviceState
                {
                    Transform = tempTransform, // Assign the reference
                    Buttons = currentButtons
                };
            }
            // Return the *old* transform array outside the lock (original logic)
            if (oldState.Transform != null && oldState.Transform != tempTransform) // Prevent returning the same array
            {
                oldState.ReturnArrays();
            }

            // Update forces - ForceManager reads the state updated just above (original logic flow)
            ForceManager.UpdateForces();

            // --- *** NEW Servo Logging Section *** ---
            if (ServoLogger.IsRecording) // Check flag set by ForceMonitor
            {
                long timestamp = -1; // Default timestamp
                if (ServoLogger.RecordingStopwatch.IsRunning)
                {
                    // Calculate microseconds from ticks
                    timestamp = ServoLogger.RecordingStopwatch.Elapsed.Ticks / (TimeSpan.TicksPerMillisecond / 1000L);
                }

                // Get the total force calculated by ForceManager in UpdateForces()
                double[] currentTotalForce = ForceManager.GetCurrentForce();

                // Create a new LogEntry using the constructor which copies the data.
                // Pass the transform data read into tempTransform earlier in *this* frame.
                ServoLogger.LogEntry entry = new ServoLogger.LogEntry(timestamp, tempTransform, currentTotalForce);

                // Enqueue the entry (struct copy is enqueued)
                ServoLogger.LogQueue.Enqueue(entry);

                // Note: currentTotalForce array likely doesn't need manual memory management
                // if GetCurrentForce returns a copy or managed array.
            }
            // --- *** END: Servo Logging Section *** ---

            HDdll.hdEndFrame(currentDeviceHandle); // Use local handle var

            // Return the temporary button array (original didn't explicitly return button array, add this)
            arrayPool.Return(tempButtons);

            return HDdll.HD_CALLBACK_CONTINUE; // Continue the loop
        }
        catch (Exception ex) // Basic exception handling (keep simple as per original implication)
        {
            Logger.Log($"Error in servo loop: {ex.Message}");
            // Consider stopping the loop on error
            Interlocked.Exchange(ref isRunningFlag, 0); // Stop loop on error
            ServoLogger.Reset(); // Reset logger state on error

            // Ensure arrays are returned even on error
            if (tempTransform != null && tempTransform != currentState.Transform) arrayPool.Return(tempTransform); // Avoid returning if assigned to currentState
            if (tempButtons != null) arrayPool.Return(tempButtons);

            return HDdll.HD_CALLBACK_DONE; // Stop the loop
        }
        // Removed finally block from previous attempt to stick closer to original structure's lack of explicit finally
        // The try-catch should handle returning arrays on error path now.
        // If successful, tempTransform is now held by currentState and will be returned later.
        // tempButtons is returned just before HD_CALLBACK_CONTINUE.
    }

    // GetCurrentState method from original upload
    public static DeviceState GetCurrentState()
    {
        // Added check for running flag and handle validity based on original structure
        if (Interlocked.Read(ref isRunningFlag) == 0 || deviceHandle == HDdll.HD_INVALID_HANDLE)
            return new DeviceState { Transform = null, Buttons = 0 }; // Return empty/invalid state


        DeviceState stateToReturn;
        lock (stateLock) // Original locked here
        {
            // Make a *copy* of the current state data for the caller (original logic)
            var state = currentState; // Get current state inside lock

            // Check if transform exists before copying (original logic)
            if (state.Transform == null)
            {
                return new DeviceState { Transform = null, Buttons = state.Buttons };
            }

            var newTransform = arrayPool.Rent(16); // Rent a new array for the copy (original logic)
            try // Added try block for safety during copy
            {
                Array.Copy(state.Transform, newTransform, 16); // Original logic copied
            }
            catch (Exception ex) // Handle potential copy error
            {
                Logger.Log($"Error copying transform in GetCurrentState: {ex.Message}");
                arrayPool.Return(newTransform); // Return rented array on error
                return new DeviceState { Transform = null, Buttons = state.Buttons };
            }


            stateToReturn = new DeviceState
            {
                Transform = newTransform, // Give the copy to the caller (original logic)
                Buttons = state.Buttons
            };

            // DO NOT put ServoLogger.Reset() or Interlocked.Exchange here! (comment from original)
        }
        return stateToReturn; // Return the copy (original logic)
    }

    // Deinitialize method from original upload (corrected version was provided)
    // Using the "CORRECTED Deinitialize" logic from the original file content seems best.
    public static void Deinitialize()
    {
        lock (deviceLock) // Original locked here
        {
            // Reset logger state FIRST (from corrected version)
            ServoLogger.Reset();

            // Signal servo loop to stop (from corrected version)
            Interlocked.Exchange(ref isRunningFlag, 0);

            if (deviceHandle != HDdll.HD_INVALID_HANDLE) // Original checked handle
            {
                // Need to ensure scheduler stops before disabling device (from corrected version)
                HDdll.hdStopScheduler(); // Original called this
                ForceManager.Reset(); // Reset forces before disabling (from corrected version)
                HDdll.hdDisableDevice(deviceHandle); // Original called this
                deviceHandle = HDdll.HD_INVALID_HANDLE; // Mark as invalid (from corrected version)
                Logger.Log("DeviceManager Deinitialized."); // From corrected version
            }

            // Clean up state arrays outside device handle check (from corrected version)
            lock (stateLock)
            {
                if (currentState.Transform != null) // From corrected version
                {
                    // var state = currentState; // Not needed, just return directly
                    currentState.ReturnArrays(); // Return the old array (modified ReturnArrays to null the field)
                    currentState = new DeviceState { Transform = null, Buttons = 0 }; // Clear current state
                }
            }
        }
    }
}