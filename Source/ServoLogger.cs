using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Threading; // Added for Interlocked if needed later

namespace ghoh
{
    public static class ServoLogger
    {
        // Structure to hold data for one log entry
        // Now includes the full transform and total force vector
        public struct LogEntry
        {
            public long TimestampMicroseconds;

            // Store the raw 4x4 device transform matrix (OpenHaptics format)
            // Use fixed-size array for simplicity within the struct
            public double[] TransformMatrix; // Expected Length: 16

            // Store the total force vector applied by ForceManager (device coordinates)
            // Use fixed-size array
            public double[] TotalForce;      // Expected Length: 3

            // Constructor can be helpful for initialization
            public LogEntry(long timestamp, double[] transform, double[] force)
            {
                TimestampMicroseconds = timestamp;

                // Ensure arrays are allocated and copy data to prevent reference issues
                TransformMatrix = new double[16];
                if (transform != null && transform.Length == 16)
                {
                    Array.Copy(transform, TransformMatrix, 16);
                }
                // else: TransformMatrix remains initialized to zeros

                TotalForce = new double[3];
                if (force != null && force.Length == 3)
                {
                    Array.Copy(force, TotalForce, 3);
                }
                // else: TotalForce remains initialized to zeros
            }

            // Parameterless constructor might be needed if used in contexts requiring it
            // public LogEntry() {
            //    TimestampMicroseconds = 0;
            //    TransformMatrix = new double[16];
            //    TotalForce = new double[3];
            // }
        }

        // Thread-safe queue for log entries
        public static ConcurrentQueue<LogEntry> LogQueue = new ConcurrentQueue<LogEntry>();

        // Stopwatch to track elapsed time relative to recording start
        public static Stopwatch RecordingStopwatch = new Stopwatch();

        // Flag to signal the servo loop whether to log (volatile for thread visibility)
        public static volatile bool IsRecording = false;

        // Helper to reset state (e.g., when Grasshopper definition closes or recording stops)
        public static void Reset()
        {
            IsRecording = false; // Stop logging signal
            RecordingStopwatch.Stop();
            RecordingStopwatch.Reset();

            // Clear the queue efficiently
            // No need to manually return arrays as they are part of the struct value type
            // (assuming LogEntry arrays are copied, not rented from pool)
            int count = 0;
            while (LogQueue.TryDequeue(out _))
            {
                count++;
                // Optional safety break if queue is enormous
                // if (count > 1000000) break;
            }
            if (count > 0)
            {
                Logger.Log($"ServoLogger.Reset: Cleared {count} entries from queue."); // Internal log
            }
        }
    }
}