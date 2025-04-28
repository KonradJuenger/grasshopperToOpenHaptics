using System;
using System.Collections.Concurrent;
using System.Diagnostics;

namespace ghoh
{
    public static class ServoLogger
    {
        // Structure to hold data for one log entry (device coordinates)
        public struct LogEntry
        {
            public long TimestampMicroseconds;
            public double PosX, PosY, PosZ;
            public double ForceX, ForceY, ForceZ;
        }

        // Thread-safe queue for log entries
        public static ConcurrentQueue<LogEntry> LogQueue = new ConcurrentQueue<LogEntry>();

        // Stopwatch to track elapsed time relative to recording start
        public static Stopwatch RecordingStopwatch = new Stopwatch();

        // Flag to signal the servo loop whether to log (volatile for thread visibility)
        public static volatile bool IsRecording = false;

        // Optional: Counter for diagnostics / checking queue buildup
        // public static System.Threading.Interlocked QueueCount = new System.Threading.Interlocked();

        // Helper to reset state (e.g., when Grasshopper definition closes)
        public static void Reset()
        {
            IsRecording = false;
            RecordingStopwatch.Stop();
            RecordingStopwatch.Reset();
            // Clear the queue
            while (LogQueue.TryDequeue(out _)) { /* discard */ }
            // QueueCount = 0;
        }
    }
}