using Rhino.Geometry;
using System;
using System.IO;

namespace ghoh
{
    public static class Logger
    {
        private static readonly object lockObj = new object();
        private static bool isLoggingEnabled = true;
        private static string logFilePath = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "ghoh_log.txt");

        public static void EnableLogging(bool enable)
        {
            lock (lockObj)
            {
                isLoggingEnabled = enable;
                if (isLoggingEnabled)
                {
                    try
                    {
                        File.WriteAllText(logFilePath, $"{DateTime.Now}: Logging started{Environment.NewLine}");
                    }
                    catch
                    {
                        // Ignore any errors
                    }
                }
            }
        }

        public static void Log(string message)
        {
            if (!isLoggingEnabled)
                return;

            lock (lockObj)
            {
                try
                {
                    File.AppendAllText(logFilePath, $"{DateTime.Now}: {message}{Environment.NewLine}");
                }
                catch
                {
                    // Ignore any errors
                }
            }
        }
        public static void LogTransformData(string context, Transform transform)
        {
            if (transform == null)
            {
                Logger.Log($"{context} - Transform is null");
                return;
            }

            var matrix = transform.ToFloatArray(true);
            Logger.Log($"{context} - Transform Matrix:");
            Logger.Log($"    [{matrix[0]:F3}, {matrix[1]:F3}, {matrix[2]:F3}, {matrix[3]:F3}]");
            Logger.Log($"    [{matrix[4]:F3}, {matrix[5]:F3}, {matrix[6]:F3}, {matrix[7]:F3}]");
            Logger.Log($"    [{matrix[8]:F3}, {matrix[9]:F3}, {matrix[10]:F3}, {matrix[11]:F3}]");
            Logger.Log($"    [{matrix[12]:F3}, {matrix[13]:F3}, {matrix[14]:F3}, {matrix[15]:F3}]");
        }

        public static void LogVectorData(string context, Vector3d vector)
        {
            Logger.Log($"{context} - Vector: X={vector.X:F3}, Y={vector.Y:F3}, Z={vector.Z:F3}");
        }

        public static void LogPointData(string context, Point3d point)
        {
            Logger.Log($"{context} - Point: X={point.X:F3}, Y={point.Y:F3}, Z={point.Z:F3}");
        }

        public static void LogException(Exception ex)
        {
            Log($"Exception: {ex.Message}{Environment.NewLine}Stack Trace: {ex.StackTrace}");
        }
    }
}
