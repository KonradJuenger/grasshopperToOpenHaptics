using System;
using System.IO;

namespace ghoh
{
    public static class Logger
    {
        private static readonly object lockObj = new object();
        private static bool isLoggingEnabled = false;
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

        public static void LogException(Exception ex)
        {
            Log($"Exception: {ex.Message}{Environment.NewLine}Stack Trace: {ex.StackTrace}");
        }
    }
}
