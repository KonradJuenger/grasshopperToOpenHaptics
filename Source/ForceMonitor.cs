using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic; // Added for List
using System.IO;
using System.Text;
using System.Diagnostics;
using System.Threading;
using System.Linq;
using System.Globalization;

namespace ghoh
{
    public class ghohForceMonitor : GH_Component
    {
        // --- Fields ---
        // Rate limiting
        private DateTime lastUpdateTime = DateTime.MinValue;
        private Vector3d lastFilteredForceVector = Vector3d.Zero;
        private double lastFilteredAmplitude = 0;
        private Point3d lastFilteredPosition = Point3d.Origin;
        private bool hasNewFilteredData = false;

        // Recording Management
        private bool isManagingRecording = false;
        private bool previousStartRecordingState = false;
        private StreamWriter logWriter = null;
        private StringBuilder logBuffer = new StringBuilder(16384);
        private DateTime lastFlushTime = DateTime.MinValue;
        private const int FLUSH_INTERVAL_MS = 1000;
        private string recordingBasePath = "";
        private int currentRunNumber = 0;
        private long lastQueueCountCheck = 0;
        private DateTime lastQueueCheckTime = DateTime.MinValue;
        private Transform lastAppliedTransform = Transform.Identity;

        // *** New Field for collecting runtime messages ***
        private List<string> runtimeMessages = new List<string>();
        // --- End Fields ---

        public ghohForceMonitor() : base(
            "ghohForceMonitor",
            "ServoLogManager",
            "Manages high-frequency servo loop logging with numbered runs (e.g., 00001). Saves world transform. Outputs transformed world-space data and runtime log.", // Updated desc
            "ghoh",
            "device")
        {
        }

        // --- New Helper Method for Logging ---
        /// <summary>
        /// Adds message to component runtime messages and internal list for output.
        /// </summary>
        private void LogAndAddMessage(GH_RuntimeMessageLevel level, string message)
        {
            string prefix = level switch
            {
                GH_RuntimeMessageLevel.Remark => "[Remark] ",
                GH_RuntimeMessageLevel.Warning => "[Warning] ",
                GH_RuntimeMessageLevel.Error => "[ERROR] ",
                _ => ""
            };
            runtimeMessages.Add(prefix + message); // Add to list for output
            AddRuntimeMessage(level, message); // Add for pop-up balloon
        }
        // --- End Helper Method ---


        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enable", "E", "Enable the component", GH_ParamAccess.item, true); // 0
            pManager.AddNumberParameter("UpdateInterval", "I", "Milliseconds between filtered world updates", GH_ParamAccess.item, 100); // 1
            pManager.AddBooleanParameter("EnableFiltered", "F", "Enable filtered world output", GH_ParamAccess.item, true); // 2
            pManager.AddTransformParameter("Transform", "X", "Optional transform for world space outputs (will be saved in log)", GH_ParamAccess.item); // 3
            pManager.AddTextParameter("LogFolderPath", "Path", "Base directory for recording run subfolders", GH_ParamAccess.item, ""); // 4
            pManager.AddBooleanParameter("StartRecording", "Rec", "True to start logging servo loop, False to stop", GH_ParamAccess.item, false); // 5
            pManager[3].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("WorldForceVector", "WF", "Current world space force vector (transformed)", GH_ParamAccess.item); // 0
            pManager.AddNumberParameter("WorldAmplitude", "WA", "Current world space force magnitude (transformed)", GH_ParamAccess.item); // 1
            pManager.AddPointParameter("WorldPosition", "WP", "Current world space device position (transformed)", GH_ParamAccess.item); // 2
            pManager.AddVectorParameter("FilteredWorldForce", "FWF", "Rate-limited world space force vector (transformed)", GH_ParamAccess.item); // 3
            pManager.AddNumberParameter("FilteredWorldAmplitude", "FWA", "Rate-limited world space force magnitude (transformed)", GH_ParamAccess.item); // 4
            pManager.AddPointParameter("FilteredWorldPosition", "FP", "Rate-limited world space device position (transformed)", GH_ParamAccess.item); // 5 - Corrected Abbreviation
            pManager.AddTextParameter("Status", "S", "Component and logging status", GH_ParamAccess.item); // 6
            // *** New Output Parameter ***
            pManager.AddTextParameter("RuntimeLog", "RL", "Log of runtime messages (warnings, errors, remarks)", GH_ParamAccess.list); // 7
        }

        // Clear messages before each solution calculation
        protected override void BeforeSolveInstance()
        {
            runtimeMessages.Clear();
            base.BeforeSolveInstance();
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string status = "OK"; // Initialize status for this run

            // --- Device Check & Input Acquisition ---
            int handle = DeviceManager.DeviceHandle;
            if (handle == HDdll.HD_INVALID_HANDLE)
            {
                status = "Device not initialized";
                LogAndAddMessage(GH_RuntimeMessageLevel.Error, status);
                EnsureLoggingStopped();
                DA.SetData(6, status); // Set status output
                DA.SetDataList(7, runtimeMessages); // Set message log output
                return;
            }
            bool enable = true; double updateIntervalMs = 100; bool enableFiltered = true;
            Transform additionalTransform = Transform.Identity; string logFolderPath = ""; bool startRecording = false;

            // Use TryGetData for better error handling if inputs are missing unexpectedly
            if (!DA.GetData(0, ref enable)) { /* Handle missing input if needed */ }
            if (!DA.GetData(1, ref updateIntervalMs)) { /* Handle missing input */ }
            if (!DA.GetData(2, ref enableFiltered)) { /* Handle missing input */ }
            DA.GetData(3, ref additionalTransform); // Optional, will use Identity if missing
            lastAppliedTransform = additionalTransform;
            if (!DA.GetData(4, ref logFolderPath)) { /* Handle missing input */ }
            if (!DA.GetData(5, ref startRecording)) { /* Handle missing input */ }


            if (!enable)
            {
                EnsureLoggingStopped();
                status = "Disabled";
                DA.SetData(0, Vector3d.Zero); DA.SetData(1, 0.0); DA.SetData(2, Point3d.Origin);
                hasNewFilteredData = false;
                DA.SetData(6, status);
                DA.SetDataList(7, runtimeMessages); // Output any messages generated during disable/stop
                return;
            }
            // --- End Device Check & Input ---

            // Update path immediately
            recordingBasePath = logFolderPath;

            // --- Recording State Management ---
            if (startRecording && !previousStartRecordingState)
            {
                string startStatus = StartNewRecording();
                status = startStatus; // Report status from starting attempt
                if (startStatus.StartsWith("Error") || startStatus.StartsWith("Warning"))
                {
                    isManagingRecording = false;
                    if (startStatus.StartsWith("Error")) ServoLogger.Reset();
                }
            }
            else if (!startRecording && previousStartRecordingState)
            {
                EnsureLoggingStopped(); // This method now uses LogAndAddMessage
                status = "Recording stopped.";
            }
            previousStartRecordingState = startRecording;
            // --- End Recording State Management ---


            // --- World Space Data / Filtered Output / Log Queue Processing ---
            var state = DeviceManager.GetCurrentState();
            if (state.Transform != null)
            {
                Point3d worldSpacePos = new Point3d(-state.Transform[12], state.Transform[14], state.Transform[13]);
                Vector3d worldForceVector = Vector3d.Zero; double worldAmplitude = 0;
                if (!lastAppliedTransform.Equals(Transform.Identity)) { worldSpacePos.Transform(lastAppliedTransform); }
                state.ReturnArrays();
                DA.SetData(0, worldForceVector); DA.SetData(1, worldAmplitude); DA.SetData(2, worldSpacePos);

                DateTime nowFiltered = DateTime.Now;
                bool shouldUpdateFiltered = (nowFiltered - lastUpdateTime).TotalMilliseconds >= updateIntervalMs;
                if (shouldUpdateFiltered && enableFiltered)
                {
                    lastFilteredForceVector = worldForceVector; lastFilteredAmplitude = worldAmplitude; lastFilteredPosition = worldSpacePos;
                    lastUpdateTime = nowFiltered; hasNewFilteredData = true;
                }
                if (enableFiltered && hasNewFilteredData)
                {
                    DA.SetData(3, lastFilteredForceVector); DA.SetData(4, lastFilteredAmplitude); DA.SetData(5, lastFilteredPosition);
                }
            }
            else
            {
                string waitingStatus = "Waiting for device state...";
                if (status == "OK") status = waitingStatus; // Don't overwrite error/warning status
                LogAndAddMessage(GH_RuntimeMessageLevel.Warning, waitingStatus);
                DA.SetData(0, Vector3d.Zero); DA.SetData(1, 0.0); DA.SetData(2, Point3d.Origin);
                DA.SetData(3, null); DA.SetData(4, null); DA.SetData(5, null);
            }

            DateTime now = DateTime.Now;
            if (isManagingRecording)
            {
                ProcessLogQueue();
                if ((now - lastFlushTime).TotalMilliseconds >= FLUSH_INTERVAL_MS) { FlushLogBuffer(); lastFlushTime = now; }
                if ((now - lastQueueCheckTime).TotalSeconds >= 1.0)
                {
                    long currentQueueCount = ServoLogger.LogQueue.Count; long processedRate = Math.Max(0, currentQueueCount - lastQueueCountCheck);
                    // Update status ONLY if no error/warning has occurred
                    if (status == "OK" || status.StartsWith("Recording") || status.StartsWith("Starting"))
                    {
                        status = $"Recording Run {currentRunNumber:D5}... Queue: {currentQueueCount} (~{processedRate}/s)";
                    }
                    lastQueueCountCheck = currentQueueCount; lastQueueCheckTime = now;
                }
            }
            else if (ServoLogger.IsRecording && !isManagingRecording)
            {
                if (status == "OK") status = "Servo logging active (managed elsewhere).";
            }
            else if (status == "OK" && !isManagingRecording)
            {
                status = "Ready to record.";
            }
            // --- End World Space / Filtered / Queue Processing ---

            // --- Set Final Outputs ---
            DA.SetData(6, status);
            DA.SetDataList(7, runtimeMessages); // Output the collected messages
            // --- End Set Final Outputs ---
        }


        /// <summary>
        /// Scans the base path for existing numbered directories (e.g., "00001")
        /// and returns the next available number. Returns -1 on critical error.
        /// Uses LogAndAddMessage for errors/warnings.
        /// </summary>
        private int GetNextRunNumber(string basePath, out string errorMsg)
        {
            int maxNumber = 0;
            errorMsg = null;
            Logger.Log($"GetNextRunNumber: Scanning base path '{basePath}'"); // Keep internal log
            try
            {
                if (!Directory.Exists(basePath))
                {
                    LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"Base path '{basePath}' does not exist. Starting run number at 1.");
                    return 1;
                }
                var subDirs = Directory.GetDirectories(basePath);
                foreach (var dir in subDirs)
                {
                    string dirName = new DirectoryInfo(dir).Name;
                    if (int.TryParse(dirName, out int number))
                    {
                        if (number > maxNumber) maxNumber = number;
                    }
                    else { /* Skip non-numeric */ }
                }
                int nextNumber = maxNumber + 1;
                return nextNumber;
            }
            catch (UnauthorizedAccessException uaEx)
            {
                errorMsg = $"Permission error scanning path '{basePath}': {uaEx.Message}";
                Logger.Log($"ERROR in GetNextRunNumber: {errorMsg}");
                LogAndAddMessage(GH_RuntimeMessageLevel.Error, errorMsg); // Use helper
                return -1;
            }
            catch (Exception ex)
            {
                errorMsg = $"Error scanning path '{basePath}': {ex.Message}";
                Logger.Log($"ERROR in GetNextRunNumber: {errorMsg}");
                LogAndAddMessage(GH_RuntimeMessageLevel.Warning, errorMsg + " Returning 1 as fallback."); // Use helper
                return 1;
            }
        }


        /// <summary>
        /// Starts a new recording run. Uses LogAndAddMessage for errors/warnings/remarks.
        /// </summary>
        private string StartNewRecording()
        {
            if (isManagingRecording) { Logger.Log("StartNewRecording: Was managing, stopping first."); EnsureLoggingStopped(); }
            if (ServoLogger.IsRecording)
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "Servo logging seems already active. Cannot start new file.");
                return "Warning: Servo logging already active.";
            }

            if (string.IsNullOrEmpty(recordingBasePath))
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Error, "LogFolderPath is not set.");
                return "Error: LogFolderPath required.";
            }
            if (!Directory.Exists(recordingBasePath))
            {
                try { Directory.CreateDirectory(recordingBasePath); }
                catch (Exception ex) { string errMsg = $"Failed to create LogFolderPath '{recordingBasePath}': {ex.Message}"; LogAndAddMessage(GH_RuntimeMessageLevel.Error, errMsg); Logger.Log($"ERROR: {errMsg}"); return "Error: Cannot create LogFolderPath."; }
            }

            currentRunNumber = GetNextRunNumber(recordingBasePath, out string scanError);
            if (currentRunNumber < 0) { return $"Error: Failed to determine next run number. {scanError ?? ""}"; } // Message already added by GetNextRunNumber
            string runNumberPadded = currentRunNumber.ToString("D5");
            string subfolderPath = Path.Combine(recordingBasePath, runNumberPadded);
            string filePath = Path.Combine(subfolderPath, "servo_log.csv");

            try
            {
                Directory.CreateDirectory(subfolderPath);
                logWriter = new StreamWriter(filePath, false, Encoding.UTF8, 65536);
                // Write Headers
                logWriter.WriteLine("#servo loop log, values are device space:");
                logWriter.WriteLine($"#recording started: {DateTime.Now:yyyy:MM:dd -- HH:mm:ss}");
                logWriter.WriteLine($"#WorldTransform:M00,M01,M02,M03,M10,M11,M12,M13,M20,M21,M22,M23,M30,M31,M32,M33");
                string transformString = string.Format(CultureInfo.InvariantCulture, "{0:F9},{1:F9},{2:F9},{3:F9},{4:F9},{5:F9},{6:F9},{7:F9},{8:F9},{9:F9},{10:F9},{11:F9},{12:F9},{13:F9},{14:F9},{15:F9}",
                    lastAppliedTransform.M00, lastAppliedTransform.M01, lastAppliedTransform.M02, lastAppliedTransform.M03, lastAppliedTransform.M10, lastAppliedTransform.M11, lastAppliedTransform.M12, lastAppliedTransform.M13,
                    lastAppliedTransform.M20, lastAppliedTransform.M21, lastAppliedTransform.M22, lastAppliedTransform.M23, lastAppliedTransform.M30, lastAppliedTransform.M31, lastAppliedTransform.M32, lastAppliedTransform.M33);
                logWriter.WriteLine(transformString);
                logWriter.WriteLine("#time_us,device_pos_x,device_pos_y,device_pos_z,device_force_x,device_force_y,device_force_z");
                logWriter.Flush();
            }
            catch (Exception ex)
            {
                string errMsg = $"Failed to create directory or open/write log file '{filePath}': {ex.Message}";
                LogAndAddMessage(GH_RuntimeMessageLevel.Error, errMsg); Logger.Log($"ERROR: {errMsg}");
                if (logWriter != null) { try { logWriter.Dispose(); } catch { } logWriter = null; }
                return $"Error: Failed creating log file/folder. {ex.Message}";
            }

            try
            {
                isManagingRecording = true; logBuffer.Clear(); lastFlushTime = DateTime.Now; lastQueueCheckTime = DateTime.Now; lastQueueCountCheck = 0;
                ServoLogger.Reset(); ServoLogger.RecordingStopwatch.Restart(); ServoLogger.IsRecording = true;
                string startMsg = $"Started servo log Run {runNumberPadded} to: {filePath}";
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, startMsg);
                return $"Starting Run {runNumberPadded}...";
            }
            catch (Exception ex)
            {
                string errMsg = $"Unexpected error activating logger state: {ex.Message}";
                LogAndAddMessage(GH_RuntimeMessageLevel.Error, errMsg); Logger.Log($"ERROR: {errMsg}");
                isManagingRecording = false; if (logWriter != null) { try { logWriter.Dispose(); } catch { } logWriter = null; }
                ServoLogger.Reset();
                return "Error: Failed activating logger.";
            }
        }


        /// <summary>
        /// Stops logging. Uses LogAndAddMessage for remarks.
        /// </summary>
        private void EnsureLoggingStopped()
        {
            if (!isManagingRecording) return;
            string runId = currentRunNumber > 0 ? currentRunNumber.ToString("D5") : "UNKNOWN";
            Logger.Log($"EnsureLoggingStopped: Stopping Run {runId}");
            isManagingRecording = false;
            ServoLogger.IsRecording = false; ServoLogger.RecordingStopwatch.Stop();
            LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"Stopping servo log Run {runId}. Processing remaining queue...");
            ProcessLogQueue(); // Process remaining queue
            string finishMsg = $"Finished processing queue for Run {runId}."; // Get message before closing file potentially logs more
            CloseLogFile(); // Close file
            LogAndAddMessage(GH_RuntimeMessageLevel.Remark, finishMsg + " Log closed."); // Add message after closing
        }


        /// <summary>
        /// Dequeues entries. Uses LogAndAddMessage for formatting errors.
        /// </summary>
        private void ProcessLogQueue()
        {
            if (!isManagingRecording && !previousStartRecordingState) return;
            int processedCount = 0;
            while (ServoLogger.LogQueue.TryDequeue(out ServoLogger.LogEntry entry))
            {
                try
                {
                    logBuffer.AppendFormat(CultureInfo.InvariantCulture, "{0},{1:F6},{2:F6},{3:F6},{4:F6},{5:F6},{6:F6}\n",
                         entry.TimestampMicroseconds, entry.PosX, entry.PosY, entry.PosZ, entry.ForceX, entry.ForceY, entry.ForceZ);
                    processedCount++;
                }
                catch (Exception ex)
                {
                    // Log formatting error to runtime messages as well
                    LogAndAddMessage(GH_RuntimeMessageLevel.Warning, $"Error formatting log data: {ex.Message}");
                }
            }
            // Optional: Log internal processing counts
            // if (processedCount > 0) Logger.Log($"ProcessLogQueue: Dequeued and buffered {processedCount} items.");
        }


        /// <summary>
        /// Flushes buffer. Uses LogAndAddMessage for errors.
        /// </summary>
        private void FlushLogBuffer()
        {
            if (!isManagingRecording || logWriter == null || logBuffer.Length == 0) return;
            try
            {
                int length = logBuffer.Length; logWriter.Write(logBuffer.ToString()); logWriter.Flush(); logBuffer.Clear();
                // Optional internal log: Logger.Log($"FlushLogBuffer: Flushed {length} characters for Run {currentRunNumber:D5}.");
            }
            catch (Exception ex)
            {
                string errMsg = $"Failed to write to log file for Run {currentRunNumber:D5}: {ex.Message}";
                LogAndAddMessage(GH_RuntimeMessageLevel.Error, errMsg); Logger.Log($"ERROR FlushLogBuffer: {errMsg}");
                EnsureLoggingStopped(); // Attempt graceful stop on write failure
            }
        }


        /// <summary>
        /// Closes file. Uses LogAndAddMessage for errors.
        /// </summary>
        private void CloseLogFile()
        {
            if (logWriter == null) return;
            string runId = currentRunNumber > 0 ? currentRunNumber.ToString("D5") : "UNKNOWN";
            try
            {
                if (logBuffer.Length > 0) { logWriter.Write(logBuffer.ToString()); logBuffer.Clear(); }
                logWriter.Flush(); logWriter.Dispose();
                Logger.Log($"CloseLogFile: Writer disposed for Run {runId}."); // Internal log remark
            }
            catch (Exception ex)
            {
                string errMsg = $"Error closing log file for Run {runId}: {ex.Message}";
                LogAndAddMessage(GH_RuntimeMessageLevel.Warning, errMsg); // Use helper
                Logger.Log($"ERROR CloseLogFile: {errMsg}");
            }
            finally
            {
                logWriter = null; logBuffer.Clear();
                Logger.Log($"CloseLogFile: Writer set null, buffer cleared for Run {runId}."); // Internal log remark
            }
        }


        // --- Component Cleanup Methods ---
        public override void RemovedFromDocument(GH_Document document) { Logger.Log($"RemovedFromDocument: {InstanceGuid}"); EnsureLoggingStopped(); base.RemovedFromDocument(document); }
        public override void DocumentContextChanged(GH_Document document, GH_DocumentContext context)
        {
            if (context == GH_DocumentContext.Close) { Logger.Log($"Doc Context Close: {InstanceGuid}"); EnsureLoggingStopped(); ServoLogger.Reset(); }
            else if (context == GH_DocumentContext.Open) { Logger.Log($"Doc Context Open: {InstanceGuid}"); isManagingRecording = false; previousStartRecordingState = false; logWriter = null; currentRunNumber = 0; logBuffer.Clear(); lastAppliedTransform = Transform.Identity; }
            base.DocumentContextChanged(document, context);
        }
        // --- End Cleanup ---

        protected override System.Drawing.Bitmap Icon => null;
        public override Guid ComponentGuid => new Guid("A8D1E24F-0C5B-4E9D-9A1F-7E2C89B3D0A0"); // Keep GUID
    }
}