using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Diagnostics;
using System.Threading;
using System.Linq;
using System.Globalization;

// ***********************************************************************
// IMPORTANT: Ensure your project ALREADY CONTAINS the definitions for:
// - DeviceManager (with DeviceHandle property and GetCurrentState() method returning DeviceState)
// - DeviceState (with Transform property as double[16] and ReturnArrays() method if needed)
// - ForceManager (with GetCurrentForce() method returning double[3])
// - ServoLogger (with LogQueue<LogEntry>, IsRecording, RecordingStopwatch, Reset() method, and LogEntry class)
// - HDdll (with HD_INVALID_HANDLE constant)
//
// The placeholder dummy classes have been REMOVED from this code block
// to resolve the ambiguity errors you encountered.
// ***********************************************************************

namespace ghoh
{
    public class ghohForceMonitor : GH_Component
    {
        // --- Fields ---
        // Rate limiting for filtered output
        private DateTime lastFilteredUpdateTime = DateTime.MinValue;
        private Point3d lastFilteredPosition = Point3d.Origin;
        private Vector3d lastFilteredForceVector = Vector3d.Zero;
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
        private Vector3d lastTcpOffset = Vector3d.Zero;

        // Camera data fields (for header only)
        private Point3d lastCamLocation = Point3d.Origin;
        private Point3d lastCamTarget = Point3d.Origin;
        private double lastLensLength = 50.0;
        private Vector3d lastCamUp = Vector3d.ZAxis;

        // Runtime messages (for status and debugging)
        private List<string> runtimeMessages = new List<string>();
        // --- End Fields ---

        public ghohForceMonitor() : base(
            "ghohForceMonitor", // Name
            "ServoLogManager", // Nickname
            "Manages servo logging and provides rate-limited live device position and force. Logs raw transform, total force, headers (Camera, WorldX, TCP), and status. Includes debug output to RuntimeLog.", // Description
            "ghoh", // Category
            "device") // Subcategory
        {
        }

        // --- Helper Method for Logging (Adds messages to RuntimeLog output) ---
        private void LogAndAddMessage(GH_RuntimeMessageLevel level, string message)
        {
            string prefix = level switch
            {
                GH_RuntimeMessageLevel.Remark => "[INFO] ",   // Use INFO for Remarks/Debug
                GH_RuntimeMessageLevel.Warning => "[WARN] ",
                GH_RuntimeMessageLevel.Error => "[ERROR] ",
                _ => ""
            };
            // Add timestamp and prefix for clarity in the Grasshopper panel
            string fullMessage = $"{prefix}{DateTime.Now:HH:mm:ss.fff} | {message}";
            runtimeMessages.Add(fullMessage); // Add to list for the RuntimeLog output parameter
            AddRuntimeMessage(level, message); // Add to component message balloons (shows warnings/errors on component)

            // NOTE: Removed call to external persistent Logger.Log here.
            // All messages now go only to the Grasshopper RuntimeLog output.
        }
        // --- End Helper Method ---

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            // --- General Control & Live Output Config ---
            pManager.AddBooleanParameter("Enable", "E", "Enable the component (Required for recording control & live output)", GH_ParamAccess.item, true); // 0
            pManager.AddNumberParameter("UpdateInterval", "I", "Milliseconds between filtered world updates for live output", GH_ParamAccess.item, 100); // 1
            pManager.AddBooleanParameter("EnableFiltered", "F", "Enable filtered world position & force live output", GH_ParamAccess.item, true); // 2

            // --- Recording Control & Setup ---
            pManager.AddTextParameter("LogFolderPath", "Path", "Base directory for recording run subfolders", GH_ParamAccess.item, ""); // 3
            pManager.AddBooleanParameter("StartRecording", "Rec", "True to start logging servo loop, False to stop", GH_ParamAccess.item, false); // 4

            // --- Data for Live Output AND/OR Recording Header ---
            pManager.AddTransformParameter("WorldTransform", "WldX", "[Optional] World transform for live output AND saved in log header for replay.", GH_ParamAccess.item); // 5
            pManager.AddVectorParameter("TCPOffset", "TCP", "[Optional] TCP offset used during recording (saved in log header). NOTE: Not applied to live output.", GH_ParamAccess.item, Vector3d.Zero); // 6

            // --- Data ONLY for Recording Header ---
            pManager.AddPointParameter("camLocation", "CamLoc", "[Optional] Camera Location (saved in log header)", GH_ParamAccess.item, Point3d.Origin); // 7
            pManager.AddPointParameter("camTarget", "CamTgt", "[Optional] Camera Target (saved in log header)", GH_ParamAccess.item, new Point3d(100, 0, 0)); // 8
            pManager.AddNumberParameter("lensLength", "Lens", "[Optional] Camera Lens Length (saved in log header)", GH_ParamAccess.item, 50.0); // 9
            pManager.AddVectorParameter("camUp", "CamUp", "[Optional] Camera Up Vector (saved in log header)", GH_ParamAccess.item, Vector3d.ZAxis); // 10

            // --- Trigger Input ---
            pManager.AddPlaneParameter("TriggerPlane", "Pos", "[Optional] Connect output (e.g., from another component or a Timer->ConstructPlane combo) to trigger updates.", GH_ParamAccess.item); // 11

            // Make optional inputs truly optional
            pManager[5].Optional = true; // WorldTransform
            pManager[6].Optional = true; // TCPOffset
            pManager[7].Optional = true; // CamLoc
            pManager[8].Optional = true; // CamTgt
            pManager[9].Optional = true; // Lens
            pManager[10].Optional = true; // CamUp
            pManager[11].Optional = true; // TriggerPlane
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            // Combined Outputs: Status, Log, Live Filtered Data
            pManager.AddTextParameter("Status", "S", "Component and logging status", GH_ParamAccess.item); // 0
            pManager.AddTextParameter("RuntimeLog", "RL", "Log of runtime messages (INFO, WARN, ERROR)", GH_ParamAccess.list); // 1 - Outputs the runtimeMessages list
            pManager.AddIntegerParameter("QueueCount", "QC", "Approximate number of items waiting in the log queue", GH_ParamAccess.item); // 2
            pManager.AddPointParameter("FilteredWorldPosition", "FWP", "Rate-limited world space device position (transformed, live)", GH_ParamAccess.item); // 3
            pManager.AddVectorParameter("FilteredWorldForce", "FWF", "Rate-limited world space total force vector (transformed, live)", GH_ParamAccess.item); // 4
        }

        protected override void BeforeSolveInstance()
        {
            runtimeMessages.Clear(); // Clear messages at the start of each solution
            base.BeforeSolveInstance();
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string status = "OK"; // Initialize status

            // --- Device Check ---
            // Use your actual HDdll class definition here
            int handle = DeviceManager.DeviceHandle;
            if (handle == HDdll.HD_INVALID_HANDLE) // Use your actual HDdll class constant
            {
                status = "Device not initialized";
                LogAndAddMessage(GH_RuntimeMessageLevel.Error, status);
                EnsureLoggingStopped(); // Make sure logging stops if device disconnects
                DA.SetData(0, status);
                DA.SetDataList(1, runtimeMessages); // Output collected messages
                DA.SetData(2, 0);
                DA.SetData(3, null); // Clear filtered output
                DA.SetData(4, null); // Clear filtered output
                return;
            }
            // --- End Device Check ---

            // --- Input Acquisition ---
            bool enable = true;
            double updateIntervalMs = 100;
            bool enableFiltered = true;
            string logFolderPath = "";
            bool startRecording = false;
            Transform worldTransform = Transform.Identity;
            Vector3d tcpOffset = Vector3d.Zero;
            Point3d camLocation = Point3d.Origin;
            Point3d camTarget = new Point3d(100, 0, 0);
            double lensLength = 50.0;
            Vector3d camUp = Vector3d.ZAxis;
            Plane triggerPlane = Plane.WorldXY; // Variable for the new trigger input

            DA.GetData(0, ref enable);
            DA.GetData(1, ref updateIntervalMs);
            DA.GetData(2, ref enableFiltered);
            DA.GetData(3, ref logFolderPath);
            DA.GetData(4, ref startRecording);
            DA.GetData(5, ref worldTransform);
            DA.GetData(6, ref tcpOffset);
            DA.GetData(7, ref camLocation);
            DA.GetData(8, ref camTarget);
            DA.GetData(9, ref lensLength);
            DA.GetData(10, ref camUp);
            DA.GetData(11, ref triggerPlane);    // Get data for trigger input

            // Store values used for potential recording start OR live output
            lastAppliedTransform = worldTransform;
            lastTcpOffset = tcpOffset;
            lastCamLocation = camLocation;
            lastCamTarget = camTarget;
            lastLensLength = lensLength;
            lastCamUp = camUp;
            // --- End Input Acquisition ---

            // --- Enable/Disable Logic ---
            if (!enable)
            {
                EnsureLoggingStopped();
                status = "Disabled";
                hasNewFilteredData = false;
                DA.SetData(0, status);
                DA.SetDataList(1, runtimeMessages);
                // Use your actual ServoLogger definition here
                DA.SetData(2, ServoLogger.LogQueue.Count);
                DA.SetData(3, null);
                DA.SetData(4, null);
                return;
            }
            // --- End Enable/Disable Logic ---

            // --- Live Data Calculation & Filtering ---
            DateTime now = DateTime.Now;
            Point3d currentWorldPos = Point3d.Unset;
            Vector3d currentWorldForce = Vector3d.Unset;

            // Use your actual DeviceManager and DeviceState definitions here
            var state = DeviceManager.GetCurrentState();
            if (state.Transform != null) // Added null check for state itself
            {
                // Calculate base de
                // vice position in Rhino coordinates
                var baseDevicePos = new Point3d(
                    -state.Transform[12], // M30 -> -X
                     state.Transform[14], // M32 ->  Y
                     state.Transform[13]  // M31 ->  Z
                );

                // --- Force Calculation with Error Handling ---
                try
                {
                    // Use your actual ForceManager definition here
                    double[] currentTotalDeviceForce = ForceManager.GetCurrentForce();
                    if (currentTotalDeviceForce == null || currentTotalDeviceForce.Length != 3)
                    {
                        LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "ForceManager.GetCurrentForce() returned invalid data (null or wrong length). Using Zero vector.");
                        currentWorldForce = Vector3d.Zero;
                    }
                    else
                    {
                        // Convert device force to Rhino vector - CHECK YOUR COORDINATE MAPPING!
                        var currentDeviceForceVector = new Vector3d(
                            -currentTotalDeviceForce[0], // Example: Device X -> Rhino -X
                             currentTotalDeviceForce[2], // Example: Device Z -> Rhino  Y
                             currentTotalDeviceForce[1]  // Example: Device Y -> Rhino  Z
                        );
                        currentWorldForce = currentDeviceForceVector;
                    }
                }
                catch (Exception ex)
                {
                    LogAndAddMessage(GH_RuntimeMessageLevel.Error, $"Error during ForceManager.GetCurrentForce() or initial conversion: {ex.Message}");
                    currentWorldForce = Vector3d.Zero; // Use default on error
                }
                // --- End Force Calculation ---


                // Apply the world transform
                currentWorldPos = baseDevicePos;

                if (!lastAppliedTransform.Equals(Transform.Identity))
                {
                    currentWorldPos.Transform(lastAppliedTransform);
                    try
                    {
                        Vector3d tempForce = currentWorldForce;
                        Transform rotationScale = lastAppliedTransform;
                        rotationScale.M03 = 0.0; rotationScale.M13 = 0.0; rotationScale.M23 = 0.0;
                        tempForce.Transform(rotationScale);
                        currentWorldForce = tempForce;
                    }
                    catch (Exception ex)
                    {
                        LogAndAddMessage(GH_RuntimeMessageLevel.Error, $"Error transforming force vector: {ex.Message}");
                    }
                }

                // Rate Limiting / Filtering
                bool shouldUpdateFiltered = (now - lastFilteredUpdateTime).TotalMilliseconds >= updateIntervalMs;
                if (shouldUpdateFiltered && enableFiltered && currentWorldPos.IsValid && currentWorldForce.IsValid)
                {
                    lastFilteredPosition = currentWorldPos;
                    lastFilteredForceVector = currentWorldForce;
                    lastFilteredUpdateTime = now;
                    hasNewFilteredData = true;
                }

                // Use your actual DeviceState definition here (if ReturnArrays is needed)
                state.ReturnArrays(); // Call if your GetCurrentState implementation requires it
            }
            else
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "Could not get current device state (state or state.Transform is null) for live output.");
                hasNewFilteredData = false;
            }

            // Set Filtered Output Data
            if (enableFiltered && hasNewFilteredData)
            {
                DA.SetData(3, lastFilteredPosition);
                DA.SetData(4, lastFilteredForceVector);
            }
            else if (!enableFiltered)
            {
                DA.SetData(3, null);
                DA.SetData(4, null);
                hasNewFilteredData = false;
            }
            // --- End Live Data ---


            // --- Recording Management & Queue Processing ---
            // Use your actual ServoLogger definitions throughout this section
            recordingBasePath = logFolderPath;

            if (startRecording && !previousStartRecordingState)
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Start recording trigger detected.");
                string startStatus = StartNewRecording();
                status = startStatus;
                if (startStatus.StartsWith("Error") || startStatus.StartsWith("Warning"))
                {
                    LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "[DEBUG] Recording start failed or produced warning. Resetting state.");
                    isManagingRecording = false;
                    if (startStatus.StartsWith("Error")) ServoLogger.Reset();
                }
                else
                {
                    LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Recording started successfully. Setting isManagingRecording = true.");
                    isManagingRecording = true;
                }
            }
            else if (!startRecording && previousStartRecordingState)
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Stop recording trigger detected.");
                EnsureLoggingStopped();
                status = "Recording stopped.";
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Recording stop process completed.");
            }
            previousStartRecordingState = startRecording;

            if (isManagingRecording)
            {
                ProcessLogQueue();

                if ((now - lastFlushTime).TotalMilliseconds >= FLUSH_INTERVAL_MS)
                {
                    FlushLogBuffer();
                    lastFlushTime = now;
                }

                if ((now - lastQueueCheckTime).TotalSeconds >= 1.0)
                {
                    long currentQueueCount = ServoLogger.LogQueue.Count;
                    long processedRate = lastQueueCountCheck - currentQueueCount;
                    if (status == "OK" || status.StartsWith("Recording") || status.StartsWith("Starting"))
                    {
                        status = $"Recording Run {currentRunNumber:D5}... Queue: {currentQueueCount} (~{processedRate}/s proc)";
                    }
                    lastQueueCountCheck = currentQueueCount;
                    lastQueueCheckTime = now;
                }
            }
            else if (ServoLogger.IsRecording && !isManagingRecording)
            {
                if (status == "OK") status = "Servo logging active (externally).";
            }
            else if (status == "OK" && !isManagingRecording && !startRecording)
            {
                status = "Ready to record.";
            }
            // --- End Recording Management & Queue Processing ---

            // --- Set Final Status Outputs ---
            DA.SetData(0, status);
            DA.SetDataList(1, runtimeMessages); // Output all collected messages
            DA.SetData(2, ServoLogger.LogQueue.Count);
            // Filtered outputs (FWP, FWF) are set within the Live Data section
            // --- End Set Final Status Outputs ---
        }


        // --- Start/Stop/Process Recording Methods ---
        // Use your actual ServoLogger definitions in these methods

        private int GetNextRunNumber(string basePath, out string errorMsg)
        {
            int maxNumber = 0;
            errorMsg = null;
            LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] GetNextRunNumber searching in: {basePath}");
            try
            {
                if (!Directory.Exists(basePath))
                {
                    LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] Base path '{basePath}' does not exist. Starting run number at 1.");
                    return 1;
                }
                var subDirs = Directory.GetDirectories(basePath);
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] Found {subDirs.Length} subdirectories.");
                foreach (var dir in subDirs)
                {
                    string dirName = new DirectoryInfo(dir).Name;
                    if (int.TryParse(dirName, out int number))
                    {
                        if (number > maxNumber) maxNumber = number;
                    }
                    else
                    {
                        // LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] Skipping non-numeric directory name: {dirName}");
                    }
                }
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] Max run number found: {maxNumber}. Next run will be {maxNumber + 1}.");
                return maxNumber + 1;
            }
            catch (UnauthorizedAccessException uaEx)
            {
                errorMsg = $"Permission error scanning path '{basePath}': {uaEx.Message}";
                LogAndAddMessage(GH_RuntimeMessageLevel.Error, $"[DEBUG] {errorMsg}");
                return -1;
            }
            catch (Exception ex)
            {
                errorMsg = $"Error scanning path '{basePath}': {ex.Message}";
                LogAndAddMessage(GH_RuntimeMessageLevel.Warning, $"[DEBUG] {errorMsg} Returning 1 as fallback.");
                return 1; // Fallback
            }
        }

        private string StartNewRecording()
        {
            LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] StartNewRecording called.");
            if (isManagingRecording)
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Already managing a recording. Stopping previous one first.");
                EnsureLoggingStopped();
            }
            // Use your actual ServoLogger definition here
            if (ServoLogger.IsRecording)
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "[DEBUG] ServoLogger.IsRecording is already true (externally?). Cannot start new file managed by this component.");
                return "Warning: Servo logging already active.";
            }
            if (string.IsNullOrWhiteSpace(recordingBasePath))
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Error, "[DEBUG] LogFolderPath is not set or is invalid.");
                return "Error: LogFolderPath required.";
            }

            if (!Directory.Exists(recordingBasePath))
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] Base LogFolderPath does not exist. Attempting to create: {recordingBasePath}");
                try
                {
                    Directory.CreateDirectory(recordingBasePath);
                    LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] Base LogFolderPath created successfully.");
                }
                catch (Exception ex)
                {
                    string errMsg = $"Failed to create base LogFolderPath '{recordingBasePath}': {ex.Message}";
                    LogAndAddMessage(GH_RuntimeMessageLevel.Error, $"[DEBUG] {errMsg}");
                    return "Error: Cannot create LogFolderPath.";
                }
            }

            currentRunNumber = GetNextRunNumber(recordingBasePath, out string scanError);
            if (currentRunNumber < 0)
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Error, $"[DEBUG] Failed to determine next run number. {scanError ?? ""}");
                return $"Error: Failed to determine next run number. {scanError ?? ""}";
            }
            string runNumberPadded = currentRunNumber.ToString("D5");
            string subfolderPath = Path.Combine(recordingBasePath, runNumberPadded);
            string filePath = Path.Combine(subfolderPath, "servo_log.csv");
            LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] Determined log path: {filePath}");

            try
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] Attempting to create run directory: {subfolderPath}");
                Directory.CreateDirectory(subfolderPath);
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] Run directory created or already exists.");

                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] Attempting to create StreamWriter (UTF8, buffer 128k) for: {filePath}");
                logWriter = new StreamWriter(filePath, false, Encoding.UTF8, 131072);
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] StreamWriter created successfully.");
                CultureInfo invC = CultureInfo.InvariantCulture;

                // --- Write Header ---
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Writing header lines...");
                logWriter.WriteLine($"#Servo Loop Log - Run {runNumberPadded}");
                logWriter.WriteLine($"#Recording Started: {DateTime.Now:yyyy-MM-dd HH:mm:ss.fff}");
                logWriter.WriteLine("#Camera Setup:");
                logWriter.WriteLine($"#CamLocation: {lastCamLocation.X.ToString("F9", invC)},{lastCamLocation.Y.ToString("F9", invC)},{lastCamLocation.Z.ToString("F9", invC)}");
                logWriter.WriteLine($"#CamTarget: {lastCamTarget.X.ToString("F9", invC)},{lastCamTarget.Y.ToString("F9", invC)},{lastCamTarget.Z.ToString("F9", invC)}");
                logWriter.WriteLine($"#LensLength: {lastLensLength.ToString("F9", invC)}");
                logWriter.WriteLine($"#CamUp: {lastCamUp.X.ToString("F9", invC)},{lastCamUp.Y.ToString("F9", invC)},{lastCamUp.Z.ToString("F9", invC)}");
                logWriter.WriteLine("#WorldTransform (Applied during replay): M00,M01,M02,M03, M10,M11,M12,M13, M20,M21,M22,M23, M30,M31,M32,M33");
                string transformString = string.Format(invC, "{0:F9},{1:F9},{2:F9},{3:F9},{4:F9},{5:F9},{6:F9},{7:F9},{8:F9},{9:F9},{10:F9},{11:F9},{12:F9},{13:F9},{14:F9},{15:F9}",
                    lastAppliedTransform.M00, lastAppliedTransform.M01, lastAppliedTransform.M02, lastAppliedTransform.M03,
                    lastAppliedTransform.M10, lastAppliedTransform.M11, lastAppliedTransform.M12, lastAppliedTransform.M13,
                    lastAppliedTransform.M20, lastAppliedTransform.M21, lastAppliedTransform.M22, lastAppliedTransform.M23,
                    lastAppliedTransform.M30, lastAppliedTransform.M31, lastAppliedTransform.M32, lastAppliedTransform.M33);
                logWriter.WriteLine(transformString);
                logWriter.WriteLine("#TCPOffset (Applied during replay): X,Y,Z");
                logWriter.WriteLine($"{lastTcpOffset.X.ToString("F9", invC)},{lastTcpOffset.Y.ToString("F9", invC)},{lastTcpOffset.Z.ToString("F9", invC)}");
                logWriter.WriteLine("#Data Columns: Time_us, M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33, ForceX, ForceY, ForceZ");
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Header written. Attempting initial flush...");
                logWriter.Flush();
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Initial flush successful.");
                // --- End Header ---
            }
            catch (Exception ex)
            {
                string errMsg = $"Failed to create directory or open/write log file header '{filePath}': {ex.Message}";
                LogAndAddMessage(GH_RuntimeMessageLevel.Error, $"[DEBUG] {errMsg}");
                if (logWriter != null)
                {
                    LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Disposing logWriter due to header writing error.");
                    try { logWriter.Dispose(); } catch { }
                    logWriter = null;
                }
                return $"Error: Failed creating log file/folder. {ex.Message}";
            }

            try
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Resetting log buffer and timers...");
                logBuffer.Clear();
                lastFlushTime = DateTime.Now;
                lastQueueCheckTime = DateTime.Now;
                lastQueueCountCheck = 0;
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Calling ServoLogger.Reset()...");
                ServoLogger.Reset(); // Use your actual ServoLogger definition
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Restarting ServoLogger.RecordingStopwatch...");
                ServoLogger.RecordingStopwatch.Restart(); // Use your actual ServoLogger definition
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Setting ServoLogger.IsRecording = true...");
                ServoLogger.IsRecording = true; // Use your actual ServoLogger definition
                string startMsg = $"Started servo log Run {runNumberPadded} to: {filePath}";
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] {startMsg}");
                return $"Starting Run {runNumberPadded}...";
            }
            catch (Exception ex)
            {
                string errMsg = $"Unexpected error activating logger state: {ex.Message}";
                LogAndAddMessage(GH_RuntimeMessageLevel.Error, $"[DEBUG] {errMsg}");
                if (logWriter != null)
                {
                    LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Disposing logWriter due to logger activation error.");
                    try { logWriter.Dispose(); } catch { }
                    logWriter = null;
                }
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Calling ServoLogger.Reset() due to activation error.");
                ServoLogger.Reset(); // Use your actual ServoLogger definition
                return "Error: Failed activating logger.";
            }
        }

        private void EnsureLoggingStopped()
        {
            string runId = currentRunNumber > 0 ? currentRunNumber.ToString("D5") : "UNKNOWN";
            LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] EnsureLoggingStopped called for Run {runId}. isManagingRecording = {isManagingRecording}");

            if (!isManagingRecording)
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Not currently managing recording. EnsureLoggingStopped exiting.");
                if (logWriter != null)
                {
                    LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "[DEBUG] logWriter was not null but isManagingRecording was false. Attempting cleanup anyway.");
                    CloseLogFile();
                }
                return;
            }

            // Use your actual ServoLogger definition here
            LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Setting ServoLogger.IsRecording = false.");
            ServoLogger.IsRecording = false;
            LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Stopping ServoLogger.RecordingStopwatch.");
            ServoLogger.RecordingStopwatch.Stop();
            double durationSec = ServoLogger.RecordingStopwatch.Elapsed.TotalSeconds;
            LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] Recorded duration: {durationSec:F3}s.");

            isManagingRecording = false;

            LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] Stopping servo log Run {runId}. Processing remaining queue...");

            LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Calling ProcessLogQueue from EnsureLoggingStopped...");
            ProcessLogQueue();
            LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Calling FlushLogBuffer from EnsureLoggingStopped...");
            FlushLogBuffer();

            string finishMsg = $"Finished processing queue for Run {runId}. Duration: {durationSec:F3}s.";
            LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] {finishMsg}");

            if (logWriter != null)
            {
                try
                {
                    LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Attempting to write duration comment to log file...");
                    logWriter.WriteLine($"#Recording Duration: {durationSec:F3} seconds");
                    logWriter.Flush();
                    LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Duration comment written and flushed.");
                }
                catch (Exception ex)
                {
                    LogAndAddMessage(GH_RuntimeMessageLevel.Warning, $"[DEBUG] Could not write duration comment to log for Run {runId}: {ex.Message}");
                }
            }
            else
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "[DEBUG] logWriter was null when trying to write duration.");
            }

            LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Calling CloseLogFile...");
            CloseLogFile();

            LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] EnsureLoggingStopped finished execution.");
        }


        private void ProcessLogQueue()
        {
            if (!isManagingRecording && !previousStartRecordingState) return;

            int processedCount = 0;
            CultureInfo invC = CultureInfo.InvariantCulture;
            int maxItemsPerCycle = 10000;
            // Use your actual ServoLogger definition here
            long initialQueueCount = ServoLogger.LogQueue.Count;

            if (initialQueueCount == 0 && logBuffer.Length == 0) return;

            Stopwatch sw = Stopwatch.StartNew();
            // Use your actual ServoLogger.LogEntry definition here
            while (processedCount < maxItemsPerCycle && ServoLogger.LogQueue.TryDequeue(out ServoLogger.LogEntry entry))
            {
                try
                {
                    if (entry.TransformMatrix == null || entry.TransformMatrix.Length != 16 ||
                        entry.TotalForce == null || entry.TotalForce.Length != 3)
                    {
                        LogAndAddMessage(GH_RuntimeMessageLevel.Warning, $"[DEBUG] Skipping invalid log entry (Timestamp: {entry.TimestampMicroseconds}). Array lengths incorrect or null.");
                        continue;
                    }

                    logBuffer.Append(entry.TimestampMicroseconds.ToString(invC));
                    for (int i = 0; i < 16; i++) { logBuffer.Append(','); logBuffer.Append(entry.TransformMatrix[i].ToString("F9", invC)); }
                    for (int i = 0; i < 3; i++) { logBuffer.Append(','); logBuffer.Append(entry.TotalForce[i].ToString("F9", invC)); }
                    logBuffer.Append('\n');

                    processedCount++;
                }
                catch (FormatException formatEx)
                {
                    LogAndAddMessage(GH_RuntimeMessageLevel.Warning, $"[DEBUG] Formatting error processing log entry (Timestamp: {entry.TimestampMicroseconds}): {formatEx.Message}");
                }
                catch (Exception ex)
                {
                    LogAndAddMessage(GH_RuntimeMessageLevel.Warning, $"[DEBUG] Error processing log entry (Timestamp: {entry.TimestampMicroseconds}): {ex.Message}");
                }
            }
            sw.Stop();

            // Optionally log performance details if needed, can be verbose
            // if (processedCount > 0) {
            //    LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] Processed {processedCount} items from queue in {sw.ElapsedMilliseconds}ms. Buffer length now: {logBuffer.Length}.");
            // }
        }


        private void FlushLogBuffer()
        {
            if (logWriter == null)
            {
                if (logBuffer.Length > 0 && isManagingRecording)
                {
                    LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "[DEBUG] FlushLogBuffer: logWriter is null but buffer has data and recording is managed! Data loss possible.");
                }
                return;
            }
            if (logBuffer.Length == 0) return;

            string runId = currentRunNumber > 0 ? currentRunNumber.ToString("D5") : "UNKNOWN";
            LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] FlushLogBuffer called for Run {runId}. Buffer length: {logBuffer.Length}");
            Stopwatch sw = Stopwatch.StartNew();

            try
            {
                logWriter.Write(logBuffer.ToString());
                logWriter.Flush();
                logBuffer.Clear();
                sw.Stop();
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] Buffer flushed successfully in {sw.ElapsedMilliseconds}ms.");
            }
            catch (ObjectDisposedException)
            {
                sw.Stop();
                LogAndAddMessage(GH_RuntimeMessageLevel.Warning, $"[DEBUG] FlushLogBuffer failed for Run {runId}: Log file writer was already disposed. Cannot flush.");
                isManagingRecording = false;
                logWriter = null;
                logBuffer.Clear();
            }
            catch (IOException ioEx)
            {
                sw.Stop();
                string errMsg = $"FlushLogBuffer failed (I/O Error) for Run {runId}: {ioEx.Message}. Flushing took {sw.ElapsedMilliseconds}ms before error.";
                LogAndAddMessage(GH_RuntimeMessageLevel.Error, $"[DEBUG] {errMsg}");
                EnsureLoggingStopped();
            }
            catch (Exception ex)
            {
                sw.Stop();
                string errMsg = $"FlushLogBuffer failed (General Error) for Run {runId}: {ex.Message}. Flushing took {sw.ElapsedMilliseconds}ms before error.";
                LogAndAddMessage(GH_RuntimeMessageLevel.Error, $"[DEBUG] {errMsg}");
                EnsureLoggingStopped();
            }
        }


        private void CloseLogFile()
        {
            if (logWriter == null)
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] CloseLogFile skipped: logWriter is already null.");
                return;
            }

            string runId = currentRunNumber > 0 ? currentRunNumber.ToString("D5") : "UNKNOWN";
            LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] CloseLogFile called for Run {runId}.");

            try
            {
                if (logBuffer.Length > 0)
                {
                    LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] Writing final buffer content (Length: {logBuffer.Length}) before closing Run {runId}...");
                    logWriter.Write(logBuffer.ToString());
                    logBuffer.Clear();
                }
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Flushing StreamWriter before dispose...");
                logWriter.Flush();
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Disposing StreamWriter...");
                logWriter.Dispose();
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] StreamWriter disposed successfully.");
            }
            catch (ObjectDisposedException)
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] CloseLogFile: Writer was already disposed for Run {runId}.");
            }
            catch (Exception ex)
            {
                string errMsg = $"Error closing log file for Run {runId}: {ex.Message}";
                LogAndAddMessage(GH_RuntimeMessageLevel.Warning, $"[DEBUG] {errMsg}");
            }
            finally
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Setting logWriter = null in finally block.");
                logWriter = null;
                logBuffer.Clear();
            }
        }


        // --- Component Cleanup Methods ---
        public override void RemovedFromDocument(GH_Document document)
        {
            LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Component RemovedFromDocument. Ensuring logging stopped.");
            EnsureLoggingStopped();
            base.RemovedFromDocument(document);
        }

        public override void DocumentContextChanged(GH_Document document, GH_DocumentContext context)
        {
            LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"[DEBUG] DocumentContextChanged: {context}");
            // Use GH_DocumentContext.Close for cleanup when document is closed
            if (context == GH_DocumentContext.Close)
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Document closing. Ensuring logging stopped.");
                EnsureLoggingStopped();
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Calling ServoLogger.Reset() on document close.");
                // Use your actual ServoLogger definition here
                ServoLogger.Reset(); // Reset global logger state if applicable
            }
            else if (context == GH_DocumentContext.Open)
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "[DEBUG] Document opened. Resetting component state.");
                isManagingRecording = false;
                previousStartRecordingState = false;
                if (logWriter != null)
                {
                    LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "[DEBUG] logWriter was not null on document open. Attempting dispose.");
                    try { logWriter.Dispose(); } catch { }
                    logWriter = null;
                }
                currentRunNumber = 0;
                logBuffer.Clear();
                lastAppliedTransform = Transform.Identity;
                lastTcpOffset = Vector3d.Zero;
                hasNewFilteredData = false;
                runtimeMessages.Clear();
            }
            base.DocumentContextChanged(document, context);
        }
        // --- End Cleanup ---

        // --- Component Metadata ---
        protected override System.Drawing.Bitmap Icon => null; // Provide an icon if desired
        public override Guid ComponentGuid => new Guid("A8D1E24F-0C5B-4E9D-9A1F-7E2C89B3D0A0"); // Keep the same GUID
        public override GH_Exposure Exposure => GH_Exposure.primary;
        // --- End Component Metadata ---
    }
} // End namespace ghoh