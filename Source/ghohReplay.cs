using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Diagnostics;
using System.Globalization;
using System.Text;

namespace ghoh
{
    // Structure to hold parsed log data for one frame/entry
    internal struct ReplayFrameData // Renamed struct
    {
        public long TimeMicroseconds;
        // Store the raw 4x4 device transform matrix (OpenHaptics format) read from log
        public double[] TransformMatrix; // Length 16
        // Store the total force vector read from log (though not directly output)
        public double[] TotalForce;      // Length 3

        // Constructor for initialization
        public ReplayFrameData(long timestamp, double[] transform, double[] force)
        {
            TimeMicroseconds = timestamp;
            // Copy data
            TransformMatrix = new double[16];
            if (transform != null && transform.Length == 16)
                Array.Copy(transform, TransformMatrix, 16);

            TotalForce = new double[3];
            if (force != null && force.Length == 3)
                Array.Copy(force, TotalForce, 3);
        }
        // Add parameterless constructor for completeness if needed elsewhere
        // public ReplayFrameData() {
        //    TimeMicroseconds = 0;
        //    TransformMatrix = new double[16];
        //    TotalForce = new double[3];
        // }
    }

    public class ghohReplay : GH_Component
    {
        // --- Fields ---
        private List<ReplayFrameData> logData = new List<ReplayFrameData>();
        private string currentFilePath = "";
        private bool isRunning = false;
        private bool previousRunState = false;
        private bool previousLoadState = false; // For load button rising edge

        // Parsed header data (Unchanged)
        private Transform recordedWorldTransform = Transform.Identity;
        private Vector3d recordedTCPOffset = Vector3d.Zero;
        private Point3d recordedCamLocation = Point3d.Origin;
        private Point3d recordedCamTarget = Point3d.Origin;
        private double recordedLensLength = 50.0;
        private Vector3d recordedCamUp = Vector3d.ZAxis;

        // Playback state (Unchanged)
        private long startTimeUs = 0;
        private long endTimeUs = 0;
        private int currentFrameNumber = -1;
        private double frameDurationUs = 0;

        // Cached last calculated outputs for display when stopped/idle
        private Plane lastCalculatedDevicePlane = Plane.Unset;
        private Plane lastCalculatedWorldPlane = Plane.Unset;
        private Vector3d lastCalculatedTotalForce = Vector3d.Zero;
        private int lastCalculatedFrameIndex = -1; // Track which frame the cache is for

        // Runtime messages
        private List<string> runtimeMessages = new List<string>();
        // --- End Fields ---

        public ghohReplay() : base(
            "ghohReplay",
            "ReplayServoLog",
            "Replays log data frame by frame or loads frame 0. Outputs header info, total force, and calculated planes.", // Updated desc
            "ghoh",
            "Utility")
        {
        }

        // --- Helper Method for Logging --- (Unchanged)
        private void LogAndAddMessage(GH_RuntimeMessageLevel level, string message) { /* ... */ }

        protected override void BeforeSolveInstance() // (Unchanged)
        {
            runtimeMessages.Clear();
            base.BeforeSolveInstance();
        }


        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("LogFolderPath", "Path", "Base directory for recording run subfolders", GH_ParamAccess.item); // 0
            pManager.AddIntegerParameter("RunNumber", "Num", "The specific run number folder to replay", GH_ParamAccess.item, 1); // 1
            pManager.AddBooleanParameter("Load", "L", "Trigger to load/reload data and show Frame 0", GH_ParamAccess.item, false); // 2 - ADDED
            pManager.AddBooleanParameter("Run", "R", "Set to True to start/continue playback, False to stop/reset", GH_ParamAccess.item, false); // 3
            pManager.AddNumberParameter("FPS", "FPS", "Target resampling Frames Per Second", GH_ParamAccess.item, 30.0); // 4
            pManager.AddTransformParameter("WorldTransform", "WldX", "[Optional] Overrides recorded WorldTransform.", GH_ParamAccess.item); // 5
            pManager.AddVectorParameter("TCPOffset", "TCP", "[Optional] Overrides recorded TCPOffset.", GH_ParamAccess.item); // 6
            pManager[5].Optional = true;
            pManager[6].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            // ORDER: Status, Frame#, Header Info..., Calculated Planes & Force, Log
            pManager.AddTextParameter("Status", "S", "Playback status", GH_ParamAccess.item); // 0
            pManager.AddIntegerParameter("FrameNumber", "FN", "Current playback frame number (-1 if stopped/idle)", GH_ParamAccess.item); // 1
            // Header Data Outputs
            pManager.AddTransformParameter("RecordedWorldTransform", "RecWldX", "WorldTransform read from log.", GH_ParamAccess.item); // 2
            pManager.AddVectorParameter("RecordedTCPOffset", "RecTCP", "TCPOffset read from log.", GH_ParamAccess.item); // 3
            // Camera Outputs
            pManager.AddPointParameter("camLocation", "CamLoc", "Recorded Camera Location.", GH_ParamAccess.item); // 4
            pManager.AddPointParameter("camTarget", "CamTgt", "Recorded Camera Target.", GH_ParamAccess.item); // 5
            pManager.AddNumberParameter("lensLength", "Lens", "Recorded Camera Lens Length.", GH_ParamAccess.item); // 6
            pManager.AddVectorParameter("camUp", "CamUp", "Recorded Camera Up Vector.", GH_ParamAccess.item); // 7
            // Calculated Outputs
            pManager.AddPlaneParameter("DevicePlane", "DevPln", "Calculated device plane (inc. TCP offset).", GH_ParamAccess.item); // 8
            pManager.AddPlaneParameter("WorldPlane", "WldPln", "Calculated world plane (inc. TCP & World transform).", GH_ParamAccess.item); // 9
            pManager.AddVectorParameter("TotalForce", "TF", "Recorded Total Force vector (device coordinates).", GH_ParamAccess.item); // 10 - ADDED
            // Runtime Log Output
            pManager.AddTextParameter("RuntimeLog", "Log", "Log of runtime messages.", GH_ParamAccess.list); // 11
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // --- Input Acquisition ---
            string folderPath = ""; int runNumber = 1; bool loadInput = false; bool run = false; double fps = 30.0;
            Transform inputWorldTransform = Transform.Identity; Vector3d inputTcpOffset = Vector3d.Zero;
            bool worldTransformInputProvided = false; bool tcpOffsetInputProvided = false;
            CultureInfo culture = CultureInfo.InvariantCulture;

            DA.GetData(0, ref folderPath); DA.GetData(1, ref runNumber); DA.GetData(2, ref loadInput); // Load trigger
            DA.GetData(3, ref run); DA.GetData(4, ref fps);
            worldTransformInputProvided = DA.GetData(5, ref inputWorldTransform);
            tcpOffsetInputProvided = DA.GetData(6, ref inputTcpOffset);
            // --- End Input Acquisition ---

            string status = "Idle";
            int outputFrameNumber = -1; // Frame number to output

            if (fps <= 0) { LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "FPS must be > 0. Using 30.0."); fps = 30.0; }
            frameDurationUs = 1000000.0 / fps;

            string runNumberPadded = runNumber.ToString("D5");
            string targetFilePath = Path.Combine(folderPath ?? "", runNumberPadded, "servo_log.csv");

            // --- Detect Load Trigger ---
            bool loadTriggered = loadInput && !previousLoadState;
            previousLoadState = loadInput;
            // --- End Load Trigger ---

            // Determine effective transforms/offsets
            Vector3d effectiveTcpOffset = tcpOffsetInputProvided ? inputTcpOffset : recordedTCPOffset;
            Transform effectiveWorldTransform = worldTransformInputProvided ? inputWorldTransform : recordedWorldTransform;

            // --- Load Data on Trigger ---
            if (loadTriggered)
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "Load triggered.");
                isRunning = false; // Stop playback if running
                currentFrameNumber = -1;
                string loadStatus = LoadLogFile(targetFilePath, culture); // Load data and parse headers
                if (loadStatus.StartsWith("OK") && logData.Count > 0)
                {
                    currentFilePath = targetFilePath;
                    // Calculate Frame 0 data and cache it
                    if (CalculatePlanesAndForce(0, effectiveTcpOffset, effectiveWorldTransform, out lastCalculatedDevicePlane, out lastCalculatedWorldPlane, out lastCalculatedTotalForce))
                    {
                        status = "Loaded Frame 0";
                        lastCalculatedFrameIndex = 0; // Mark cache as valid for frame 0
                        LogAndAddMessage(GH_RuntimeMessageLevel.Remark, status);
                    }
                    else
                    {
                        status = "Load OK, but failed to calculate Frame 0";
                        LogAndAddMessage(GH_RuntimeMessageLevel.Error, status);
                        lastCalculatedFrameIndex = -1; // Mark cache as invalid
                    }
                }
                else if (loadStatus.StartsWith("OK")) // Load OK but no data points
                {
                    status = "Load OK, but no data points found.";
                    LogAndAddMessage(GH_RuntimeMessageLevel.Warning, status);
                    lastCalculatedFrameIndex = -1;
                }
                else // Load failed
                {
                    status = loadStatus; // Error message from LoadLogFile
                    lastCalculatedFrameIndex = -1;
                }
                ExpireSolution(true); // Force update outputs after load attempt
                                      // Set final outputs after load attempt below
            }
            // --- End Load Data ---

            // --- State Management (Run/Stop) ---
            else if (run && !previousRunState) // Start/resume trigger (only if Load not triggered)
            {
                if (logData.Count == 0) // Try loading if data isn't present
                {
                    LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "Run triggered with no data loaded. Attempting load...");
                    string loadStatus = LoadLogFile(targetFilePath, culture);
                    if (!loadStatus.StartsWith("OK"))
                    {
                        status = loadStatus; // Report error
                        isRunning = false;
                    }
                    else if (logData.Count == 0)
                    {
                        status = "Load OK, but no data points found.";
                        LogAndAddMessage(GH_RuntimeMessageLevel.Warning, status);
                        isRunning = false;
                    }
                    else // Load successful
                    {
                        currentFilePath = targetFilePath;
                        LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"Loaded log file: {targetFilePath}");
                    }
                }

                if (logData.Count > 0) // Proceed if data is loaded
                {
                    if (!isRunning)
                    { // Prevent restarting if already running
                        isRunning = true; currentFrameNumber = (currentFrameNumber < 0) ? 0 : currentFrameNumber; // Start from 0 or resume
                        status = $"Running Frame {currentFrameNumber}";
                        LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "Playback starting/resuming.");
                        ExpireSolution(true); // Trigger first frame calculation/resume
                    }
                    else
                    {
                        status = $"Already Running Frame {currentFrameNumber}"; // If Run stays true
                    }
                }
                // If load failed or no data, isRunning remains false, status holds error
            }
            else if (!run && previousRunState) // Stop trigger
            {
                if (isRunning) LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "Playback stopped by user.");
                isRunning = false; currentFrameNumber = -1; status = "Stopped";
                // Keep last calculated frame data in cache
            }
            previousRunState = run; // Update run state *after* checking edges
                                    // --- End State Management ---


            // --- Playback Loop / Update Cache ---
            if (isRunning && logData.Count > 0)
            {
                if (currentFrameNumber < 0) currentFrameNumber = 0; // Should be >= 0 if running
                outputFrameNumber = currentFrameNumber;

                double targetTimeUs = startTimeUs + currentFrameNumber * frameDurationUs;

                // Use >= for finish condition check to handle exact match with last timestamp
                if (targetTimeUs >= endTimeUs + frameDurationUs * 0.1) // Stop if target time passes last entry time (with small tolerance)
                {
                    double playbackDurationSec = (currentFrameNumber > 0) ? (currentFrameNumber * frameDurationUs / 1000000.0) : 0.0;
                    status = $"Finished (Playback: ~{playbackDurationSec:F2}s)";
                    LogAndAddMessage(GH_RuntimeMessageLevel.Remark, status);
                    outputFrameNumber = -1;
                    isRunning = false; // Stop running
                                       // Keep last calculated frame data in cache
                }
                else // Calculate current frame
                {
                    int closestIndex = FindClosestEntryIndex(targetTimeUs);
                    if (closestIndex >= 0)
                    {
                        // Calculate and cache data for this frame
                        if (CalculatePlanesAndForce(closestIndex, effectiveTcpOffset, effectiveWorldTransform, out lastCalculatedDevicePlane, out lastCalculatedWorldPlane, out lastCalculatedTotalForce))
                        {
                            status = $"Running Frame {currentFrameNumber} (LogT: {logData[closestIndex].TimeMicroseconds / 1000.0:F0}ms)";
                            lastCalculatedFrameIndex = closestIndex; // Update cache index
                            currentFrameNumber++; // Advance frame
                            ExpireSolution(true); // Schedule next update
                        }
                        else // Error calculating
                        {
                            status = $"Error calc planes/force for Frame {currentFrameNumber} (Log Index {closestIndex})";
                            LogAndAddMessage(GH_RuntimeMessageLevel.Error, status);
                            outputFrameNumber = -1; isRunning = false; lastCalculatedFrameIndex = -1;
                        }
                    }
                    else // Error finding index
                    {
                        status = "Error finding frame data.";
                        LogAndAddMessage(GH_RuntimeMessageLevel.Error, status);
                        outputFrameNumber = -1; isRunning = false; lastCalculatedFrameIndex = -1;
                    }
                }
            }
            // --- End Playback Loop ---


            // --- Determine final status if not running ---
            if (!isRunning && !loadTriggered) // Update status if idle/stopped/finished and not just loaded
            {
                outputFrameNumber = -1;
                if (status == "Idle")
                { // Only update if truly idle, not stopped/finished
                    if (File.Exists(targetFilePath) && logData.Count > 0 && lastCalculatedFrameIndex >= 0) status = "Ready (Frame 0 loaded)";
                    else if (File.Exists(targetFilePath) && logData.Count > 0 && lastCalculatedFrameIndex < 0) status = "Ready (Data loaded, press Load)";
                    else if (File.Exists(targetFilePath) && logData.Count == 0) status = "Ready (No data points)";
                    else if (!File.Exists(targetFilePath)) status = $"Log file not found: {runNumberPadded}";
                }
                // Keep status if "Stopped", "Finished", or Error
            }
            // --- End Status Update ---


            // --- Set Outputs ---
            DA.SetData(0, status);
            DA.SetData(1, outputFrameNumber);
            // Header Data
            DA.SetData(2, recordedWorldTransform);
            DA.SetData(3, recordedTCPOffset);
            // Camera Data
            DA.SetData(4, recordedCamLocation);
            DA.SetData(5, recordedCamTarget);
            DA.SetData(6, recordedLensLength);
            DA.SetData(7, recordedCamUp);
            // Calculated Outputs (from cache)
            DA.SetData(8, lastCalculatedDevicePlane);
            DA.SetData(9, lastCalculatedWorldPlane);
            DA.SetData(10, lastCalculatedTotalForce);
            // Log
            DA.SetDataList(11, runtimeMessages);
            // --- End Set Outputs ---
        }


        /// <summary>
        /// Calculates Device/World planes and Total Force for a specific frame index.
        /// </summary>
        private bool CalculatePlanesAndForce(int frameIndex, Vector3d tcpOffsetToApply, Transform worldTransformToApply, out Plane devicePlane, out Plane worldPlane, out Vector3d totalForce)
        {
            devicePlane = Plane.Unset;
            worldPlane = Plane.Unset;
            totalForce = Vector3d.Zero; // Default force output

            if (logData == null || frameIndex < 0 || frameIndex >= logData.Count) return false;

            ReplayFrameData entry = logData[frameIndex];
            if (entry.TransformMatrix == null || entry.TransformMatrix.Length != 16) return false;
            if (entry.TotalForce == null || entry.TotalForce.Length != 3) return false; // Check force array

            try
            {
                // 1. Calculate Base Device Plane
                var origin = new Point3d(-entry.TransformMatrix[12], entry.TransformMatrix[14], entry.TransformMatrix[13]);
                var xDirection = new Vector3d(-entry.TransformMatrix[0], entry.TransformMatrix[2], entry.TransformMatrix[1]);
                var yDirection = new Vector3d(-entry.TransformMatrix[4], entry.TransformMatrix[6], entry.TransformMatrix[5]);
                Plane baseDevicePlane = new Plane(origin, xDirection, yDirection);

                // 2. Apply TCP Offset
                devicePlane = baseDevicePlane;
                if (!tcpOffsetToApply.IsZero)
                {
                    Vector3d baseZDirection = Vector3d.CrossProduct(baseDevicePlane.XAxis, baseDevicePlane.YAxis);
                    Vector3d offsetInPlaneSpace = tcpOffsetToApply.X * baseDevicePlane.XAxis + tcpOffsetToApply.Y * baseDevicePlane.YAxis + tcpOffsetToApply.Z * baseZDirection;
                    devicePlane.Origin += offsetInPlaneSpace;
                }

                // 3. Calculate World Plane
                worldPlane = devicePlane;
                if (!worldTransformToApply.Equals(Transform.Identity))
                {
                    worldPlane.Transform(worldTransformToApply);
                }

                // 4. Convert Recorded Force (Device Coordinates) to Rhino Vector
                totalForce = new Vector3d(-entry.TotalForce[0], entry.TotalForce[2], entry.TotalForce[1]);

                return true; // Success
            }
            catch (Exception ex)
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Error, $"Exception calculating planes/force for log index {frameIndex}: {ex.Message}");
                devicePlane = Plane.Unset; worldPlane = Plane.Unset; totalForce = Vector3d.Zero;
                return false; // Indicate failure
            }
        }



        /// <summary>
        /// Resets playback state variables (clears data, resets headers, frame count).
        /// </summary>
        private void ResetPlaybackState()
        {
            logData.Clear();
            recordedWorldTransform = Transform.Identity;
            recordedTCPOffset = Vector3d.Zero;
            recordedCamLocation = Point3d.Origin;
            recordedCamTarget = Point3d.Origin;
            recordedLensLength = 50.0;
            recordedCamUp = Vector3d.ZAxis;
            startTimeUs = 0;
            endTimeUs = 0;
            currentFrameNumber = -1;
        }


        /// <summary>
        /// Loads the specified log file, parses headers (Camera, Transform, TCP), and data.
        /// Uses LogAndAddMessage for feedback. Returns status string.
        /// </summary>
        private string LoadLogFile(string filePath, CultureInfo culture)
        {
            ResetPlaybackState(); // Clear previous data before loading

            bool foundWorldTransformHeader = false; bool foundWorldTransformValues = false;
            bool foundTcpOffsetHeader = false; bool foundTcpOffsetValues = false;
            bool foundCamLocation = false, foundCamTarget = false, foundLensLength = false, foundCamUp = false;
            bool foundDataHeader = false;

            if (!File.Exists(filePath))
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Error, $"File not found: {filePath}");
                return $"Error: File not found."; // Ensure return
            }

            try
            {
                using (StreamReader reader = new StreamReader(filePath))
                {
                    string line; int lineNum = 0;
                    while ((line = reader.ReadLine()) != null)
                    {
                        lineNum++; line = line.Trim();
                        if (string.IsNullOrEmpty(line)) continue;

                        if (line.StartsWith("#"))
                        { // --- Parse Header Lines ---
                            line = line.Substring(1).Trim();
                            if (line.StartsWith("WorldTransform")) foundWorldTransformHeader = true;
                            else if (line.StartsWith("TCPOffset")) foundTcpOffsetHeader = true;
                            else if (line.StartsWith("Data Columns:")) foundDataHeader = true;
                            else if (line.StartsWith("CamLocation:")) ParsePoint3DHeader(line, "CamLocation", ref recordedCamLocation, ref foundCamLocation, lineNum);
                            else if (line.StartsWith("CamTarget:")) ParsePoint3DHeader(line, "CamTarget", ref recordedCamTarget, ref foundCamTarget, lineNum);
                            else if (line.StartsWith("LensLength:")) ParseDoubleHeader(line, "LensLength", ref recordedLensLength, ref foundLensLength, lineNum, culture);
                            else if (line.StartsWith("CamUp:")) ParseVector3DHeader(line, "CamUp", ref recordedCamUp, ref foundCamUp, lineNum, culture);
                            continue; // Skip to next line after processing header line
                        }
                        else if (foundWorldTransformHeader && !foundWorldTransformValues)
                        { // --- Parse World Transform Values ---
                            if (ParseTransformValues(line, ref recordedWorldTransform, lineNum, culture)) foundWorldTransformValues = true;
                            foundWorldTransformHeader = false; // Consume flag regardless of success
                            continue;
                        }
                        else if (foundTcpOffsetHeader && !foundTcpOffsetValues)
                        { // --- Parse TCP Offset Values ---
                            if (ParseVector3DValues(line, ref recordedTCPOffset, lineNum, culture)) foundTcpOffsetValues = true;
                            foundTcpOffsetHeader = false; // Consume flag regardless of success
                            continue;
                        }
                        else
                        { // --- Parse Main Data Line ---
                            string[] values = line.Split(',');
                            if (values.Length >= 20)
                            {
                                if (long.TryParse(values[0], NumberStyles.Any, culture, out long timeUs))
                                {
                                    double[] transformMatrix = new double[16]; double[] totalForce = new double[3]; bool parseOk = true;
                                    // Parse Transform Matrix
                                    for (int i = 0; i < 16; i++) { if (!double.TryParse(values[i + 1].Trim(), NumberStyles.Any, culture, out transformMatrix[i])) { LogAndAddMessage(GH_RuntimeMessageLevel.Warning, $"Failed parse transform value #{i} ('{values[i + 1]}') on data line {lineNum}. Skipping."); parseOk = false; break; } }
                                    if (!parseOk) continue;
                                    // Parse Total Force
                                    for (int i = 0; i < 3; i++) { if (!double.TryParse(values[i + 17].Trim(), NumberStyles.Any, culture, out totalForce[i])) { LogAndAddMessage(GH_RuntimeMessageLevel.Warning, $"Failed parse force value #{i} ('{values[i + 17]}') on data line {lineNum}. Skipping."); parseOk = false; break; } }
                                    if (!parseOk) continue;
                                    // Add entry if all parsed OK
                                    logData.Add(new ReplayFrameData(timeUs, transformMatrix, totalForce));
                                }
                                else { LogAndAddMessage(GH_RuntimeMessageLevel.Warning, $"Failed parse timestamp on data line {lineNum}. Skipping."); }
                            }
                            else { LogAndAddMessage(GH_RuntimeMessageLevel.Warning, $"Incorrect data values ({values.Length}) on line {lineNum}. Expected >= 20. Skipping."); }
                        }
                    } // End While Loop
                } // End Using Reader

                // --- Post-Load Checks and Setup ---
                if (!foundDataHeader) LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "Data header line ('#Data Columns:...') not found.");
                if (!foundWorldTransformValues) LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "WorldTransform values not found/parsed.");
                if (!foundTcpOffsetValues) LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "TCPOffset values not found/parsed.");
                if (!(foundCamLocation && foundCamTarget && foundLensLength && foundCamUp)) LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "One or more Camera header values not found/parsed.");

                if (logData.Count > 0)
                {
                    logData = logData.OrderBy(e => e.TimeMicroseconds).ToList();
                    startTimeUs = logData[0].TimeMicroseconds;
                    endTimeUs = logData[logData.Count - 1].TimeMicroseconds;
                    double logDurationUs = endTimeUs - startTimeUs;
                    LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"Total log duration: {logDurationUs / 1000000.0:F3} seconds ({logData.Count} entries).");
                    return $"OK: Loaded {logData.Count} data points."; // Return success
                }
                else
                {
                    LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "No valid data points loaded.");
                    return "Warning: No data points loaded."; // Return warning
                }
            }
            catch (IOException ioEx)
            {
                ResetPlaybackState(); LogAndAddMessage(GH_RuntimeMessageLevel.Error, $"Error reading file (I/O): {ioEx.Message}");
                return $"Error loading file (I/O)."; // Return error
            }
            catch (Exception ex)
            {
                ResetPlaybackState(); LogAndAddMessage(GH_RuntimeMessageLevel.Error, $"Error loading file (General): {ex.Message}");
                return $"Error loading file (General)."; // Return error
            }
        } // --- End LoadLogFile ---


        /// <summary>
        /// Calculates Device and World planes for a specific frame index from loaded log data.
        /// </summary>
        private bool CalculatePlanesForFrame(int frameIndex, Vector3d tcpOffsetToApply, Transform worldTransformToApply, out Plane devicePlane, out Plane worldPlane)
        {
            devicePlane = Plane.Unset;
            worldPlane = Plane.Unset;

            if (logData == null || frameIndex < 0 || frameIndex >= logData.Count) { return false; }

            ReplayFrameData entry = logData[frameIndex];
            if (entry.TransformMatrix == null || entry.TransformMatrix.Length != 16) { return false; }

            try
            {
                var origin = new Point3d(-entry.TransformMatrix[12], entry.TransformMatrix[14], entry.TransformMatrix[13]);
                var xDirection = new Vector3d(-entry.TransformMatrix[0], entry.TransformMatrix[2], entry.TransformMatrix[1]);
                var yDirection = new Vector3d(-entry.TransformMatrix[4], entry.TransformMatrix[6], entry.TransformMatrix[5]);
                Plane baseDevicePlane = new Plane(origin, xDirection, yDirection);

                devicePlane = baseDevicePlane; // Start with base
                if (!tcpOffsetToApply.IsZero)
                {
                    Vector3d baseZDirection = Vector3d.CrossProduct(baseDevicePlane.XAxis, baseDevicePlane.YAxis);
                    Vector3d offsetInPlaneSpace = tcpOffsetToApply.X * baseDevicePlane.XAxis + tcpOffsetToApply.Y * baseDevicePlane.YAxis + tcpOffsetToApply.Z * baseZDirection;
                    devicePlane.Origin += offsetInPlaneSpace;
                }

                worldPlane = devicePlane; // Start with TCP-adjusted plane
                if (!worldTransformToApply.Equals(Transform.Identity))
                {
                    worldPlane.Transform(worldTransformToApply);
                }
                return true; // Success
            }
            catch (Exception ex)
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Error, $"Exception calculating planes for log index {frameIndex}: {ex.Message}");
                devicePlane = Plane.Unset; worldPlane = Plane.Unset;
                return false; // Indicate failure
            }
        }


        // --- Header Parsing Helpers (Ensure return values for bool methods) ---
        private bool ParsePoint3DHeader(string line, string expectedPrefix, ref Point3d targetPoint, ref bool foundFlag, int lineNum)
        {
            if (!line.StartsWith(expectedPrefix + ":")) return false; // Return false if prefix mismatch
            string valuePart = line.Substring(expectedPrefix.Length + 1).Trim();
            string[] parts = valuePart.Split(',');
            if (parts.Length == 3 &&
                double.TryParse(parts[0], NumberStyles.Any, CultureInfo.InvariantCulture, out double x) &&
                double.TryParse(parts[1], NumberStyles.Any, CultureInfo.InvariantCulture, out double y) &&
                double.TryParse(parts[2], NumberStyles.Any, CultureInfo.InvariantCulture, out double z))
            {
                targetPoint = new Point3d(x, y, z); foundFlag = true; return true; // Return true on success
            }
            LogAndAddMessage(GH_RuntimeMessageLevel.Warning, $"Failed to parse '{expectedPrefix}' values on line {lineNum}. Found: '{valuePart}'");
            return false; // Return false on failure
        }

        private bool ParseVector3DHeader(string line, string expectedPrefix, ref Vector3d targetVector, ref bool foundFlag, int lineNum, CultureInfo culture)
        {
            if (!line.StartsWith(expectedPrefix + ":")) return false; // Return false
            string valuePart = line.Substring(expectedPrefix.Length + 1).Trim();
            string[] parts = valuePart.Split(',');
            if (parts.Length == 3 &&
                double.TryParse(parts[0], NumberStyles.Any, culture, out double x) &&
                double.TryParse(parts[1], NumberStyles.Any, culture, out double y) &&
                double.TryParse(parts[2], NumberStyles.Any, culture, out double z))
            {
                targetVector = new Vector3d(x, y, z); foundFlag = true; return true; // Return true
            }
            LogAndAddMessage(GH_RuntimeMessageLevel.Warning, $"Failed to parse '{expectedPrefix}' values on line {lineNum}. Found: '{valuePart}'");
            return false; // Return false
        }

        private bool ParseDoubleHeader(string line, string expectedPrefix, ref double targetDouble, ref bool foundFlag, int lineNum, CultureInfo culture)
        {
            if (!line.StartsWith(expectedPrefix + ":")) return false; // Return false
            string valuePart = line.Substring(expectedPrefix.Length + 1).Trim();
            if (double.TryParse(valuePart, NumberStyles.Any, culture, out double val))
            {
                targetDouble = val; foundFlag = true; return true; // Return true
            }
            LogAndAddMessage(GH_RuntimeMessageLevel.Warning, $"Failed to parse '{expectedPrefix}' value on line {lineNum}. Found: '{valuePart}'");
            return false; // Return false
        }

        private bool ParseTransformValues(string line, ref Transform targetTransform, int lineNum, CultureInfo culture)
        {
            string[] matrixValues = line.Split(',');
            if (matrixValues.Length == 16)
            {
                double[] m = new double[16];
                for (int i = 0; i < 16; i++)
                {
                    if (!double.TryParse(matrixValues[i].Trim(), NumberStyles.Any, culture, out m[i]))
                    {
                        LogAndAddMessage(GH_RuntimeMessageLevel.Warning, $"Failed parse WorldTransform value '{matrixValues[i]}' at index {i} on line {lineNum}.");
                        return false; // Return false
                    }
                }
                targetTransform = new Transform();
                targetTransform.M00 = m[0]; targetTransform.M01 = m[1]; targetTransform.M02 = m[2]; targetTransform.M03 = m[3];
                targetTransform.M10 = m[4]; targetTransform.M11 = m[5]; targetTransform.M12 = m[6]; targetTransform.M13 = m[7];
                targetTransform.M20 = m[8]; targetTransform.M21 = m[9]; targetTransform.M22 = m[10]; targetTransform.M23 = m[11];
                targetTransform.M30 = m[12]; targetTransform.M31 = m[13]; targetTransform.M32 = m[14]; targetTransform.M33 = m[15];
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "Parsed WorldTransform values from log file.");
                return true; // Return true
            }
            LogAndAddMessage(GH_RuntimeMessageLevel.Warning, $"WorldTransform value line {lineNum} has incorrect number of values ({matrixValues.Length}). Expected 16.");
            return false; // Return false
        }

        private bool ParseVector3DValues(string line, ref Vector3d targetVector, int lineNum, CultureInfo culture)
        {
            string[] parts = line.Split(',');
            if (parts.Length == 3 &&
                double.TryParse(parts[0].Trim(), NumberStyles.Any, culture, out double x) &&
                double.TryParse(parts[1].Trim(), NumberStyles.Any, culture, out double y) &&
                double.TryParse(parts[2].Trim(), NumberStyles.Any, culture, out double z))
            {
                targetVector = new Vector3d(x, y, z);
                LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "Parsed TCPOffset values from log file.");
                return true; // Return true
            }
            LogAndAddMessage(GH_RuntimeMessageLevel.Warning, $"TCPOffset value line {lineNum} has incorrect number of values ({parts.Length}). Expected 3.");
            return false; // Return false
        }

        /// <summary>
        /// Finds the index of the log entry closest to the target time using a binary search approach.
        /// Assumes logData is sorted by TimeMicroseconds. Returns -1 if data is empty.
        /// </summary>
        private int FindClosestEntryIndex(double targetTimeUs)
        {
            if (logData == null || logData.Count == 0) return -1; // Return -1

            int low = 0;
            int high = logData.Count - 1;

            // Handle edge cases: target time before first or after last entry
            if (targetTimeUs <= logData[low].TimeMicroseconds) return low; // Return 0
            if (targetTimeUs >= logData[high].TimeMicroseconds) return high; // Return last index

            int closestIndex = low; // Initialize closest index

            while (low <= high)
            {
                int mid = low + (high - low) / 2;
                // Prevent accessing invalid index if mid becomes equal to logData.Count
                if (mid >= logData.Count)
                {
                    high = logData.Count - 1; // Adjust high bound and try again or exit loop
                    continue;
                }

                long midTime = logData[mid].TimeMicroseconds;

                // Check difference with mid and update closestIndex if mid is closer
                // Use long for difference calculation to avoid potential double precision issues with large timestamps
                if (Math.Abs(midTime - targetTimeUs) < Math.Abs(logData[closestIndex].TimeMicroseconds - targetTimeUs))
                {
                    closestIndex = mid;
                }

                if (midTime < targetTimeUs)
                {
                    low = mid + 1;
                }
                else if (midTime > targetTimeUs)
                {
                    high = mid - 1;
                }
                else
                { // Exact match found
                    return mid; // Return exact match index
                }
            }

            // After loop, check neighbors of 'closestIndex' found during search
            int bestIndex = closestIndex;
            // Use long for difference calculation
            long minDiff = Math.Abs(logData[bestIndex].TimeMicroseconds - (long)targetTimeUs);

            if (closestIndex > 0)
            { // Check index before
                long diffPrev = Math.Abs(logData[closestIndex - 1].TimeMicroseconds - (long)targetTimeUs);
                if (diffPrev < minDiff)
                {
                    minDiff = diffPrev; bestIndex = closestIndex - 1;
                }
            }
            if (closestIndex < logData.Count - 1)
            { // Check index after
                long diffNext = Math.Abs(logData[closestIndex + 1].TimeMicroseconds - (long)targetTimeUs);
                if (diffNext < minDiff)
                {
                    bestIndex = closestIndex + 1;
                }
            }

            return bestIndex; // Return the index with the absolute minimum difference
        }
        // --- End Helper Methods ---

        protected override System.Drawing.Bitmap Icon => null;
        public override Guid ComponentGuid => new Guid("3E9F0C4A-1D7A-4F8C-B2E1-8D4A1B5E9C1F");
        public override GH_Exposure Exposure => GH_Exposure.primary;

    } // End Class
} // End Namespace