using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Diagnostics;
using System.Globalization;

namespace ghoh
{
    // Simple struct to hold parsed log data (Internal to prevent potential conflicts if reused elsewhere)
    internal struct ReplayEntry_ghohReplay // Renamed struct slightly to avoid potential name clashes if defined elsewhere
    {
        public long TimeMicroseconds;
        public Point3d DevicePosition;
        public Vector3d DeviceForce;
    }

    public class ghohReplay : GH_Component
    {
        // --- Fields ---
        private List<ReplayEntry_ghohReplay> logData = new List<ReplayEntry_ghohReplay>();
        private string currentFilePath = "";
        private bool isRunning = false;
        private bool previousRunState = false;
        private Transform recordedWorldTransform = Transform.Identity; // Store transform parsed from log
        private long startTimeUs = 0;
        private long endTimeUs = 0;
        private int currentFrameNumber = -1; // Use -1 to indicate not running/before start
        private double frameDurationUs = 0; // Calculated from FPS input

        // *** New Field for collecting runtime messages ***
        private List<string> runtimeMessages = new List<string>();
        // --- End Fields ---

        public ghohReplay() : base(
            "ghohReplay",
            "ReplayServoLog",
            "Replays device data from log. Calculates world pos using recorded or input transform. Outputs frame number and runtime log.", // Updated desc
            "ghoh",
            "Utility")
        {
        }

        // --- New Helper Method for Logging ---
        /// <summary>
        /// Adds message to component runtime messages (balloon) and internal list for output parameter.
        /// </summary>
        private void LogAndAddMessage(GH_RuntimeMessageLevel level, string message)
        {
            string prefix = level switch
            {
                GH_RuntimeMessageLevel.Error => "[ERROR] ",
                GH_RuntimeMessageLevel.Warning => "[WARN] ",
                GH_RuntimeMessageLevel.Remark => "[INFO] ", // Using INFO for Remark for clarity
                _ => ""
            };
            runtimeMessages.Add(prefix + message); // Add to list for output
            AddRuntimeMessage(level, message); // Add for standard GH pop-up balloon
        }
        // --- End Helper Method ---


        // Clear runtime messages before each solution calculation
        protected override void BeforeSolveInstance()
        {
            runtimeMessages.Clear();
            base.BeforeSolveInstance();
        }


        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("LogFolderPath", "Path", "Base directory containing the numbered run subfolders", GH_ParamAccess.item); // Index 0
            pManager.AddIntegerParameter("RunNumber", "Num", "The specific run number folder to replay (e.g., 1, 2, 112)", GH_ParamAccess.item, 1); // Index 1
            pManager.AddBooleanParameter("Run", "R", "Set to True to start/continue playback, False to stop/reset", GH_ParamAccess.item, false); // Index 2
            pManager.AddNumberParameter("FPS", "FPS", "Target resampling Frames Per Second (e.g., 29.97, >0 required)", GH_ParamAccess.item, 30.0); // Index 3
            pManager.AddIntegerParameter("SubsampleFactor", "Sub", "Average force over N data points around target frame (1 = no subsampling)", GH_ParamAccess.item, 1); // Index 4
            pManager.AddTransformParameter("WorldTransform", "X", "[Optional] If provided, overrides the transform read from the log file for WorldPosition output.", GH_ParamAccess.item); // Index 5
            pManager[5].Optional = true; // Make transform optional
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("TimeMicroseconds", "Tus", "Timestamp of the chosen frame's data point from log file (microseconds)", GH_ParamAccess.item); // Index 0
            pManager.AddPointParameter("DevicePosition", "DevPos", "Raw device position from log file for the chosen frame", GH_ParamAccess.item); // Index 1
            pManager.AddPointParameter("WorldPosition", "WldPos", "Calculated world position (using recorded or input transform) for the chosen frame", GH_ParamAccess.item); // Index 2
            pManager.AddVectorParameter("DeviceForce", "DevFrc", "Raw (potentially subsampled) device force from log file for the chosen frame", GH_ParamAccess.item); // Index 3
            pManager.AddTextParameter("Status", "S", "Playback status", GH_ParamAccess.item); // Index 4
            // *** New Output Parameters ***
            pManager.AddIntegerParameter("FrameNumber", "FN", "Current playback frame number (0-based). -1 if stopped/idle.", GH_ParamAccess.item); // Index 5
            pManager.AddTextParameter("RuntimeLog", "Log", "Log of runtime messages (errors, warnings, info)", GH_ParamAccess.list); // Index 6
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string folderPath = ""; int runNumber = 1; bool run = false;
            double fps = 30.0; int subsample = 1;
            Transform inputTransform = Transform.Identity; bool transformInputProvided = false; // Flag to track input connection

            CultureInfo culture = CultureInfo.InvariantCulture; // Use invariant culture for parsing

            // --- Input Acquisition ---
            // Use TryGetData for potentially better error handling if inputs are unexpectedly disconnected
            if (!DA.GetData(0, ref folderPath)) { LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "LogFolderPath input missing."); /* Default handled */ }
            if (!DA.GetData(1, ref runNumber)) { LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "RunNumber input missing."); /* Default handled */ }
            if (!DA.GetData(2, ref run)) { LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "Run input missing."); /* Default handled */ }
            if (!DA.GetData(3, ref fps)) { LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "FPS input missing."); /* Default handled */ }
            if (!DA.GetData(4, ref subsample)) { LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "SubsampleFactor input missing."); /* Default handled */ }
            // Check optional transform input (Index 5)
            transformInputProvided = DA.GetData(5, ref inputTransform); // Optional, uses Identity if not provided or fails
            // --- End Input Acquisition ---

            string status = "Idle";
            int outputFrameNumber = -1; // Default output frame number when not running
            subsample = Math.Max(1, subsample); // Ensure >= 1
            if (fps <= 0)
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "FPS must be greater than 0. Using 30.0 as fallback.");
                fps = 30.0;
            }
            frameDurationUs = 1000000.0 / fps; // Calculate frame duration in microseconds

            string runNumberPadded = runNumber.ToString("D5");
            string targetFilePath = Path.Combine(folderPath ?? "", runNumberPadded, "servo_log.csv"); // Handle potentially null folderPath gracefully

            // --- State Management ---
            if (run && !previousRunState) // Triggered start/resume
            {
                // Load file if it's different or data is empty
                bool needsLoad = (targetFilePath != currentFilePath || logData.Count == 0);
                if (!File.Exists(targetFilePath) && needsLoad)
                {
                    status = $"Error: Log file not found: {targetFilePath}";
                    LogAndAddMessage(GH_RuntimeMessageLevel.Error, status);
                    isRunning = false; // Ensure not running
                }
                else if (needsLoad)
                {
                    status = LoadLogFile(targetFilePath, culture); // Load data and parse transform
                    if (!status.StartsWith("OK"))
                    {
                        // Loading failed, LoadLogFile already added message
                        isRunning = false; // Ensure not running
                    }
                    else
                    {
                        currentFilePath = targetFilePath; // Update path only on successful load
                        LogAndAddMessage(GH_RuntimeMessageLevel.Remark, $"Loaded log file: {targetFilePath}");
                    }
                }
                else if (!needsLoad && logData.Count == 0)
                {
                    // File path matches, but logData is empty (maybe previous load failed but didn't reset path?)
                    status = "Error: Log data is empty despite matching file path.";
                    LogAndAddMessage(GH_RuntimeMessageLevel.Error, status);
                    isRunning = false;
                }

                // Only proceed if data was successfully loaded (or already loaded) and is present
                if (logData.Count > 0)
                {
                    if (!isRunning && !status.StartsWith("Error")) // Only set running if no error occurred during load
                    {
                        isRunning = true;
                        currentFrameNumber = 0; // Reset frame number to start from beginning
                        startTimeUs = logData[0].TimeMicroseconds;
                        endTimeUs = logData[logData.Count - 1].TimeMicroseconds;
                        status = $"Running Frame {currentFrameNumber}";
                        LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "Playback starting.");
                        ExpireSolution(true); // Immediately trigger first frame calculation
                    }
                }
                else if (!status.StartsWith("Error")) // If no file/load error, but still no data
                {
                    status = "No data loaded or file empty.";
                    LogAndAddMessage(GH_RuntimeMessageLevel.Warning, status);
                    isRunning = false; // Can't run without data
                }
            }
            else if (!run && previousRunState) // Triggered stop
            {
                if (isRunning) LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "Playback stopped by user.");
                isRunning = false;
                currentFrameNumber = -1; // Reset frame number to stopped state
                status = "Stopped";
            }
            previousRunState = run; // Update state for next iteration
            // --- End State Management ---


            // --- Playback Logic ---
            if (isRunning && logData.Count > 0)
            {
                // Calculate the target timestamp for the current frame number
                // Ensure currentFrameNumber is valid before using
                if (currentFrameNumber < 0) currentFrameNumber = 0; // Should be 0 if isRunning is true, but safety check
                outputFrameNumber = currentFrameNumber; // Set output frame number for this iteration

                double targetTimeUs = startTimeUs + currentFrameNumber * frameDurationUs;

                // Check if target time exceeds the duration of the log (+ one frame allowance)
                // Use endTimeUs directly as the last valid timestamp
                if (targetTimeUs > endTimeUs + frameDurationUs) // Allow calculation slightly past the end
                {
                    status = "Finished";
                    LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "Playback finished.");
                    outputFrameNumber = -1; // Indicate finished state for frame number output
                    // Set outputs to null/default to indicate finish? Or keep last frame? Let's clear.
                    DA.SetData(0, null); DA.SetData(1, null); DA.SetData(2, null); DA.SetData(3, null);
                    // Optionally stop isRunning here, or let user toggle Run input
                    // isRunning = false; // Uncomment to auto-stop after finishing one cycle
                }
                else
                {
                    // Find the log entry closest to the target time
                    int closestIndex = FindClosestEntryIndex(targetTimeUs);

                    if (closestIndex < 0)
                    {
                        status = "Error finding frame data."; // Should only happen if logData is empty, checked above
                        LogAndAddMessage(GH_RuntimeMessageLevel.Error, status);
                        outputFrameNumber = -1; // Indicate error state
                                                // Clear outputs
                        DA.SetData(0, null); DA.SetData(1, null); DA.SetData(2, null); DA.SetData(3, null);
                    }
                    else
                    {
                        // Get the representative entry for this frame
                        ReplayEntry_ghohReplay entry = logData[closestIndex];

                        // Calculate potentially subsampled force centered around this entry
                        Vector3d outputForce = CalculateAverageForce(closestIndex, subsample);

                        // --- Calculate World Position ---
                        Point3d devicePos = entry.DevicePosition;
                        // 1. Convert raw device position to default Rhino space
                        Point3d worldPos = new Point3d(-devicePos.X, devicePos.Z, devicePos.Y); // Note coordinate mapping

                        // 2. Determine which transform to use: Input overrides Recorded
                        Transform transformToUse = transformInputProvided ? inputTransform : recordedWorldTransform;

                        // 3. Apply final transform
                        worldPos.Transform(transformToUse);
                        // --- End World Position ---

                        // --- Set Outputs ---
                        DA.SetData(0, (double)entry.TimeMicroseconds); // Timestamp of closest frame (Index 0) - Cast to double maybe safer? Check param type. NumberParameter -> double
                        DA.SetData(1, devicePos);               // Raw Device Position (Index 1)
                        DA.SetData(2, worldPos);                // Calculated World Position (Index 2)
                        DA.SetData(3, outputForce);             // (Potentially) averaged Device Force (Index 3)

                        status = $"Running Frame {currentFrameNumber} (LogT: {entry.TimeMicroseconds / 1000.0:F0}ms)";

                        // Advance to the next target frame number
                        currentFrameNumber++;

                        // Trigger the next solution calculation if still potentially running
                        // Check if the *next* frame's target time might still be valid or close
                        double nextTargetTimeUs = startTimeUs + currentFrameNumber * frameDurationUs;
                        if (nextTargetTimeUs <= endTimeUs + frameDurationUs * 1.5) // Allow slightly more leeway
                        {
                            ExpireSolution(true);
                        }
                        else
                        {
                            // If the next frame is definitely past the end, update status immediately
                            status = "Finished";
                            LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "Playback finished (end of data reached).");
                            outputFrameNumber = -1; // Indicate finished state
                                                    // Clear outputs to signify end
                            DA.SetData(0, null); DA.SetData(1, null); DA.SetData(2, null); DA.SetData(3, null);
                            // isRunning = false; // Uncomment to auto-stop
                        }
                    }
                }
            }
            else if (isRunning && logData.Count == 0)
            {
                // This state should ideally be caught during the start trigger
                status = "Running, but no log data loaded.";
                LogAndAddMessage(GH_RuntimeMessageLevel.Warning, status);
                outputFrameNumber = -1; // Indicate invalid state
            }
            else if (!isRunning)
            {
                // Update status message when stopped/idle
                outputFrameNumber = -1; // Ensure frame number is -1 when stopped
                if (File.Exists(targetFilePath) && status == "Idle")
                {
                    status = "Ready"; // Indicate ready if stopped and file exists
                }
                else if (!File.Exists(targetFilePath) && status == "Idle")
                {
                    status = $"Log file not found: {runNumberPadded}";
                    // Don't log error here again, was logged on load attempt
                }
                // Otherwise keep status as "Idle" or "Stopped" or error message from loading
                DA.SetData(0, null); DA.SetData(1, null); DA.SetData(2, null); DA.SetData(3, null); // Clear data outputs when stopped
            }
            // --- End Playback Logic ---

            // --- Set Final Outputs ---
            DA.SetData(4, status);                 // Set Status output (Index 4)
            DA.SetData(5, outputFrameNumber);      // Set FrameNumber output (Index 5)
            DA.SetDataList(6, runtimeMessages);    // Set RuntimeLog output (Index 6)
            // --- End Set Final Outputs ---
        }


        /// <summary>
        /// Loads the specified log file, parses headers (including WorldTransform), and data.
        /// Uses LogAndAddMessage for warnings/errors.
        /// </summary>
        private string LoadLogFile(string filePath, CultureInfo culture)
        {
            logData.Clear(); startTimeUs = 0; endTimeUs = 0; currentFilePath = ""; // Reset state related to file
            recordedWorldTransform = Transform.Identity; // Reset transform for each load attempt
            bool transformValuesFound = false; // Track if transform was successfully parsed

            if (!File.Exists(filePath))
            {
                LogAndAddMessage(GH_RuntimeMessageLevel.Error, $"File not found: {filePath}");
                return $"Error: File not found."; // Return error status
            }

            try
            {
                using (StreamReader reader = new StreamReader(filePath))
                {
                    string line; int lineNum = 0; bool transformHeaderFound = false;
                    bool foundDataHeader = false; // Track if the data header row is found
                    while ((line = reader.ReadLine()) != null)
                    {
                        lineNum++; line = line.Trim();
                        if (string.IsNullOrEmpty(line)) continue; // Skip empty lines

                        if (line.StartsWith("#"))
                        {
                            // Process Header Lines
                            if (line.StartsWith("#WorldTransform:"))
                            {
                                transformHeaderFound = true; // Mark that we found the transform definition line
                            }
                            // Check for specific data header to ensure format compatibility
                            else if (line.StartsWith("#time_us,device_pos_x"))
                            {
                                foundDataHeader = true;
                            }
                            // Skip other comment lines (like recording start time)
                            continue; // Move to next line
                        }
                        else if (transformHeaderFound) // Check if this non-comment line is the transform data
                        {
                            transformHeaderFound = false; // Consume this flag, only expect data on the line immediately after header
                            // This line *should* be the transform values
                            string[] matrixValues = line.Split(',');
                            if (matrixValues.Length == 16)
                            {
                                double[] m = new double[16]; bool parseOk = true;
                                for (int i = 0; i < 16; i++)
                                {
                                    // Try parsing each matrix element
                                    if (!double.TryParse(matrixValues[i], NumberStyles.Float | NumberStyles.AllowExponent, culture, out m[i])) // Allow scientific notation
                                    {
                                        parseOk = false;
                                        LogAndAddMessage(GH_RuntimeMessageLevel.Warning, $"Failed to parse transform value '{matrixValues[i]}' at index {i} on line {lineNum}.");
                                        break; // Stop parsing this line
                                    }
                                }
                                if (parseOk)
                                {
                                    // Construct Transform object correctly
                                    recordedWorldTransform = new Transform(); // Start fresh
                                    recordedWorldTransform.M00 = m[0]; recordedWorldTransform.M01 = m[1]; recordedWorldTransform.M02 = m[2]; recordedWorldTransform.M03 = m[3];
                                    recordedWorldTransform.M10 = m[4]; recordedWorldTransform.M11 = m[5]; recordedWorldTransform.M12 = m[6]; recordedWorldTransform.M13 = m[7];
                                    recordedWorldTransform.M20 = m[8]; recordedWorldTransform.M21 = m[9]; recordedWorldTransform.M22 = m[10]; recordedWorldTransform.M23 = m[11];
                                    recordedWorldTransform.M30 = m[12]; recordedWorldTransform.M31 = m[13]; recordedWorldTransform.M32 = m[14]; recordedWorldTransform.M33 = m[15];
                                    transformValuesFound = true; // Mark success
                                    LogAndAddMessage(GH_RuntimeMessageLevel.Remark, "Parsed WorldTransform from log file.");
                                }
                                else { LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "Failed to parse all WorldTransform values from log."); }
                            }
                            else { LogAndAddMessage(GH_RuntimeMessageLevel.Warning, $"WorldTransform value line {lineNum} has incorrect number of values ({matrixValues.Length}). Expected 16."); }
                            // Continue to next line after processing potential transform data
                            continue;
                        }

                        // --- Parse Data Line ---
                        // Only parse if it doesn't start with # and isn't the transform line we just handled
                        string[] values = line.Split(',');
                        if (values.Length >= 7)
                        {
                            if (long.TryParse(values[0], out long timeUs) &&
                                double.TryParse(values[1], NumberStyles.Float | NumberStyles.AllowExponent, culture, out double posX) &&
                                double.TryParse(values[2], NumberStyles.Float | NumberStyles.AllowExponent, culture, out double posY) &&
                                double.TryParse(values[3], NumberStyles.Float | NumberStyles.AllowExponent, culture, out double posZ) &&
                                double.TryParse(values[4], NumberStyles.Float | NumberStyles.AllowExponent, culture, out double forceX) &&
                                double.TryParse(values[5], NumberStyles.Float | NumberStyles.AllowExponent, culture, out double forceY) &&
                                double.TryParse(values[6], NumberStyles.Float | NumberStyles.AllowExponent, culture, out double forceZ))
                            {
                                logData.Add(new ReplayEntry_ghohReplay
                                {
                                    TimeMicroseconds = timeUs,
                                    DevicePosition = new Point3d(posX, posY, posZ),
                                    DeviceForce = new Vector3d(forceX, forceY, forceZ)
                                });
                            }
                            else { LogAndAddMessage(GH_RuntimeMessageLevel.Warning, $"Failed parse numeric data line {lineNum}. Skipping."); }
                        }
                        else { LogAndAddMessage(GH_RuntimeMessageLevel.Warning, $"Incorrect data values ({values.Length}) on line {lineNum}. Expected >= 7. Skipping."); }
                        // --- End Parse Data Line ---
                    } // End While Loop
                    if (!foundDataHeader && logData.Count > 0)
                    {
                        LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "Data header line ('#time_us,device_pos_x...') not found. File format might be incompatible.");
                    }
                    else if (!foundDataHeader && logData.Count == 0)
                    {
                        LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "Data header line not found and no data parsed.");
                    }
                } // End Using Reader

                // Set start/end times after loading all data
                if (logData.Count > 0)
                {
                    // Ensure data is sorted by timestamp (it should be, but safety check)
                    logData = logData.OrderBy(e => e.TimeMicroseconds).ToList();
                    startTimeUs = logData[0].TimeMicroseconds;
                    endTimeUs = logData[logData.Count - 1].TimeMicroseconds;
                    // Adjust start time to be relative to zero? No, keep original timestamps.
                    // The calculation `targetTimeUs = startTimeUs + ...` handles the offset.
                }
                else
                {
                    LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "No valid data points loaded from file.");
                }
                // Report if transform wasn't found/parsed
                if (!transformValuesFound) { LogAndAddMessage(GH_RuntimeMessageLevel.Warning, "WorldTransform not found or parsed from log file header. Using Identity transform."); }

                // Store path only if loading finished without critical file access error
                // currentFilePath = filePath; // Set in caller only on success
                return $"OK: Loaded {logData.Count} data points.";
            }
            catch (IOException ioEx)
            {
                logData.Clear(); // Ensure data is cleared on error
                LogAndAddMessage(GH_RuntimeMessageLevel.Error, $"Error reading file (I/O): {ioEx.Message}");
                return $"Error loading file (I/O): {ioEx.Message}";
            }
            catch (Exception ex)
            {
                logData.Clear(); // Ensure data is cleared on error
                LogAndAddMessage(GH_RuntimeMessageLevel.Error, $"Error loading file (General): {ex.Message}");
                return $"Error loading file (General): {ex.Message}";
            }
        }
        // --- End LoadLogFile ---


        /// <summary>
        /// Finds the index of the log entry closest to the target time using a linear scan.
        /// Assumes logData is sorted by TimeMicroseconds.
        /// Returns -1 if data is empty.
        /// </summary>
        private int FindClosestEntryIndex(double targetTimeUs)
        {
            if (logData == null || logData.Count == 0) return -1;
            if (logData.Count == 1) return 0; // Only one entry

            // Optimization: If target time is before the first entry or after the last
            if (targetTimeUs <= logData[0].TimeMicroseconds) return 0;
            if (targetTimeUs >= logData[logData.Count - 1].TimeMicroseconds) return logData.Count - 1;

            // Binary search would be faster for large logs, but linear scan is simpler
            // and likely sufficient unless logs are extremely long. Let's keep linear for now.
            int bestIndex = 0;
            double minDiff = double.MaxValue;

            // Simple linear scan
            for (int i = 0; i < logData.Count; i++)
            {
                double diff = Math.Abs(logData[i].TimeMicroseconds - targetTimeUs);
                if (diff < minDiff)
                {
                    minDiff = diff;
                    bestIndex = i;
                }
                // Optimization: Since data is sorted, if we pass the target time
                // and the difference starts increasing, we've found the minimum.
                // However, need to handle edge cases carefully. Let's stick to full scan for robustness.
                // Alternative: Find the first entry *greater than* targetTimeUs,
                // then compare its difference and the previous entry's difference.

                // Optimization based on sorted data:
                if (logData[i].TimeMicroseconds > targetTimeUs)
                {
                    // We just passed the target time. Compare current index i and previous index i-1
                    if (i > 0)
                    {
                        double diffPrev = Math.Abs(logData[i - 1].TimeMicroseconds - targetTimeUs);
                        if (diffPrev < diff)
                        {
                            return i - 1; // Previous index was closer
                        }
                    }
                    return i; // Current index is closer or it's the first element
                }
            }
            // Should theoretically be covered by the end-check or the loop optimization,
            // but return the last index if loop completes somehow (e.g., targetTimeUs matches last entry exactly)
            return logData.Count - 1;
        }


        /// <summary>
        /// Calculates the average force vector over a window centered around closestIndex.
        /// </summary>
        private Vector3d CalculateAverageForce(int closestIndex, int subsampleFactor)
        {
            // If no subsampling or invalid index/data, return the force at the closest index
            if (subsampleFactor <= 1 || logData == null || logData.Count == 0 || closestIndex < 0 || closestIndex >= logData.Count)
            {
                return (logData != null && closestIndex >= 0 && closestIndex < logData.Count)
                       ? logData[closestIndex].DeviceForce : Vector3d.Zero;
            }

            // Determine window boundaries, centered around closestIndex
            int halfWindow = (subsampleFactor - 1) / 2;
            int startIndex = Math.Max(0, closestIndex - halfWindow);
            // Calculate end index based on start + factor, capped by data count
            // End index is exclusive in loops, so aim for startIndex + subsampleFactor
            int endIndexExclusive = Math.Min(logData.Count, startIndex + subsampleFactor);
            // Recalculate start index to ensure window size near the end, if end got capped
            // Ensure we don't go below 0 if the window size is large
            startIndex = Math.Max(0, endIndexExclusive - subsampleFactor);

            double sumX = 0, sumY = 0, sumZ = 0;
            int count = 0;

            for (int i = startIndex; i < endIndexExclusive; i++) // Use < endIndexExclusive
            {
                sumX += logData[i].DeviceForce.X;
                sumY += logData[i].DeviceForce.Y;
                sumZ += logData[i].DeviceForce.Z;
                count++;
            }

            // Return average or Zero if count is somehow zero (shouldn't happen if logic is correct)
            return (count > 0) ? new Vector3d(sumX / count, sumY / count, sumZ / count) : Vector3d.Zero;
        }


        protected override System.Drawing.Bitmap Icon => null; // Provide an icon if desired
        public override Guid ComponentGuid => new Guid("3E9F0C4A-1D7A-4F8C-B2E1-8D4A1B5E9C1F"); // Keep existing GUID
    }
}