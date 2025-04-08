using Grasshopper.Kernel;
using Rhino.Geometry;
using System;
using System.IO;
using System.Text;

namespace ghoh
{
    public class ghohForceMonitor : GH_Component
    {
        // Fields for rate limiting
        private DateTime lastUpdateTime = DateTime.MinValue;
        private Vector3d lastFilteredForceVector = Vector3d.Zero;
        private double lastFilteredAmplitude = 0;
        private Point3d lastFilteredPosition = Point3d.Origin;
        private bool hasNewFilteredData = false;

        // Fields for logging
        private bool isLogging = false;
        private StreamWriter logWriter = null;
        private StringBuilder logBuffer = new StringBuilder(4096); // 4KB buffer
        private DateTime lastFlushTime = DateTime.MinValue;
        private const int FLUSH_INTERVAL_MS = 500; // Flush to disk every 500ms

        public ghohForceMonitor() : base(
            "ghohForceMonitor",
            "ForceMonitor",
            "Outputs both real-time and rate-limited haptic force vectors and position with optional logging",
            "ghoh",
            "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enable", "E", "Enable the component", GH_ParamAccess.item, true);
            pManager.AddNumberParameter("UpdateInterval", "I", "Milliseconds between filtered updates (0 for every frame)", GH_ParamAccess.item, 100);
            pManager.AddBooleanParameter("EnableFiltered", "F", "Enable filtered output", GH_ParamAccess.item, true);
            pManager.AddTransformParameter("Transform", "X", "Optional transform matrix for world to device space", GH_ParamAccess.item);
            pManager.AddBooleanParameter("LogToFile", "L", "Enable logging to file", GH_ParamAccess.item, false);
            pManager.AddTextParameter("LogFilePath", "P", "Path to log file (leave empty for default)", GH_ParamAccess.item, "");

            pManager[3].Optional = true;  // Transform is optional
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("ForceVector", "V", "Current unfiltered force vector", GH_ParamAccess.item);
            pManager.AddNumberParameter("Amplitude", "A", "Current unfiltered force magnitude", GH_ParamAccess.item);
            pManager.AddPointParameter("Position", "P", "Current unfiltered device position", GH_ParamAccess.item);
            pManager.AddVectorParameter("FilteredForceVector", "FV", "Rate-limited force vector", GH_ParamAccess.item);
            pManager.AddNumberParameter("FilteredAmplitude", "FA", "Rate-limited force magnitude", GH_ParamAccess.item);
            pManager.AddPointParameter("FilteredPosition", "FP", "Rate-limited device position", GH_ParamAccess.item);
            pManager.AddTextParameter("Status", "S", "Component status", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Check if device is initialized
            int handle = DeviceManager.DeviceHandle;
            if (handle == HDdll.HD_INVALID_HANDLE)
            {
                DA.SetData(0, Vector3d.Zero);
                DA.SetData(1, 0.0);
                DA.SetData(2, Point3d.Origin);
                // Don't set anything for filtered outputs
                DA.SetData(6, "Device not initialized");
                return;
            }

            // Get input parameters
            bool enable = true;
            double updateIntervalMs = 100;
            bool enableFiltered = true;
            Transform additionalTransform = Transform.Identity;
            bool logToFile = false;
            string logFilePath = "";

            DA.GetData(0, ref enable);
            DA.GetData(1, ref updateIntervalMs);
            DA.GetData(2, ref enableFiltered);
            DA.GetData(3, ref additionalTransform);
            DA.GetData(4, ref logToFile);
            DA.GetData(5, ref logFilePath);

            // Handle disabling
            if (!enable)
            {
                CloseLogFile();
                DA.SetData(0, Vector3d.Zero);
                DA.SetData(1, 0.0);
                DA.SetData(2, Point3d.Origin);
                // Don't set anything for filtered outputs when disabled
                DA.SetData(6, "Disabled");
                hasNewFilteredData = false;
                return;
            }

            string status = "OK";

            // Handle file logging toggle
            if (logToFile && !isLogging)
            {
                OpenLogFile(logFilePath);
            }
            else if (!logToFile && isLogging)
            {
                CloseLogFile();
            }

            // Get current device state
            var state = DeviceManager.GetCurrentState();

            // Get position in Rhino coordinates
            var devicePosition = new Point3d(
                -state.Transform[12],
                state.Transform[14],
                state.Transform[13]
            );

            // Apply additional transform if provided
            if (!additionalTransform.Equals(Transform.Identity))
            {
                devicePosition.Transform(additionalTransform);
            }

            // Always get current unfiltered force
            double[] forceRaw = ForceManager.GetCurrentForce();
            double amplitude = Math.Sqrt(
                forceRaw[0] * forceRaw[0] +
                forceRaw[1] * forceRaw[1] +
                forceRaw[2] * forceRaw[2]
            );

            // Convert to Rhino coordinate system
            Vector3d forceVector = new Vector3d(
                -forceRaw[0],  // Negate X for Rhino space
                forceRaw[2],   // Device Y maps to Rhino Z
                forceRaw[1]    // Device Z maps to Rhino Y
            );

            // Apply transform to force vector if needed
            if (!additionalTransform.Equals(Transform.Identity))
            {
                // Create a copy of the transform for vector transformation
                // We only want the rotation/scaling part, not the translation
                Transform vectorTransform = additionalTransform;
                vectorTransform.M03 = 0; // Zero out translation components
                vectorTransform.M13 = 0;
                vectorTransform.M23 = 0;

                forceVector.Transform(vectorTransform);
            }

            // Set unfiltered outputs
            DA.SetData(0, forceVector);
            DA.SetData(1, amplitude);
            DA.SetData(2, devicePosition);

            // Check if we need to update filtered output based on time interval
            DateTime now = DateTime.Now;
            bool shouldUpdateFiltered = (now - lastUpdateTime).TotalMilliseconds >= updateIntervalMs;

            // Update filtered outputs if needed
            if (shouldUpdateFiltered && enableFiltered)
            {
                lastFilteredForceVector = forceVector;
                lastFilteredAmplitude = amplitude;
                lastFilteredPosition = devicePosition;
                lastUpdateTime = now;
                hasNewFilteredData = true;
            }

            // Only set filtered outputs if enabled and we have new data
            if (enableFiltered && hasNewFilteredData)
            {
                DA.SetData(3, lastFilteredForceVector);
                DA.SetData(4, lastFilteredAmplitude);
                DA.SetData(5, lastFilteredPosition);
            }
            else
            {
                // Don't set filtered outputs - they'll appear as null in Grasshopper
                // This is important for creating discontinuous trails
            }

            // Log if enabled (always log unfiltered data for accuracy)
            if (isLogging)
            {
                LogForceData(now, forceVector, amplitude, devicePosition);

                // Flush log buffer if needed
                if ((now - lastFlushTime).TotalMilliseconds >= FLUSH_INTERVAL_MS)
                {
                    FlushLogBuffer();
                    lastFlushTime = now;
                }
            }

            // Set status output
            DA.SetData(6, status);

            // Return arrays to pool
            state.ReturnArrays();
        }

        private void OpenLogFile(string filePath)
        {
            try
            {
                // Create default path if none provided
                if (string.IsNullOrEmpty(filePath))
                {
                    string docPath = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
                    string fileName = $"ghoh_force_log_{DateTime.Now:yyyy-MM-dd_HH-mm-ss}.csv";
                    filePath = Path.Combine(docPath, fileName);
                }

                // Create the file and writer
                logWriter = new StreamWriter(filePath, false, Encoding.UTF8, 65536); // 64KB buffer
                logWriter.WriteLine("Timestamp,ForceX,ForceY,ForceZ,Magnitude,PositionX,PositionY,PositionZ");
                isLogging = true;
                logBuffer.Clear();
                lastFlushTime = DateTime.Now;
            }
            catch (Exception ex)
            {
                isLogging = false;
                if (logWriter != null)
                {
                    logWriter.Dispose();
                    logWriter = null;
                }
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $"Failed to open log file: {ex.Message}");
            }
        }

        private void LogForceData(DateTime timestamp, Vector3d force, double magnitude, Point3d position)
        {
            if (!isLogging || logWriter == null) return;

            try
            {
                // Format: Timestamp,ForceX,ForceY,ForceZ,Magnitude,PositionX,PositionY,PositionZ
                logBuffer.AppendLine(
                    $"{timestamp:yyyy-MM-dd HH:mm:ss.fff}," +
                    $"{force.X:F6},{force.Y:F6},{force.Z:F6},{magnitude:F6}," +
                    $"{position.X:F6},{position.Y:F6},{position.Z:F6}"
                );
            }
            catch (Exception ex)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, $"Error logging data: {ex.Message}");
            }
        }

        private void FlushLogBuffer()
        {
            if (!isLogging || logWriter == null || logBuffer.Length == 0) return;

            try
            {
                logWriter.Write(logBuffer.ToString());
                logWriter.Flush();
                logBuffer.Clear();
            }
            catch (Exception ex)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $"Failed to write to log file: {ex.Message}");
                CloseLogFile();
            }
        }

        private void CloseLogFile()
        {
            if (!isLogging) return;

            try
            {
                if (logBuffer.Length > 0 && logWriter != null)
                {
                    logWriter.Write(logBuffer.ToString());
                }

                if (logWriter != null)
                {
                    logWriter.Flush();
                    logWriter.Dispose();
                    logWriter = null;
                }
            }
            catch
            {
                // Suppress errors during shutdown
            }
            finally
            {
                isLogging = false;
                logBuffer.Clear();
            }
        }

        protected override void AfterSolveInstance()
        {
            // Ensure log is flushed after each solution
            if (isLogging && logBuffer.Length > 0)
            {
                FlushLogBuffer();
            }
        }

        public override void RemovedFromDocument(GH_Document document)
        {
            CloseLogFile();
            base.RemovedFromDocument(document);
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("e4826449-a6e0-3edf-b7d2-0e001322c10a");
    }
}