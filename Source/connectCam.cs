using System;
using System.IO.Ports; // Required for SerialPort
using System.Drawing;
using System.Linq; // Used for pinCode.All(char.IsDigit)
using System.Collections.Generic; // Required for List<>

using Grasshopper.Kernel;
using GH_IO.Serialization; // Potentially needed if storing complex state

namespace Ghoh // Use the same namespace as your UCInit or choose a new one
{
    public class ConnectCam : GH_Component
    {
        // --- Member Variables ---
        // Instance variables to hold state (unlike static in Script_Instance)
        private SerialPort _serialPort = null;
        private string _previousComPort = "";
        private int _previousBaudRate = 0; // Track baud rate changes too
        private bool _previousTriggerState = false;
        private string _previousPinCode = ""; // Track previous PIN
        private bool _isConnected = false; // Track connection state explicitly

        public ConnectCam()
          : base("ConnectCam", // Name
                 "ConnectCam", // Nickname
                 "Connects to a device via Serial and sends trigger commands.", // Description
                 "Ghoh", // Category (match your other components)
                 "Device") // Subcategory (match your other components)
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            // Match inputs from the original script
            pManager.AddTextParameter("COM Port", "P", "Serial port name (e.g., COM3)", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Trigger", "T", "Trigger state (True=rec, False=stop)", GH_ParamAccess.item, false);
            pManager.AddIntegerParameter("Baud Rate", "B", "Serial baud rate", GH_ParamAccess.item, 115200);
            pManager.AddTextParameter("PIN Code", "PIN", "6-digit PIN for pairing reminder (optional)", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Connect", "C", "Connect (True) or Disconnect (False) the serial port", GH_ParamAccess.item, true);

            // Make PIN optional
            pManager[3].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            // Index 0
            pManager.AddTextParameter("Status", "S", "Current connection and operation status", GH_ParamAccess.item);
            // Index 1 - NEW OUTPUT
            pManager.AddTextParameter("Log", "L", "Runtime messages, warnings, and errors", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // List to collect messages for the Log output
            List<string> logMessages = new List<string>();
            string currentStatus = "Disconnected"; // Default status

            // --- Input Processing ---
            string comPort = null;
            bool trigger = false;
            int baudRate = 115200;
            string pinCode = null;
            bool connectPort = true; // Default to trying to connect

            // Use DA.GetData() to retrieve inputs
            if (!DA.GetData(0, ref comPort)) return;
            if (!DA.GetData(1, ref trigger)) return;
            if (!DA.GetData(2, ref baudRate)) return;
            DA.GetData(3, ref pinCode); // PIN is optional
            if (!DA.GetData(4, ref connectPort)) return;

            // Validate baud rate
            if (baudRate <= 0)
            {
                baudRate = 115200;
                string baudWarning = "Invalid baud rate provided, using default 115200.";
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, baudWarning);
                logMessages.Add($"[Warning] {baudWarning}");
            }

            // --- Disconnect Logic ---
            if (!connectPort)
            {
                if (_serialPort != null && _serialPort.IsOpen)
                {
                    string msg = $"Disconnecting port {_previousComPort} via Connect input.";
                    logMessages.Add($"[Info] {msg}");
                    try
                    {
                        _serialPort.Close();
                    }
                    catch (Exception ex)
                    {
                        string closeError = $"Error closing port {_previousComPort}: {ex.Message}";
                        AddRuntimeMessage(GH_RuntimeMessageLevel.Error, closeError);
                        logMessages.Add($"[Error] {closeError}");
                    }
                    // Clean up state after closing
                    _serialPort.Dispose(); // Dispose the object
                    _serialPort = null;
                }
                else
                {
                    logMessages.Add("[Info] Request to disconnect, but port was already closed or null.");
                }
                // Reset tracking variables when disconnected or explicitly told to disconnect
                _previousComPort = "";
                _previousBaudRate = 0;
                _previousPinCode = "";
                _isConnected = false;
                currentStatus = "Disconnected (Connect is False)";
                DA.SetData(0, currentStatus);
                DA.SetDataList(1, logMessages); // Set log output
                return; // Exit if we are explicitly disconnecting
            }

            // --- Connect Logic (Only if connectPort is true) ---

            // Validate comPort input
            if (string.IsNullOrEmpty(comPort))
            {
                if (_serialPort != null && _serialPort.IsOpen)
                {
                    string msg = $"Serial port closed due to cleared/invalid COM Port input.";
                    logMessages.Add($"[Info] {msg}");
                    CloseAndCleanupPort(logMessages); // Pass list to helper
                }
                _previousComPort = ""; // Clear tracking
                _previousBaudRate = 0;
                _isConnected = false;
                currentStatus = "Please provide a valid COM port name to connect.";
                DA.SetData(0, currentStatus);
                DA.SetDataList(1, logMessages); // Set log output
                return; // Exit early
            }

            // --- Port Management (Open/Reconnect) ---
            if (comPort != _previousComPort || baudRate != _previousBaudRate || _serialPort == null || !_serialPort.IsOpen)
            {
                if (_serialPort != null && _serialPort.IsOpen)
                {
                    string msg = $"Settings changed. Closing port {_previousComPort} before opening new connection.";
                    logMessages.Add($"[Info] {msg}");
                    CloseAndCleanupPort(logMessages); // Pass list to helper
                }

                if (_serialPort == null)
                {
                    logMessages.Add($"[Info] Attempting to open {comPort} at {baudRate} baud...");
                    try
                    {
                        _serialPort = new SerialPort(comPort, baudRate);
                        _serialPort.NewLine = "\n";
                        _serialPort.ReadTimeout = 500;
                        _serialPort.WriteTimeout = 500;
                        _serialPort.Open();
                        currentStatus = $"Connected to {comPort}";
                        logMessages.Add($"[Info] {currentStatus}");
                        _previousComPort = comPort;
                        _previousBaudRate = baudRate;
                        _previousPinCode = "";
                        _isConnected = true;
                    }
                    catch (Exception ex)
                    {
                        string openError = $"Failed to open {comPort}: {ex.Message}";
                        AddRuntimeMessage(GH_RuntimeMessageLevel.Error, openError);
                        logMessages.Add($"[Error] {openError}");
                        if (_serialPort != null) _serialPort.Dispose();
                        _serialPort = null;
                        _previousComPort = "";
                        _previousBaudRate = 0;
                        _isConnected = false;
                        currentStatus = $"Error: Could not open {comPort}";
                        DA.SetData(0, currentStatus);
                        DA.SetDataList(1, logMessages); // Set log output
                        return; // Exit if connection failed
                    }
                }
            }

            // --- Command Sending Logic (Only if port is open/connected) ---
            if (_serialPort != null && _serialPort.IsOpen)
            {
                if (!currentStatus.StartsWith("Connected") && !currentStatus.Contains("command sent")) // Avoid overwriting specific messages
                    currentStatus = $"Connected to {comPort}";

                // --- Check if PIN input changed (for reminder only) ---
                if (pinCode != null && pinCode != _previousPinCode)
                {
                    if (!string.IsNullOrEmpty(pinCode))
                    {
                        if (pinCode.Length == 6 && pinCode.All(char.IsDigit))
                        {
                            string pinMsg = $"PIN code set: {pinCode}. Enter in ESP32 Serial Monitor if prompted.";
                            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, pinMsg);
                            logMessages.Add($"[Info] {pinMsg}");
                        }
                        else
                        {
                            string pinWarn = $"Invalid PIN format '{pinCode}' in input. Must be 6 digits.";
                            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, pinWarn);
                            logMessages.Add($"[Warning] {pinWarn}");
                        }
                    }
                    else
                    {
                        logMessages.Add("[Info] PIN code input cleared.");
                    }
                    _previousPinCode = pinCode;
                }


                // --- Send Record/Stop Trigger if state changed ---
                if (trigger != _previousTriggerState)
                {
                    string commandToSend = trigger ? "rec" : "stop";
                    string logMessage = trigger ? "Trigger Activated: Sending 'rec'" : "Trigger Deactivated: Sending 'stop'";
                    string statusMessage = trigger ? $"Record START command sent to {comPort}" : $"Record STOP command sent to {comPort}";

                    logMessages.Add($"[Info] {logMessage}"); // Log intent
                    try
                    {
                        _serialPort.WriteLine(commandToSend);
                        AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, statusMessage); // Use remark for successful command
                        logMessages.Add($"[Info] Command '{commandToSend}' sent successfully.");
                        currentStatus = statusMessage;
                    }
                    catch (TimeoutException tex)
                    {
                        string timeoutError = $"Timeout writing '{commandToSend}' to {comPort}: {tex.Message}";
                        AddRuntimeMessage(GH_RuntimeMessageLevel.Error, timeoutError);
                        logMessages.Add($"[Error] {timeoutError}");
                        currentStatus = $"Timeout writing to {comPort}";
                        // Optionally close port on timeout
                        // CloseAndCleanupPort(logMessages, $"Closing port due to write timeout.");
                        // _isConnected = false;
                    }
                    catch (Exception ex)
                    {
                        string writeError = $"Error writing '{commandToSend}' to {comPort}: {ex.Message}";
                        AddRuntimeMessage(GH_RuntimeMessageLevel.Error, writeError);
                        logMessages.Add($"[Error] {writeError}");
                        currentStatus = $"Error writing to {comPort}";
                        // Optionally close port on other errors
                        // CloseAndCleanupPort(logMessages, $"Closing port due to write error.");
                        // _isConnected = false;
                    }
                    _previousTriggerState = trigger;
                }
            }
            else
            {
                if (connectPort)
                {
                    currentStatus = $"Error: Connection failed or lost to {comPort}";
                    logMessages.Add($"[Error] {currentStatus}");
                    _isConnected = false;
                    _previousComPort = "";
                    _previousBaudRate = 0;
                }
            }


            // --- Update State & Final Output ---
            _isConnected = (_serialPort != null && _serialPort.IsOpen);
            if (_isConnected && currentStatus.StartsWith("Disconnected")) // If connected but status hasn't updated, set default connected status
            {
                currentStatus = $"Connected to {comPort}";
            }

            DA.SetData(0, currentStatus);
            DA.SetDataList(1, logMessages); // Set the log output list
        }

        // Modified helper method to accept the log list
        private void CloseAndCleanupPort(List<string> logs, string reason = null)
        {
            if (_serialPort != null)
            {
                string portDesc = _previousComPort ?? "port"; // Get a name for logging
                if (_serialPort.IsOpen)
                {
                    try
                    {
                        _serialPort.Close();
                        logs?.Add($"[Info] Closed serial port {portDesc}."); // Add closing message to log
                    }
                    catch (Exception ex)
                    {
                        string closeError = $"Error during port close ({portDesc}): {ex.Message}";
                        AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, closeError);
                        logs?.Add($"[Warning] {closeError}");
                    }
                }
                _serialPort.Dispose();
                _serialPort = null;
                _isConnected = false;
                if (!string.IsNullOrEmpty(reason))
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, reason);
                    logs?.Add($"[Info] {reason}"); // Add reason to log
                }
            }
            else
            {
                logs?.Add($"[Info] CloseAndCleanupPort called, but port was already null.");
            }
            _isConnected = false;
        }

        // Overload for cases where log list isn't needed/available
        private void CloseAndCleanupPort()
        {
            CloseAndCleanupPort(null, null);
        }


        // --- Component Cleanup ---
        ~ConnectCam()
        {
            CloseAndCleanupPort(); // Basic cleanup in destructor
        }


        // --- Component Icon ---
        protected override Bitmap Icon => null; // Provide path to an embedded 24x24px Bitmap or return null

        // --- Component GUID ---
        // MAKE SURE YOU HAVE REPLACED THIS WITH YOUR OWN UNIQUE GUID
        public override Guid ComponentGuid => new Guid("A4E8F6D1-B8C9-4A2E-9D11-D4D1C5B7E9F0");
    }
}