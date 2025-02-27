using System;
using System.IO.Ports;
using System.Threading;
using System.Threading.Tasks;

namespace ghoh
{
    public static class UCManager
    {
        private static SerialPort serialPort;
        private static readonly object portLock = new object();
        private static bool isConnected = false;
        private static Task readTask;
        private static CancellationTokenSource cancellationTokenSource;

        // Latest raw ADC value read from the microcontroller
        private static int currentRawValue = 0;
        private static readonly object valueLock = new object();

        // Moving average filter
        private static MovingAverageFilter filter;
        private static bool filterEnabled = true;
        private static int filterWindowSize = 10; // Default window size
        private static readonly object filterLock = new object();

        // Force parameters
        private static bool forceEnabled = false;
        private static double forceScale = 1.0;
        private static double inputMin = 0;
        private static double inputMax = 4095;
        private static double outputMin = 0;
        private static double outputMax = 10.0;
        private static readonly object paramLock = new object();

        public static bool IsConnected
        {
            get
            {
                lock (portLock)
                {
                    return isConnected && serialPort != null && serialPort.IsOpen;
                }
            }
        }

        public static int CurrentRawValue
        {
            get
            {
                lock (valueLock)
                {
                    return currentRawValue;
                }
            }
            private set
            {
                lock (valueLock)
                {
                    currentRawValue = value;
                }
            }
        }

        public static int FilteredValue
        {
            get
            {
                lock (filterLock)
                {
                    if (filter == null || !filterEnabled)
                        return CurrentRawValue;

                    return filter.GetCurrentValue();
                }
            }
        }

        public static bool FilterEnabled
        {
            get
            {
                lock (filterLock)
                {
                    return filterEnabled;
                }
            }
        }

        public static bool ForceEnabled
        {
            get
            {
                lock (paramLock)
                {
                    return forceEnabled;
                }
            }
        }

        public static double GetMappedForceValue()
        {
            double scale, inMin, inMax, outMin, outMax;
            int valueToUse;

            lock (paramLock)
            {
                scale = forceScale;
                inMin = inputMin;
                inMax = inputMax;
                outMin = outputMin;
                outMax = outputMax;
            }

            // Use filtered or raw value based on filter setting
            lock (filterLock)
            {
                if (filterEnabled && filter != null)
                {
                    valueToUse = filter.GetCurrentValue();
                }
                else
                {
                    lock (valueLock)
                    {
                        valueToUse = currentRawValue;
                    }
                }
            }

            // Map the value to the force range and apply scaling
            double normalizedValue = (valueToUse - inMin) / (inMax - inMin);
            double mappedValue = outMin + normalizedValue * (outMax - outMin);

            // Apply scaling and ensure it's within bounds
            double scaledForce = mappedValue * scale;
            return Math.Max(0, Math.Min(scaledForce, 10.0)); // Clamp between 0-10N
        }

        public static bool Initialize(string portName, int baudRate, out string errorMessage)
        {
            lock (portLock)
            {
                if (IsConnected)
                {
                    errorMessage = null;
                    return true;
                }

                try
                {
                    Logger.Log($"MicrocontrollerManager - Initializing on port {portName} at {baudRate} baud");

                    // Create and configure the serial port
                    serialPort = new SerialPort(portName, baudRate)
                    {
                        ReadTimeout = 1000,
                        WriteTimeout = 1000,
                        DtrEnable = true, // Data Terminal Ready signal
                        RtsEnable = true  // Request To Send signal
                    };

                    // Initialize the moving average filter
                    lock (filterLock)
                    {
                        filter = new MovingAverageFilter(filterWindowSize);
                    }

                    // Open the port
                    serialPort.Open();
                    isConnected = true;

                    // Start the reading task
                    cancellationTokenSource = new CancellationTokenSource();
                    readTask = Task.Run(() => ReadSerialData(cancellationTokenSource.Token));

                    errorMessage = null;
                    Logger.Log("MicrocontrollerManager - Initialization successful");
                    return true;
                }
                catch (Exception ex)
                {
                    errorMessage = $"Failed to initialize microcontroller: {ex.Message}";
                    Logger.Log(errorMessage);

                    // Clean up resources
                    if (serialPort != null && serialPort.IsOpen)
                    {
                        serialPort.Close();
                        serialPort.Dispose();
                        serialPort = null;
                    }

                    isConnected = false;
                    return false;
                }
            }
        }

        public static void Deinitialize()
        {
            lock (portLock)
            {
                if (!isConnected)
                    return;

                Logger.Log("MicrocontrollerManager - Deinitializing");

                // Cancel the reading task
                cancellationTokenSource?.Cancel();

                try
                {
                    readTask?.Wait(1000);
                }
                catch (Exception ex)
                {
                    Logger.Log($"Error waiting for read task to complete: {ex.Message}");
                }

                // Close and dispose the serial port
                if (serialPort != null && serialPort.IsOpen)
                {
                    try
                    {
                        serialPort.Close();
                        serialPort.Dispose();
                    }
                    catch (Exception ex)
                    {
                        Logger.Log($"Error closing serial port: {ex.Message}");
                    }
                    serialPort = null;
                }

                isConnected = false;
                CurrentRawValue = 0;

                // Reset the filter
                lock (filterLock)
                {
                    if (filter != null)
                    {
                        filter.Reset();
                    }
                }

                // Disable force application
                lock (paramLock)
                {
                    forceEnabled = false;
                }

                Logger.Log("MicrocontrollerManager - Deinitialized");
            }
        }

        private static void ReadSerialData(CancellationToken cancellationToken)
        {
            Logger.Log("MicrocontrollerManager - Serial read task started");

            while (!cancellationToken.IsCancellationRequested && IsConnected)
            {
                try
                {
                    string line = serialPort.ReadLine().Trim();

                    // Parse the raw ADC value
                    if (int.TryParse(line, out int adcValue))
                    {
                        // Update both raw and filtered value
                        CurrentRawValue = adcValue;

                        // Apply the moving average filter if enabled
                        lock (filterLock)
                        {
                            if (filterEnabled && filter != null)
                            {
                                int filteredValue = filter.AddSample(adcValue);
                                //Logger.Log($"MicrocontrollerManager - Raw: {adcValue}, Filtered: {filteredValue}");
                            }
                            else
                            {
                                //Logger.Log("MicrocontrollerManager - Raw value: " + adcValue);
                            }
                        }
                    }
                }
                catch (TimeoutException)
                {
                    // Timeout is normal, just continue
                }
                catch (Exception ex)
                {
                    // Only log significant errors to avoid spamming the log
                    if (!(ex is OperationCanceledException))
                    {
                        Logger.Log($"Error reading serial data: {ex.Message}");
                        Thread.Sleep(100); // Avoid tight loop in case of error
                    }
                }
            }

            Logger.Log("MicrocontrollerManager - Serial read task ended");
        }

        public static void SetForceParameters(bool enable, double scale, double inMin, double inMax, double outMin, double outMax)
        {
            lock (paramLock)
            {
                forceEnabled = enable;
                forceScale = scale;
                inputMin = inMin;
                inputMax = inMax;
                outputMin = outMin;
                outputMax = outMax;
            }
        }

        public static void SetFilterParameters(bool enable, int windowSize)
        {
            lock (filterLock)
            {
                filterEnabled = enable;

                if (windowSize < 1)
                    windowSize = 1;

                filterWindowSize = windowSize;

                if (filter != null)
                {
                    filter.SetWindowSize(windowSize);
                }
                else
                {
                    filter = new MovingAverageFilter(windowSize);
                }
            }
        }
    }
}