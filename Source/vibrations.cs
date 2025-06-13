using System;

namespace ghoh
{
    public static class Vibration
    {
        public enum VibrationMode { Sine, Square, Pulse }

        private static DateTime lastVibrationTime = DateTime.Now;
        private static double phase = 0.0; // For sine/square wave continuity

        // State for Pulse mode
        private static bool isPulsing = true;
        private static DateTime pulseStateChangeTime = DateTime.Now;

        /// <summary>
        /// Gets a vibration multiplier (-1 to 1) based on the current time and selected mode.
        /// </summary>
        /// <param name="mode">The type of vibration to generate.</param>
        /// <param name="frequency">Frequency in Hz for the underlying wave.</param>
        /// <param name="pulseDuration">The 'on' duration for Pulse mode (in seconds).</param>
        /// <param name="currentPauseDuration">The 'off' duration for Pulse mode (in seconds).</param>
        /// <returns>Oscillation multiplier, typically between -1 and 1.</returns>
        public static double GetVibrationMultiplier(VibrationMode mode, double frequency, double pulseDuration, double currentPauseDuration)
        {
            DateTime currentTime = DateTime.Now;
            double timeDelta = Math.Min(0.1, (currentTime - lastVibrationTime).TotalSeconds);
            lastVibrationTime = currentTime;

            // Always update phase for smooth transitions if switching back to Sine/Square
            phase += 2.0 * Math.PI * frequency * timeDelta;
            phase = phase % (2.0 * Math.PI);

            switch (mode)
            {
                case VibrationMode.Sine:
                    return Math.Sin(phase);

                case VibrationMode.Square:
                    return (Math.Sin(phase) >= 0) ? 1.0 : -1.0;

                case VibrationMode.Pulse:
                    double timeInCurrentState = (currentTime - pulseStateChangeTime).TotalSeconds;

                    if (isPulsing)
                    {
                        if (timeInCurrentState > pulseDuration)
                        {
                            isPulsing = false;
                            pulseStateChangeTime = currentTime;
                            return 0.0; // Start of pause
                        }
                        // While pulsing, use a sine wave for the vibration itself
                        return Math.Sin(phase);
                    }
                    else // Is Pausing
                    {
                        if (timeInCurrentState > currentPauseDuration)
                        {
                            isPulsing = true;
                            pulseStateChangeTime = currentTime;
                            // When switching back, immediately start the sine wave
                            return Math.Sin(phase);
                        }
                        return 0.0; // Still pausing
                    }

                default:
                    return 0.0;
            }
        }

        /// <summary>
        /// Resets the persistent state for the vibration.
        /// </summary>
        public static void Reset()
        {
            lastVibrationTime = DateTime.Now;
            phase = 0.0;
            // Reset pulse state as well
            isPulsing = true;
            pulseStateChangeTime = DateTime.Now;
        }
    }
}
