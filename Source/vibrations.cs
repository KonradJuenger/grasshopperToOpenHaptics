using System;
using System.Diagnostics;

namespace ghoh
{
    public static class Vibration
    {
        // High precision timer for oscillation
        private static readonly Stopwatch timer = new Stopwatch();

        static Vibration()
        {
            // Start the timer immediately
            timer.Start();
        }

        /// <summary>
        /// Calculates a vibration multiplier (-1 to 1) based on current time
        /// </summary>
        /// <param name="frequency">Frequency in Hz</param>
        /// <param name="useSquareWave">True for square wave, false for sine wave</param>
        /// <returns>Vibration multiplier between -1 and 1</returns>
        public static double GetVibrationMultiplier(double frequency, bool useSquareWave)
        {
            // Calculate phase based on time and frequency
            double elapsedSeconds = timer.Elapsed.TotalSeconds;
            double phase = 2 * Math.PI * frequency * elapsedSeconds;

            // Calculate oscillation factor (-1 to 1)
            if (useSquareWave)
            {
                // Square wave (either -1 or 1)
                return (Math.Sin(phase) >= 0) ? 1.0 : -1.0;
            }
            else
            {
                // Sine wave
                return Math.Sin(phase);
            }
        }

        /// <summary>
        /// Resets the vibration timer
        /// </summary>
        public static void Reset()
        {
            timer.Restart();
        }
    }
}