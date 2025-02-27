using System;
using System.Collections.Generic;

namespace ghoh
{
    /// <summary>
    /// Implements an adjustable moving average filter for ADC values
    /// </summary>
    public class MovingAverageFilter
    {
        private Queue<int> samples;
        private int maxSamples;
        private int sum;
        private readonly object lockObject = new object();

        /// <summary>
        /// Initialize a new moving average filter
        /// </summary>
        /// <param name="windowSize">The number of samples to average (window size)</param>
        public MovingAverageFilter(int windowSize)
        {
            if (windowSize < 1)
                throw new ArgumentException("Window size must be at least 1", nameof(windowSize));

            maxSamples = windowSize;
            samples = new Queue<int>(windowSize);
            sum = 0;
        }

        /// <summary>
        /// Add a new sample to the filter and get the filtered value
        /// </summary>
        /// <param name="newSample">The new ADC value to add</param>
        /// <returns>The filtered (averaged) value</returns>
        public int AddSample(int newSample)
        {
            lock (lockObject)
            {
                // Add the new sample to the sum
                sum += newSample;
                samples.Enqueue(newSample);

                // If we've exceeded our window size, remove the oldest sample
                if (samples.Count > maxSamples)
                {
                    sum -= samples.Dequeue();
                }

                // Return the average
                return sum / samples.Count;
            }
        }

        /// <summary>
        /// Get the current filtered value without adding a new sample
        /// </summary>
        /// <returns>The current filtered value or 0 if no samples available</returns>
        public int GetCurrentValue()
        {
            lock (lockObject)
            {
                if (samples.Count == 0)
                    return 0;

                return sum / samples.Count;
            }
        }

        /// <summary>
        /// Change the window size of the filter
        /// </summary>
        /// <param name="newSize">The new window size (must be at least 1)</param>
        public void SetWindowSize(int newSize)
        {
            if (newSize < 1)
                throw new ArgumentException("Window size must be at least 1", nameof(newSize));

            lock (lockObject)
            {
                maxSamples = newSize;

                // Trim the queue if the new size is smaller than current count
                while (samples.Count > maxSamples)
                {
                    sum -= samples.Dequeue();
                }
            }
        }

        /// <summary>
        /// Reset the filter, clearing all samples
        /// </summary>
        public void Reset()
        {
            lock (lockObject)
            {
                samples.Clear();
                sum = 0;
            }
        }
    }
}