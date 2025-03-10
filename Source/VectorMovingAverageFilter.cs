using System;
using System.Collections.Generic;

namespace ghoh
{
    /// <summary>
    /// Implements a moving average filter for 3D vector values
    /// </summary>
    public class VectorMovingAverageFilter
    {
        private Queue<double[]> samples;
        private int maxSamples;
        private double[] sum;
        private readonly object lockObject = new object();

        /// <summary>
        /// Initialize a new moving average filter for 3D vectors
        /// </summary>
        /// <param name="windowSize">The number of samples to average (window size)</param>
        public VectorMovingAverageFilter(int windowSize)
        {
            if (windowSize < 1)
                throw new ArgumentException("Window size must be at least 1", nameof(windowSize));

            maxSamples = windowSize;
            samples = new Queue<double[]>(windowSize);
            sum = new double[3] { 0, 0, 0 };
        }
        public int GetWindowSize()
        {
            lock (lockObject)
            {
                return maxSamples;
            }
        }
        /// <summary>
        /// Add a new vector sample to the filter and get the filtered value
        /// </summary>
        /// <param name="newSample">The new vector to add</param>
        /// <returns>The filtered (averaged) vector</returns>
        public double[] AddSample(double[] newSample)
        {
            if (newSample.Length != 3)
                throw new ArgumentException("Vector must have 3 components", nameof(newSample));

            lock (lockObject)
            {
                // Add the new sample to the sum
                for (int i = 0; i < 3; i++)
                {
                    sum[i] += newSample[i];
                }

                // Clone the array to avoid external modifications affecting our queue
                samples.Enqueue((double[])newSample.Clone());

                // If we've exceeded our window size, remove the oldest sample
                if (samples.Count > maxSamples)
                {
                    var oldestSample = samples.Dequeue();
                    for (int i = 0; i < 3; i++)
                    {
                        sum[i] -= oldestSample[i];
                    }
                }

                // Return the average
                double[] result = new double[3];
                for (int i = 0; i < 3; i++)
                {
                    result[i] = sum[i] / samples.Count;
                }
                return result;
            }
        }

        /// <summary>
        /// Get the current filtered value without adding a new sample
        /// </summary>
        /// <returns>The current filtered vector or [0,0,0] if no samples available</returns>
        public double[] GetCurrentValue()
        {
            lock (lockObject)
            {
                if (samples.Count == 0)
                    return new double[3] { 0, 0, 0 };

                double[] result = new double[3];
                for (int i = 0; i < 3; i++)
                {
                    result[i] = sum[i] / samples.Count;
                }
                return result;
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
                    var oldestSample = samples.Dequeue();
                    for (int i = 0; i < 3; i++)
                    {
                        sum[i] -= oldestSample[i];
                    }
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
                for (int i = 0; i < 3; i++)
                {
                    sum[i] = 0;
                }
            }
        }
    }
}