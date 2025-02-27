using Grasshopper.Kernel;
using System;
using System.IO.Ports;

namespace ghoh
{
    public class UCReadForce : GH_Component
    {
        private bool lastEnabled = false;
        private double lastScale = 1.0;
        private bool lastFilterEnabled = true;
        private int lastWindowSize = 10;

        public UCReadForce() : base(
            "UCReadForce",
            "UCForce",
            "Configures force reading from the microcontroller and applies it to the haptic device",
            "ghoh",
            "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enable", "E", "Enable or disable force reading and application", GH_ParamAccess.item, false);
            pManager.AddNumberParameter("Scale", "S", "Scale factor for the force", GH_ParamAccess.item, 1.0);
            pManager.AddNumberParameter("InputMin", "IMin", "Minimum input ADC value", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("InputMax", "IMax", "Maximum input ADC value", GH_ParamAccess.item, 4095.0);
            pManager.AddNumberParameter("OutputMin", "OMin", "Minimum output force value (N)", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("OutputMax", "OMax", "Maximum output force value (N)", GH_ParamAccess.item, 10.0);
            pManager.AddBooleanParameter("FilterEnable", "FE", "Enable or disable the moving average filter", GH_ParamAccess.item, true);
            pManager.AddIntegerParameter("WindowSize", "WS", "Number of samples to average (higher = smoother but more lag)", GH_ParamAccess.item, 10);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("RawValue", "R", "Current raw ADC value from microcontroller", GH_ParamAccess.item);
            pManager.AddNumberParameter("FilteredValue", "F", "Filtered ADC value (if filter enabled)", GH_ParamAccess.item);
            pManager.AddNumberParameter("MappedForce", "M", "Mapped force value (N)", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool enable = false;
            double scaleFactor = 1.0;
            double inputMin = 0.0;
            double inputMax = 4095.0;
            double outputMin = 0.0;
            double outputMax = 10.0;
            bool filterEnable = true;
            int windowSize = 10;

            if (!DA.GetData(0, ref enable)) return;
            if (!DA.GetData(1, ref scaleFactor)) return;
            if (!DA.GetData(2, ref inputMin)) return;
            if (!DA.GetData(3, ref inputMax)) return;
            if (!DA.GetData(4, ref outputMin)) return;
            if (!DA.GetData(5, ref outputMax)) return;
            if (!DA.GetData(6, ref filterEnable)) return;
            if (!DA.GetData(7, ref windowSize)) return;

            // Validate and clamp parameters
            scaleFactor = Math.Max(0, Math.Min(10, scaleFactor));
            outputMax = Math.Max(0, Math.Min(10, outputMax));

            // Validate window size
            if (windowSize < 1)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Window size must be at least 1. Using 1 instead.");
                windowSize = 1;
            }
            else if (windowSize > 100)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Large window sizes may cause significant lag in response.");
            }

            // Ensure input range is valid
            if (inputMax <= inputMin)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "InputMax must be greater than InputMin");
                inputMax = inputMin + 1;
            }

            // Ensure output range is valid
            if (outputMax <= outputMin)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "OutputMax must be greater than OutputMin");
                outputMax = outputMin + 1;
            }

            if (!UCManager.IsConnected)
            {
                DA.SetData(0, 0);
                DA.SetData(1, 0);
                DA.SetData(2, 0.0);
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Microcontroller not connected");
                return;
            }

            // Set filter parameters
            UCManager.SetFilterParameters(filterEnable, windowSize);

            // Set force parameters in UCManager
            UCManager.SetForceParameters(
                enable,
                scaleFactor,
                inputMin,
                inputMax,
                outputMin,
                outputMax
            );

            // Get the current raw value, filtered value, and mapped force
            int rawValue = UCManager.CurrentRawValue;
            int filteredValue = UCManager.FilteredValue;
            double mappedForce = UCManager.GetMappedForceValue();

            // Log significant changes
            bool forceParamsChanged = enable != lastEnabled || Math.Abs(scaleFactor - lastScale) > 0.05;
            bool filterParamsChanged = filterEnable != lastFilterEnabled || windowSize != lastWindowSize;

            if (forceParamsChanged)
            {
                if (enable)
                {
                    Logger.Log($"UCReadForce - Force enabled with scale: {scaleFactor:F2}, " +
                               $"Input: {inputMin}-{inputMax}, Output: {outputMin}-{outputMax}");
                }
                else if (lastEnabled)
                {
                    Logger.Log("UCReadForce - Force application disabled");
                }

                lastEnabled = enable;
                lastScale = scaleFactor;
            }

            if (filterParamsChanged)
            {
                if (filterEnable)
                {
                    Logger.Log($"UCReadForce - Filter enabled with window size: {windowSize}");
                }
                else if (lastFilterEnabled)
                {
                    Logger.Log("UCReadForce - Filter disabled, using raw values");
                }

                lastFilterEnabled = filterEnable;
                lastWindowSize = windowSize;
            }

            // Output the current values
            DA.SetData(0, rawValue);
            DA.SetData(1, filteredValue);
            DA.SetData(2, mappedForce);
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("99f37d10-f396-11ef-ac77-0800200c9a66");
    }
}