using Grasshopper.Kernel;
using System;
using System.IO.Ports;

namespace ghoh
{
    public class UCReadForce : GH_Component
    {
        private bool lastEnabled = false;
        private double lastScale = 1.0;

        public UCReadForce() : base(
            "UCReadForce",  // CHANGED: Match class name
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
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("RawValue", "R", "Current raw ADC value from microcontroller", GH_ParamAccess.item);
            pManager.AddNumberParameter("MappedForce", "F", "Mapped force value (N)", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool enable = false;
            double scaleFactor = 1.0;
            double inputMin = 0.0;
            double inputMax = 4095.0;
            double outputMin = 0.0;
            double outputMax = 10.0;

            if (!DA.GetData(0, ref enable)) return;
            if (!DA.GetData(1, ref scaleFactor)) return;
            if (!DA.GetData(2, ref inputMin)) return;
            if (!DA.GetData(3, ref inputMax)) return;
            if (!DA.GetData(4, ref outputMin)) return;
            if (!DA.GetData(5, ref outputMax)) return;

            // Validate and clamp parameters
            scaleFactor = Math.Max(0, Math.Min(10, scaleFactor));
            outputMax = Math.Max(0, Math.Min(10, outputMax));

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
                DA.SetData(1, 0.0);
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Microcontroller not connected");
                return;
            }

            // Set force parameters in UCManager
            UCManager.SetForceParameters(
                enable,
                scaleFactor,
                inputMin,
                inputMax,
                outputMin,
                outputMax
            );

            // Get the current raw value and mapped force
            int rawValue = UCManager.CurrentRawValue;
            double mappedForce = UCManager.GetMappedForceValue();

            // Only log changes to reduce console spam
            bool significantChange = enable != lastEnabled || Math.Abs(scaleFactor - lastScale) > 0.05;
            if (significantChange)
            {
                if (enable)
                {
                    Logger.Log($"UCReadForce - Enabled with scale: {scaleFactor:F2}, " +  // CHANGED: Updated log message
                               $"Input: {inputMin}-{inputMax}, Output: {outputMin}-{outputMax}");
                }
                else if (lastEnabled)
                {
                    Logger.Log("UCReadForce - Force application disabled");  // CHANGED: Updated log message
                }

                lastEnabled = enable;
                lastScale = scaleFactor;
            }

            // Output the current values
            DA.SetData(0, rawValue);
            DA.SetData(1, mappedForce);
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("99f37d10-f396-11ef-ac77-0800200c9a66");
    }
}