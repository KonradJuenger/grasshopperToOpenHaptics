using Grasshopper.Kernel;
using System;
using System.Collections.Generic;

namespace ghoh
{
    public class DampingControl : GH_Component
    {
        private bool lastEnabled = false;
        private double lastSmoothingCoef = 0.5;
        private double lastDerivativeCoef = 0.0;
        private ForceManager.DampingMethod lastMethod = ForceManager.DampingMethod.ExponentialSmoothing;

        public DampingControl() : base(
            "DampingControl",
            "Damping",
            "Controls global force damping to reduce oscillations",
            "ghoh",
            "device")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Enable", "E", "Enable or disable force damping", GH_ParamAccess.item, false);
            pManager.AddIntegerParameter("Method", "M", "Damping method: 0=Smoothing, 1=Derivative, 2=Both", GH_ParamAccess.item, 0);
            pManager.AddNumberParameter("SmoothingCoef", "S", "Smoothing coefficient (0.0-0.99, higher = more damping)", GH_ParamAccess.item, 0.5);
            pManager.AddNumberParameter("DerivativeCoef", "D", "Derivative damping coefficient (0.0-1.0)", GH_ParamAccess.item, 0.0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Status", "S", "Damping status information", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool enable = false;
            int methodIndex = 0;
            double smoothingCoef = 0.5;
            double derivativeCoef = 0.0;

            if (!DA.GetData(0, ref enable)) return;
            if (!DA.GetData(1, ref methodIndex)) return;
            if (!DA.GetData(2, ref smoothingCoef)) return;
            if (!DA.GetData(3, ref derivativeCoef)) return;

            // Validate and constrain parameters
            smoothingCoef = Math.Max(0, Math.Min(0.99, smoothingCoef));
            derivativeCoef = Math.Max(0, Math.Min(1.0, derivativeCoef));

            // Convert method index to enum
            ForceManager.DampingMethod method = ForceManager.DampingMethod.ExponentialSmoothing;
            switch (methodIndex)
            {
                case 0:
                    method = ForceManager.DampingMethod.ExponentialSmoothing;
                    break;
                case 1:
                    method = ForceManager.DampingMethod.ForceDerivative;
                    break;
                case 2:
                    method = ForceManager.DampingMethod.Both;
                    break;
                default:
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
                        "Invalid method index. Using Exponential Smoothing.");
                    break;
            }

            // Check if parameters have changed
            bool paramsChanged = enable != lastEnabled ||
                                Math.Abs(smoothingCoef - lastSmoothingCoef) > 0.001 ||
                                Math.Abs(derivativeCoef - lastDerivativeCoef) > 0.001 ||
                                method != lastMethod;

            // Update damping parameters if they've changed
            if (paramsChanged)
            {
                ForceManager.SetDampingParameters(enable, smoothingCoef, derivativeCoef, method);

                // Store the current values for next comparison
                lastEnabled = enable;
                lastSmoothingCoef = smoothingCoef;
                lastDerivativeCoef = derivativeCoef;
                lastMethod = method;
            }

            // Generate status message
            string methodName = method.ToString();
            string status = enable
                ? $"Damping enabled: {methodName}, Smoothing={smoothingCoef:F2}, Derivative={derivativeCoef:F2}"
                : "Damping disabled";

            DA.SetData(0, status);
        }

        public override GH_Exposure Exposure => GH_Exposure.primary;

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e003862c69d");
    }
}