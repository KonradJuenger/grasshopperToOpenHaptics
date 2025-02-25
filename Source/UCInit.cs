using Grasshopper.Kernel;
using System;
using System.Collections.Generic;
using System.IO.Ports;
namespace ghoh
{
    public class UCInit : GH_Component
    {
        public UCInit() : base(
            "UCInit",  // CHANGED: Match class name
            "initUC",
            "Initializes the microcontroller for force reading",
            "ghoh",
            "device")
        {
        }
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("Toggle", "->", "Toggle to initialize or deinitialize the microcontroller", GH_ParamAccess.item);
            pManager.AddTextParameter("Port", "P", "Serial port name (e.g., COM3)", GH_ParamAccess.item, "COM3");
            pManager.AddIntegerParameter("Baud Rate", "B", "Serial baud rate", GH_ParamAccess.item, 460800);
        }
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddBooleanParameter("Connected", "C", "True if microcontroller is connected", GH_ParamAccess.item);
            pManager.AddNumberParameter("Raw Value", "R", "Current raw ADC value", GH_ParamAccess.item);
            pManager.AddTextParameter("Status", "S", "Status message", GH_ParamAccess.item);
        }
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool toggle = false;
            string portName = "COM3";
            int baudRate = 460800;
            if (!DA.GetData(0, ref toggle)) return;
            if (!DA.GetData(1, ref portName)) return;
            if (!DA.GetData(2, ref baudRate)) return;
            Logger.Log("UCInit - SolveInstance called");  // CHANGED: Updated log message
            Logger.Log($"UCInit - Toggle value: {toggle}, Port: {portName}, Baud: {baudRate}");  // CHANGED: Updated log message
            if (!toggle)
            {
                Logger.Log("UCInit - Deinitializing microcontroller");  // CHANGED: Updated log message
                UCManager.Deinitialize();
                DA.SetData(0, false);
                DA.SetData(1, 0);
                DA.SetData(2, "Microcontroller deinitialized.");
                return;
            }
            string errorMessage;
            if (!UCManager.Initialize(portName, baudRate, out errorMessage))
            {
                DA.SetData(0, false);
                DA.SetData(1, 0);
                DA.SetData(2, errorMessage);
                Logger.Log($"UCInit - Initialization failed: {errorMessage}");  // CHANGED: Updated log message
                return;
            }
            DA.SetData(0, true);
            DA.SetData(1, UCManager.CurrentRawValue);
            DA.SetData(2, $"Connected to {portName} at {baudRate} baud");
            Logger.Log("UCInit - Microcontroller initialized successfully");  // CHANGED: Updated log message
        }
        public override void AppendAdditionalMenuItems(System.Windows.Forms.ToolStripDropDown menu)
        {
            base.AppendAdditionalMenuItems(menu);
            // Add a menu item to list available ports
            System.Windows.Forms.ToolStripMenuItem portMenuItem = new System.Windows.Forms.ToolStripMenuItem("Available Ports");
            try
            {
                string[] ports = System.IO.Ports.SerialPort.GetPortNames();
                if (ports.Length == 0)
                {
                    portMenuItem.DropDownItems.Add("No ports available");
                }
                else
                {
                    foreach (string port in ports)
                    {
                        portMenuItem.DropDownItems.Add(port);
                    }
                }
            }
            catch (Exception ex)
            {
                portMenuItem.DropDownItems.Add($"Error: {ex.Message}");
            }
            menu.Items.Add(portMenuItem);
        }
        protected override System.Drawing.Bitmap Icon => null;
        public override Guid ComponentGuid => new Guid("e4826449-a6e0-4edf-b7d2-0e001828c49d");
    }
}