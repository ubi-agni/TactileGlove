// *** Graphical user interface for tactile dataglove ***
//
// Modified:
// 2013-04-30 Risto Kõiva, Added data dump
// 2012-11-13 Risto Kõiva, Changed GUI layout to resize the contents, added left/right glove choice
// 2012-11-12 Risto Kõiva, Changed from List<> to Queue<>, added comments
// 2012-11-09 Risto Kõiva, Initial version

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;
using System.IO;
using System.IO.Ports;

namespace TactileDataglove
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        // Constant declarations
        private const string sCONNECT = "_Connect"; // Button content for unconnected state
        private const string sDISCONNECT = "_Disconnect"; // Button content for connected state
        private const int CQUEUESIZE = 1024 * 1024; // Receive buffer size (1 MByte)
        private const int MAXCHANNELS = 64;

        private TextWriter tw;
        private string[] saNameMapping = new string[64];

        // Variable declarations
        private SerialPort spUSB = new SerialPort(); // Communication over (virtual) serial port
        private Queue<byte> qbReceiveQueue = new Queue<byte>(CQUEUESIZE); // Receive queue (FIFO)
        static private readonly object locker = new object(); // Queue locker object to regulate access from multiple threads
        private uint uiLastRemaining; // Saves the count of remaning bytes from last packet parser run (value range is 0 to 4)
        private byte[] baLastRemaining = new byte[4]; // Remaining byte from last packet parser run

        private bool[] baInitialTaxelValueAvailable = new bool[MAXCHANNELS];
        private int[] iaInitialTaxelValues = new int[MAXCHANNELS];

        private int[] iaTaxelValues = new int[MAXCHANNELS]; // Array holding the parsed taxel values (value range is 0 to 4095)
        private int[] iaOldTaxelValues = new int[MAXCHANNELS]; // Array for comparison if update is required, holding the last run taxel values
        private DispatcherTimer dtGUIUpdateTimer = new DispatcherTimer(); // Paces the GUI update framerate

        // MainWindow constructor gets called when the window is created - at the start of the program
        public MainWindow()
        {
            InitializeComponent();

            btConnectDisconnect.Content = sCONNECT; // Initialize the button content
            spUSB.DataReceived += new SerialDataReceivedEventHandler(spUSB_DataReceived); // Register new event for receiving serial data

            dtGUIUpdateTimer.Tick += new EventHandler(dtGUIUpdateTimer_Tick); // Register new event for timer to fire
            dtGUIUpdateTimer.Interval = new TimeSpan(0, 0, 0, 0, 20); // Set timer interval to 20ms evaluating to 50 Hz frame update rate

            // Populate the combobox with available serial ports of the current system
            string[] saAvailableSerialPorts = SerialPort.GetPortNames();
            foreach (string sAvailableSerialPort in saAvailableSerialPorts)
                cbSerialPort.Items.Add(sAvailableSerialPort);
            if (cbSerialPort.Items.Count > 0)
            {
                cbSerialPort.Text = cbSerialPort.Items[0].ToString();
                for (int i = 0; i < cbSerialPort.Items.Count; i++)
                    if (cbSerialPort.Items[i].ToString() == "COM11") // If available, default to COM11 initially
                        cbSerialPort.SelectedIndex = i;
            }

            saNameMapping[5] = "RightLFDPLF";
            saNameMapping[6] = "RightLFDPTIP";
            saNameMapping[7] = "RightLFDPMID";
            saNameMapping[8] = "RightLFMPRF";
            saNameMapping[9] = "RightLFDPRF";
            saNameMapping[10] = "RightLFMPLF";
            saNameMapping[11] = "RightLFMPMID";
            saNameMapping[12] = "RightLFPPRF";
            saNameMapping[13] = "RightLFPPLF";
            saNameMapping[16] = "RightFFDPMF";
            saNameMapping[17] = "RightFFDPTIP";
            saNameMapping[18] = "RightFFDPTH";
            saNameMapping[19] = "RightFFMPTH";
            saNameMapping[20] = "RightFFMPMF";
            saNameMapping[21] = "RightTHDPMID";
            saNameMapping[22] = "RightTHDPTIP";
            saNameMapping[23] = "RightTHDPFF";
            saNameMapping[24] = "RightFFPPMF";
            saNameMapping[25] = "RightTHDPTH";
            saNameMapping[26] = "RightTHMPTH";
            saNameMapping[27] = "RightTHMPFF";
            saNameMapping[28] = "RightFFMPMID";
            saNameMapping[29] = "RightFFDPMID";
            saNameMapping[30] = "RightFFPPTH";
            saNameMapping[31] = "RightRFMPMF";
            saNameMapping[32] = "RightRFDPMID";
            saNameMapping[33] = "RightRFDPLF";
            saNameMapping[34] = "RightRFDPTIP";
            saNameMapping[35] = "RightRFMPMID";
            saNameMapping[36] = "RightRFMPLF";
            saNameMapping[37] = "RightMFMPFF";
            saNameMapping[38] = "RightMFMPRF";
            saNameMapping[39] = "RightMFDPTIP";
            saNameMapping[40] = "RightMFDPRF";
            saNameMapping[41] = "RightMFPP";
            saNameMapping[42] = "RightMFDPFF";
            saNameMapping[43] = "RightMFMPMID";
            saNameMapping[44] = "RightMFDPMID";
            saNameMapping[46] = "RightRFDPMF";
            saNameMapping[47] = "RightRFPP";
            saNameMapping[48] = "RightPalmMIDL";
            saNameMapping[49] = "RightPalmMIDR";
            saNameMapping[50] = "RightPalmMIDBR";
            saNameMapping[51] = "RightPalmLF";
            saNameMapping[53] = "RightPalmUpFF";
            saNameMapping[54] = "RightPalmUpLF";
            saNameMapping[55] = "RightPalmUpMF";
            saNameMapping[56] = "RightPalmTHL";
            saNameMapping[57] = "RightPalmTHD";
            saNameMapping[58] = "RightPalmTHU";
            saNameMapping[59] = "RightPalmUpRF";
            saNameMapping[60] = "RightPalmTHR";
            saNameMapping[61] = "RightPalmMIDU";
            saNameMapping[62] = "RightPalmMIDBL";
        }

        // btConnectDisconnect_Click gets calles each time a button on MainWindow is pressed
        private void btConnectDisconnect_Click(object sender, RoutedEventArgs e)
        {
            // Check according to button content, if the "Connect" or "Disconnect" was pressed
            if ((string)btConnectDisconnect.Content == sCONNECT)
            {
                // Connect was pressed

                tw = new StreamWriter(String.Format("{0:yyyy.MM.dd_HH.mm.ss}.txt", DateTime.Now));
                tw.WriteLine("New Log from: " + DateTime.Now);

                // Initialize variables
                qbReceiveQueue.Clear(); // Clear the receive queue
                uiLastRemaining = 0; // Set the last packet parses "pointer" to 0
                for (int i = 0; i < MAXCHANNELS; i++) // Initialize taxel arrays
                {
                    iaTaxelValues[i] = 0;
                    iaOldTaxelValues[i] = 4095; // This forces an GUI update on initial round, as the value differs from iaTaxelValue
                    baInitialTaxelValueAvailable[i] = false;
                    iaInitialTaxelValues[i] = 0;
                }

                // Set (virtual) serial port parameters
                spUSB.BaudRate = 115200; // 115.2 kbaud/s
                spUSB.DataBits = 8;
                spUSB.DiscardNull = false;
                spUSB.DtrEnable = false;
                spUSB.Handshake = Handshake.None;
                spUSB.Parity = Parity.None;
                spUSB.ParityReplace = 63;
                spUSB.PortName = cbSerialPort.SelectedItem.ToString();
                spUSB.ReadBufferSize = 4096;
                spUSB.ReadTimeout = -1;
                spUSB.ReceivedBytesThreshold = 5 * 64; // Fire receive event when a complete data is available = Packet size * Taxel count
                spUSB.RtsEnable = true;
                spUSB.StopBits = StopBits.One;
                spUSB.WriteBufferSize = 2048;
                spUSB.WriteTimeout = -1;

                try
                {
                    if (!spUSB.IsOpen)
                        spUSB.Open(); // Try to connect to the selected serial port

                    // If we got here, then the port was successfully opened, continue
                    btConnectDisconnect.Content = sDISCONNECT; // Change button content to disconnect string
                    cbSerialPort.IsEnabled = false; // Disable port selection combobox

                    dtGUIUpdateTimer.Start(); // Start the GUI updating timer
                }
                catch (Exception ex)
                {
                    // In case of error, show message to user
                    MessageBox.Show("Could not open the communication port!" + "\n" + "Error: " + ex.Message);
                }
            }
            else
            {
                // Disconnect was pressed

                dtGUIUpdateTimer.Stop(); // Stop the GUI updating timer

                if (tw != null)
                {
                    tw.Close();
                    tw = null;

                }

                try
                {
                    if (spUSB.IsOpen)
                        spUSB.Close(); // Try to stop serial port communications

                    cbSerialPort.IsEnabled = true; // Enable the port selection combobox
                    btConnectDisconnect.Content = sCONNECT; // Change button content to connect string
                }
                catch (Exception ex)
                {
                    // In case of error, show message to user
                    MessageBox.Show("Could not close the communication port!" + "\n" + "Error: " + ex.Message);
                }

                // Reset taxels on GUI to idle state
                for (int i = 0; i < MAXCHANNELS; i++)
                {
                    iaTaxelValues[i] = 0;
                    baInitialTaxelValueAvailable[i] = false;
                }

                // Redraw the GUI manually to show this
                Paint_Taxels();
            }
        }

        // spUSB_DataReceived event gets called when data from (virtual) serial port is received.
        // Note! This function runs in a different thread as the MainWindow elements, thus care (for example
        // locking) needs to be taken in interacting with MainWindow elements.
        // It collects the data from serial port and sends them to a shared queue (FIFO).
        private void spUSB_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            int iDataLength = spUSB.BytesToRead; // The amount of bytes, serial port has received
            byte[] baSerialPortDataIn = new byte[iDataLength]; // Create an internal array to store the data
            if (iDataLength > 0) // Makes sense to continue only, if there really is new data available
            {
                spUSB.Read(baSerialPortDataIn, 0, iDataLength); // Copy the data from serial port to internal array

                lock (locker) // As the next command acts with a variable accessed from multiple threads, lock the access first
                    baSerialPortDataIn.ToList().ForEach(b => qbReceiveQueue.Enqueue(b)); // Put the bytes into thread-arching queue
            }
        }

        // dtGUIUpdateTimer_Tick event gets called when the GUI update timer fires.
        // It collects the received information from the thread-arching queue, processes the data
        // and finished by calling the GUI redraw function.
        private void dtGUIUpdateTimer_Tick(object sender, EventArgs e)
        {
            byte[] baDataIn; // Local array to hold the received data
            lock (locker) // As the commands in curly brackets read or write variable that it accessed also on other threads, a lock is requested
            {
                baDataIn = qbReceiveQueue.ToArray(); // Copy the whole queue to local array
                qbReceiveQueue.Clear(); // Clear the queue
            }

            const int iBYTESINPACKET = 5; // A single TactileDataglove packet uses 5 bytes

            // Data can be processed only if the whole packet is available. Thus there might
            // have been rest data from previous run that need to be preceded.
            // Create new byte array including the previous rest
            byte[] baWorking = new byte[uiLastRemaining + baDataIn.Length];

            if (uiLastRemaining > 0)
                baLastRemaining.CopyTo(baWorking, 0);
            if (baDataIn.Length > 0)
                baDataIn.CopyTo(baWorking, uiLastRemaining);

            int iStartPointer = 0;

            // Continue only if queue has at least the size of a full frame (5 bytes)
            while ((baWorking.Length - iStartPointer) >= iBYTESINPACKET)
            {
                // Plausibility check
                // First byte must be between 0x3C and 0x7B
                // Second byte must be 0x01
                // Fifth byte must be 0x00
                if (((baWorking[iStartPointer + 0] >= 0x3C) && (baWorking[iStartPointer + 0] <= 0x7B)) &&
                    (baWorking[iStartPointer + 1] == 0x01) &&
                    (baWorking[iStartPointer + 4] == 0x00))
                {
                    // Valid packet, save the data
                    int iChannel = baWorking[iStartPointer + 0] - 0x3C;
                    if ((iChannel < 0) && (iChannel >= 64))
                        throw new System.ArgumentException("Unvalid Taxel number received");

                    int iReceivedValue = Math.Max(0, 4095 - (256 * (0x0F & baWorking[iStartPointer + 2]) + baWorking[iStartPointer + 3]));

                    if (!baInitialTaxelValueAvailable[iChannel])
                    {
                        baInitialTaxelValueAvailable[iChannel] = true;
                        iaInitialTaxelValues[iChannel] = iReceivedValue;
                    }

                    iaTaxelValues[iChannel] = Math.Max(0, iReceivedValue - iaInitialTaxelValues[iChannel]);
                    tw.WriteLine(saNameMapping[iChannel] + ";" + iaTaxelValues[iChannel]);

                    // Move the pointer further
                    iStartPointer += 5;
                }
                else
                {
                    // Failed plausibility check. Remove first byte from queue.
                    iStartPointer++;
                }
            }

            // Save the remaining 0 to 4 bytes for next run
            if (baWorking.Length - iStartPointer > 0)
            {
                uiLastRemaining = 0;
                for (int i = 0; i < baWorking.Length - iStartPointer; i++)
                {
                    baLastRemaining[i] = baWorking[iStartPointer + i];
                    uiLastRemaining++;
                }
            }
            else
                uiLastRemaining = 0;

            Paint_Taxels();
        }

        // Applies the correct taxel states to GUI
        private void Paint_Taxels()
        {
            // ID-Patch Mapping
            if (cbLeftOrRight.SelectedIndex == 0)
            {
                // Right
                Paint_Patch(5, RightLFDPLF);
                Paint_Patch(6, RightLFDPTIP);
                Paint_Patch(7, RightLFDPMID);
                Paint_Patch(8, RightLFMPRF);
                Paint_Patch(9, RightLFDPRF);
                Paint_Patch(10, RightLFMPLF);
                Paint_Patch(11, RightLFMPMID);
                Paint_Patch(12, RightLFPPRF);
                Paint_Patch(13, RightLFPPLF);
                Paint_Patch(16, RightFFDPMF);
                Paint_Patch(17, RightFFDPTIP);
                Paint_Patch(18, RightFFDPTH);
                Paint_Patch(19, RightFFMPTH);
                Paint_Patch(20, RightFFMPMF);
                Paint_Patch(21, RightTHDPMID);
                Paint_Patch(22, RightTHDPTIP);
                Paint_Patch(23, RightTHDPFF);
                Paint_Patch(24, RightFFPPMF);
                Paint_Patch(25, RightTHDPTH);
                Paint_Patch(26, RightTHMPTH);
                Paint_Patch(27, RightTHMPFF);
                Paint_Patch(28, RightFFMPMID);
                Paint_Patch(29, RightFFDPMID);
                Paint_Patch(30, RightFFPPTH);
                Paint_Patch(31, RightRFMPMF);
                Paint_Patch(32, RightRFDPMID);
                Paint_Patch(33, RightRFDPLF);
                Paint_Patch(34, RightRFDPTIP);
                Paint_Patch(35, RightRFMPMID);
                Paint_Patch(36, RightRFMPLF);
                Paint_Patch(37, RightMFMPFF);
                Paint_Patch(38, RightMFMPRF);
                Paint_Patch(39, RightMFDPTIP);
                Paint_Patch(40, RightMFDPRF);
                Paint_Patch(41, RightMFPP);
                Paint_Patch(42, RightMFDPFF);
                Paint_Patch(43, RightMFMPMID);
                Paint_Patch(44, RightMFDPMID);
                Paint_Patch(46, RightRFDPMF);
                Paint_Patch(47, RightRFPP);
                Paint_Patch(48, RightPalmMIDL);
                Paint_Patch(49, RightPalmMIDR);
                Paint_Patch(50, RightPalmMIDBR);
                Paint_Patch(51, RightPalmLF);
                Paint_Patch(53, RightPalmUpFF);
                Paint_Patch(54, RightPalmUpLF);
                Paint_Patch(55, RightPalmUpMF);
                Paint_Patch(56, RightPalmTHL);
                Paint_Patch(57, RightPalmTHD);
                Paint_Patch(58, RightPalmTHU);
                Paint_Patch(59, RightPalmUpRF);
                Paint_Patch(60, RightPalmTHR);
                Paint_Patch(61, RightPalmMIDU);
                Paint_Patch(62, RightPalmMIDBL);
            }
            else
            {
                // Left
                Paint_Patch(0, LeftTHDPTIP);
                Paint_Patch(1, LeftTHDPTH);
                Paint_Patch(2, LeftTHDPMID);
                Paint_Patch(3, LeftTHDPFF);
                Paint_Patch(4, LeftTHMPTH);
                Paint_Patch(5, LeftTHMPFF);
                Paint_Patch(6, LeftFFDPTIP);
                Paint_Patch(7, LeftFFDPTH);
                Paint_Patch(8, LeftFFDPMID);
                Paint_Patch(9, LeftFFDPMF);
                Paint_Patch(10, LeftFFMPTH);
                Paint_Patch(11, LeftFFMPMID);
                Paint_Patch(12, LeftFFMPMF);
                Paint_Patch(13, LeftFFPPTH);
                Paint_Patch(14, LeftFFPPMF);
                Paint_Patch(15, LeftMFDPTIP);
                Paint_Patch(16, LeftMFDPFF);
                Paint_Patch(17, LeftMFDPMID);
                Paint_Patch(18, LeftMFDPRF);
                Paint_Patch(19, LeftMFMPFF);
                Paint_Patch(20, LeftMFMPMID);
                Paint_Patch(21, LeftMFMPRF);
                Paint_Patch(22, LeftMFPP);
                Paint_Patch(23, LeftRFDPTIP);
                Paint_Patch(24, LeftRFDPMF);
                Paint_Patch(25, LeftRFDPMID);
                Paint_Patch(26, LeftRFDPLF);
                Paint_Patch(27, LeftRFMPMF);
                Paint_Patch(28, LeftRFMPMID);
                Paint_Patch(29, LeftRFMPLF);
                Paint_Patch(30, LeftRFPP);
                Paint_Patch(31, LeftLFDPTIP);
                Paint_Patch(32, LeftLFDPRF);
                Paint_Patch(33, LeftLFDPMID);
                Paint_Patch(34, LeftLFDPLF);
                Paint_Patch(35, LeftLFMPRF);
                Paint_Patch(36, LeftLFMPMID);
                Paint_Patch(37, LeftLFMPLF);
                Paint_Patch(38, LeftLFPPRF);
                Paint_Patch(39, LeftLFPPLF);
                Paint_Patch(40, LeftPalmUpFF);
                Paint_Patch(41, LeftPalmUpMF);
                Paint_Patch(42, LeftPalmUpRF);
                Paint_Patch(43, LeftPalmUpLF);
                Paint_Patch(44, LeftPalmTHL);
                Paint_Patch(45, LeftPalmTHU);
                Paint_Patch(46, LeftPalmTHD);
                Paint_Patch(47, LeftPalmTHR);
                Paint_Patch(48, LeftPalmMIDU);
                Paint_Patch(49, LeftPalmMIDL);
                Paint_Patch(50, LeftPalmMIDR);
                Paint_Patch(51, LeftPalmMIDBL);
                Paint_Patch(52, LeftPalmMIDBR);
                Paint_Patch(53, LeftPalmLF);
            }

            // Update the iaOldTaxelValues variable for comparison on next round
            iaTaxelValues.CopyTo(iaOldTaxelValues, 0);
        }

        // Update a single taxel patch, but only if it meets the criteria
        private void Paint_Patch(int iTaxelID, System.Windows.Shapes.Path pPatch)
        {
            const int iTHRESHOLD = 100; // Detection threshold in scale 0-4095 (0-no contact, 4095-full pressure)

            // Update patch only if
            // * old value and new value differ
            // AND
            // * (old OR new values are over the detection threshold (no update for very low noise))

            if (iaTaxelValues[iTaxelID] != iaOldTaxelValues[iTaxelID])
                if (((iaOldTaxelValues[iTaxelID] > iTHRESHOLD) || (iaTaxelValues[iTaxelID] > iTHRESHOLD)))
                {
                    if (baInitialTaxelValueAvailable[iTaxelID])
                    {
                        // Stretch range according to initial value
                        int iRestOfTheRange = 4095 - iaInitialTaxelValues[iTaxelID];
                        double dAmplifyCoefficient = 4095.0 / (double)(iRestOfTheRange);
                        pPatch.Fill = Gradient((int)(dAmplifyCoefficient * (double)iaTaxelValues[iTaxelID]));
                    }
                    else
                    {
                        // Unadjusted range
                        pPatch.Fill = Gradient(iaTaxelValues[iTaxelID]);
                    }
                }
                else
                {
                    SolidColorBrush mySolidColorBrush = new SolidColorBrush();
                    mySolidColorBrush.Color = Color.FromArgb(255, 0, 50, 0);
                    pPatch.Fill = mySolidColorBrush;
                }
        }

        // Color code the input range of 0 to 4095 into beautiful color gradient
        private SolidColorBrush Gradient(int iTexelValue)
        {
            // Limit the input between 0 and 4095
            iTexelValue = Math.Max(0, iTexelValue);
            iTexelValue = Math.Min(4095, iTexelValue);

            SolidColorBrush mySolidColorBrush = new SolidColorBrush();

            if (iTexelValue < 1366) // Dark green to light green (0,50,0 -> 0,255,0)
                mySolidColorBrush.Color = Color.FromArgb(255, 0, Math.Min((byte)255, (byte)(50 + ((double)iTexelValue / 6.65))), 0);
            else
                if (iTexelValue < 2731) // Light green to yellow (0,255,0 -> 255,255,0)
                    mySolidColorBrush.Color = Color.FromArgb(255, Math.Min((byte)255, (byte)((double)(iTexelValue - 1365) / 5.35)), 255, 0);
                else // Yellow to red (255,255,0 -> 255,0,0)
                    mySolidColorBrush.Color = Color.FromArgb(255, 255, Math.Min((byte)255, (byte)(255 - ((double)(iTexelValue - 1365) / 5.35))), 0);

            return (mySolidColorBrush);
        }

        // GloveGrid_SizeChanged event gets called, when the User GUI resize triggers GloveGrid size change.
        // It recalculates the scaling factor for the glove inside the grid.
        private void GloveGrid_SizeChanged(object sender, SizeChangedEventArgs e)
        {
            // Calculate the scale relative to default window size (700H x 780W)
            double yScale = ActualHeight / 700f;
            double xScale = ActualWidth / 780f;
            // Use proportional scaling, as this does not distort the image.
            // Thus use the smalles value of both axis to avoid image clipping.
            double value = Math.Min(xScale, yScale);
            // The scale needs to be available on custom property to be able to access from XAML
            ScaleValue = (double)OnCoerceScaleValue(wMainWindow, value);
        }

        #region ScaleValue Depdency Property
        // Register Custom Property to change the scaling value from XAML
        public static readonly DependencyProperty ScaleValueProperty = DependencyProperty.Register("ScaleValue", typeof(double), typeof(MainWindow), new UIPropertyMetadata(1.0, new PropertyChangedCallback(OnScaleValueChanged), new CoerceValueCallback(OnCoerceScaleValue)));

        private static object OnCoerceScaleValue(DependencyObject o, object value)
        {
            MainWindow mainWindow = o as MainWindow;
            if (mainWindow != null)
                return mainWindow.OnCoerceScaleValue((double)value);
            else
                return value;
        }

        private static void OnScaleValueChanged(DependencyObject o, DependencyPropertyChangedEventArgs e)
        {
            MainWindow mainWindow = o as MainWindow;
            if (mainWindow != null)
                mainWindow.OnScaleValueChanged((double)e.OldValue, (double)e.NewValue);
        }

        protected virtual double OnCoerceScaleValue(double value)
        {
            // In case the parameter value is not a number, return to default scale 1:1
            if (double.IsNaN(value))
                return 1.0f;

            // Do not go under 50% of original resolution
            value = Math.Max(0.5, value);
            return value;
        }

        protected virtual void OnScaleValueChanged(double oldValue, double newValue)
        {

        }

        public double ScaleValue
        {
            get { return (double)GetValue(ScaleValueProperty); }
            set { SetValue(ScaleValueProperty, value); }
        }
        #endregion

        private void cbLeftOrRight_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (LeftCanvasGlove != null)
            {
                if (cbLeftOrRight.SelectedIndex == 0)
                    LeftCanvasGlove.Visibility = Visibility.Hidden;
                else
                    LeftCanvasGlove.Visibility = Visibility.Visible;
            }

            if (RightCanvasGlove != null)
            {
                if (cbLeftOrRight.SelectedIndex == 0)
                    RightCanvasGlove.Visibility = Visibility.Visible;
                else
                    RightCanvasGlove.Visibility = Visibility.Hidden;
            }
        }

        private void btBias_Click(object sender, RoutedEventArgs e)
        {
            for (int i = 0; i < MAXCHANNELS; i++)
            {
                baInitialTaxelValueAvailable[i] = false;
                iaInitialTaxelValues[i] = 0;
            }
        }
    }
}
