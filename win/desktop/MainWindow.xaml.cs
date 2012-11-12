// *** Graphical user interface for left tactile dataglove v1 ***
//
// Modified:
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

        // Variable declarations
        private SerialPort spUSB = new SerialPort(); // Communication over (virtual) serial port 
        private Queue<byte> qbReceiveQueue = new Queue<byte>(CQUEUESIZE); // Receive queue (FIFO)
        static private readonly object locker = new object(); // Queue locker object to regulate access from multiple threads
        private uint uiLastRemaining; // Saves the count of remaning bytes from last packet parser run (value range is 0 and 4)
        private byte[] baLastRemaining = new byte[4]; // Remaining byte from last packet parser run
        private uint[] iaTaxelValues = new uint[64]; // Array holding the parsed taxel values (value range is 0 to 4095)
        private uint[] iaOldTaxelValues = new uint[64]; // Array for comparison if update is required, holding the last run taxel values
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
                    if (cbSerialPort.Items[i].ToString() == "COM4") // If available, default to COM4 initially 
                        cbSerialPort.SelectedIndex = i;
            }
        }

        // btConnectDisconnect_Click gets calles each time a button on MainWindow is pressed
        private void btConnectDisconnect_Click(object sender, RoutedEventArgs e)
        {
            // Check according to button content, if the "Connect" or "Disconnect" was pressed
            if ((string)btConnectDisconnect.Content == sCONNECT)
            {
                // Connect was pressed

                // Initialize variables
                qbReceiveQueue.Clear(); // Clear the receive queue
                uiLastRemaining = 0; // Set the last packet parses "pointer" to 0
                for (int i = 0; i < iaTaxelValues.Length; i++) // Initialize taxel arrays
                {
                    iaTaxelValues[i] = 0;
                    iaOldTaxelValues[i] = 4095; // This forces an GUI update on initial round, as the value differs from iaTaxelValue
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
                for (int i = 0; i < iaTaxelValues.Length; i++)
                    iaTaxelValues[i] = 0;

                // Redraw the GUI manually to show this
                Paint_Taxels();
            }
        }

        // spUSB_DataReceived event procedure that gets called when data from (virtual) serial port is received.
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

        // dtGUIUpdateTimer_Tick event procedure gets called when the GUI update timer fires.
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

                    iaTaxelValues[iChannel] = (uint)Math.Max(0, 4095 - (256 * (0x0F & baWorking[iStartPointer + 2]) + baWorking[iStartPointer + 3]));

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
            Paint_Patch(0, THDPTIP);
            Paint_Patch(1, THDPTH);
            Paint_Patch(2, THDPMID);
            Paint_Patch(3, THDPFF);
            Paint_Patch(4, THMPTH);
            Paint_Patch(5, THMPFF);
            Paint_Patch(6, FFDPTIP);
            Paint_Patch(7, FFDPTH);
            Paint_Patch(8, FFDPMID);
            Paint_Patch(9, FFDPMF);
            Paint_Patch(10, FFMPTH);
            Paint_Patch(11, FFMPMID);
            Paint_Patch(12, FFMPMF);
            Paint_Patch(13, FFPPTH);
            Paint_Patch(14, FFPPMF);
            Paint_Patch(15, MFDPTIP);
            Paint_Patch(16, MFDPFF);
            Paint_Patch(17, MFDPMID);
            Paint_Patch(18, MFDPRF);
            Paint_Patch(19, MFMPFF);
            Paint_Patch(20, MFMPMID);
            Paint_Patch(21, MFMPRF);
            Paint_Patch(22, MFPP);
            Paint_Patch(23, RFDPTIP);
            Paint_Patch(24, RFDPMF);
            Paint_Patch(25, RFDPMID);
            Paint_Patch(26, RFDPLF);
            Paint_Patch(27, RFMPMF);
            Paint_Patch(28, RFMPMID);
            Paint_Patch(29, RFMPLF);
            Paint_Patch(30, RFPP);
            Paint_Patch(31, LFDPTIP);
            Paint_Patch(32, LFDPRF);
            Paint_Patch(33, LFDPMID);
            Paint_Patch(34, LFDPLF);
            Paint_Patch(35, LFMPRF);
            Paint_Patch(36, LFMPMID);
            Paint_Patch(37, LFMPLF);
            Paint_Patch(38, LFPPRF);
            Paint_Patch(39, LFPPLF);
            Paint_Patch(40, PalmUpFF);
            Paint_Patch(41, PalmUpMF);
            Paint_Patch(42, PalmUpRF);
            Paint_Patch(43, PalmUpLF);
            Paint_Patch(44, PalmTHL);
            Paint_Patch(45, PalmTHU);
            Paint_Patch(46, PalmTHD);
            Paint_Patch(47, PalmTHR);
            Paint_Patch(48, PalmMIDU);
            Paint_Patch(49, PalmMIDL);
            Paint_Patch(50, PalmMIDR);
            Paint_Patch(51, PalmMIDBL);
            Paint_Patch(52, PalmMIDBR);
            Paint_Patch(53, PalmLF);

            // Update the iaOldTaxelValues variable for comparison on next round
            iaTaxelValues.CopyTo(iaOldTaxelValues, 0);
        }

        // Update a single taxel patch, but only if it meets the criteria
        private void Paint_Patch(int iTaxelID, System.Windows.Shapes.Path pPatch)
        {
            const int iTHRESHOLD = 10; // Detection threshold in scale 0-4095 (0-no contact, 4095-full pressure)

            // Update patch only if
            // * old value and new value differ
            // AND
            // * (old OR new values are over the detection threshold (no update for very low noise))

            if ((iaTaxelValues[iTaxelID] != iaOldTaxelValues[iTaxelID]) &&
                ((iaOldTaxelValues[iTaxelID] > iTHRESHOLD) || (iaTaxelValues[iTaxelID] > iTHRESHOLD)))
                pPatch.Fill = Gradient(iaTaxelValues[iTaxelID]);
        }

        // Color code the input range of 0 to 4095 into beautiful color gradient
        private SolidColorBrush Gradient(uint uiTexelValue)
        {
            // Limit the input between 0 and 4095
            uiTexelValue = Math.Max(0, uiTexelValue);
            uiTexelValue = Math.Min(4095, uiTexelValue);

            SolidColorBrush mySolidColorBrush = new SolidColorBrush();

            if (uiTexelValue < 1366) // Dark green to light green (0,50,0 -> 0,255,0)
                mySolidColorBrush.Color = Color.FromArgb(255, 0, Math.Min((byte)255, (byte)(50 + ((double)uiTexelValue / 6.65))), 0);
            else
                if (uiTexelValue < 2731) // Light green to yellow (0,255,0 -> 255,255,0)
                    mySolidColorBrush.Color = Color.FromArgb(255, Math.Min((byte)255, (byte)((double)(uiTexelValue - 1365) / 5.35)), 255, 0);
                else // Yellow to red (255,255,0 -> 255,0,0)
                    mySolidColorBrush.Color = Color.FromArgb(255, 255, Math.Min((byte)255, (byte)(255 - ((double)(uiTexelValue - 1365) / 5.35))), 0);

            return (mySolidColorBrush);
        }
    }
}
