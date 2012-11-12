// Graphical user interface for left tactile dataglove v1
//#define LOG

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
        private const string sCONNECT = "_Connect";
        private const string sDISCONNECT = "_Disconnect";

        private SerialPort spUSB = new SerialPort();

        // GUI update timer
        private DispatcherTimer dispatcherTimer = new DispatcherTimer();

        const int CQUEUESIZE = 1024 * 1024;
        private Queue<byte> qbReceiveQueue = new Queue<byte>(CQUEUESIZE);
        private byte[] baLastRemaining = new byte[4];
        private int iLastRemaining;

        private int[] iaTaxelValues = new int[64];
        private int[] iaOldTaxelValues = new int[64];

        // Lock object to regulate access from multiple threads:
        static private readonly object locker = new object();

#if LOG
        private string sFileName = "log.txt";
        private TextWriter twLog;
#endif

        public MainWindow()
        {
            InitializeComponent();
            btConnectDisconnect.Content = sCONNECT;
            spUSB.DataReceived += new SerialDataReceivedEventHandler(spUSB_DataReceived);

            dispatcherTimer.Tick += new EventHandler(dispatcherTimer_Tick);
            dispatcherTimer.Interval = new TimeSpan(0, 0, 0, 0, 20); // 20ms -> 50 Hz Update rate

            string[] saAvailableSerialPorts = SerialPort.GetPortNames();
            foreach (string sAvailableSerialPort in saAvailableSerialPorts)
                cbSerialPort.Items.Add(sAvailableSerialPort);
            if (cbSerialPort.Items.Count > 0)
            {
                cbSerialPort.Text = cbSerialPort.Items[0].ToString();
                for (int i = 0; i < cbSerialPort.Items.Count; i++)
                    // default to COM4 initially if available
                    if (cbSerialPort.Items[i].ToString() == "COM4")
                        cbSerialPort.SelectedIndex = i;
            }
        }

        private void btConnectDisconnect_Click(object sender, RoutedEventArgs e)
        {
            if ((string)btConnectDisconnect.Content == sCONNECT)
            {
                // Trying to connect
                try
                {
#if LOG
                    twLog = new StreamWriter(sFileName);
#endif
                    qbReceiveQueue.Clear();
                    iLastRemaining = 0;

                    for (int i = 0; i < iaTaxelValues.Length; i++)
                    {
                        iaTaxelValues[i] = 0;
                        iaOldTaxelValues[i] = 4095; // Force a GUI update
                    }

                    spUSB.BaudRate = 115200;
                    spUSB.DataBits = 8;
                    spUSB.DiscardNull = false;
                    spUSB.DtrEnable = false;
                    spUSB.Handshake = Handshake.None;
                    spUSB.Parity = Parity.None;
                    spUSB.ParityReplace = 63;
                    spUSB.PortName = cbSerialPort.SelectedItem.ToString();
                    spUSB.ReadBufferSize = 4096;
                    spUSB.ReadTimeout = -1;
                    spUSB.ReceivedBytesThreshold = 5 * 64; // Packet size * Taxel count
                    spUSB.RtsEnable = true;
                    spUSB.StopBits = StopBits.One;
                    spUSB.WriteBufferSize = 2048;
                    spUSB.WriteTimeout = -1;

                    if (!spUSB.IsOpen)
                        spUSB.Open();

                    btConnectDisconnect.Content = sDISCONNECT;
                    cbSerialPort.IsEnabled = false;

                    dispatcherTimer.Start();
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Could not open the communication port!" + "\n" + "Error: " + ex.Message);
                }
            }
            else
            {
                // Disconnect
                try
                {
                    dispatcherTimer.Stop();
                    if (spUSB.IsOpen)
                        spUSB.Close();
                    cbSerialPort.IsEnabled = true;
                    btConnectDisconnect.Content = sCONNECT;
#if LOG
                    if (twLog != null)
                    {
                        twLog.Close();
                        twLog = null;
                    }
#endif

                    // Reset taxels on GUI to idle state
                    for (int i = 0; i < iaTaxelValues.Length; i++)
                        iaTaxelValues[i] = 0;

                    
                    Paint_Taxels();
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Could not close the communication port!" + "\n" + "Error: " + ex.Message);
                }
            }
        }

        private void spUSB_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            // This function runs in a different thread as the frmMain elements
            byte[] baSerialPortIn = new byte[spUSB.BytesToRead];
            int iDataLength = baSerialPortIn.Length;
            if (iDataLength > 0)
            {
                spUSB.Read(baSerialPortIn, 0, iDataLength);
                lock (locker)
                    baSerialPortIn.ToList().ForEach(b => qbReceiveQueue.Enqueue(b));
            }
        }

        private void dispatcherTimer_Tick(object sender, EventArgs e)
        {
            ProcessData();
        }

        private void ProcessData()
        {
            byte[] baDataIn;
            lock (locker)
            {
                //baDataIn = new byte[qbReceiveQueue.Count];
                baDataIn = qbReceiveQueue.ToArray();

//                foreach (byte by in qbReceiveQueue)
                    //qbProcessQueue.Enqueue(by);
                qbReceiveQueue.Clear();
            }
#if LOG
            twLog.WriteLine("{0:hh:mm:ss.ffff} Length: " + baDataIn.Length.ToString() + " Data: " + BitConverter.ToString(baDataIn), DateTime.Now);
            twLog.WriteLine("{0:hh:mm:ss.ffff} Starting processing. Lastremaining: " + iLastRemaining.ToString(), DateTime.Now);
#endif

            const int iBYTESINPACKET = 5;

            // Create new byte array including the previous rest
            byte[] baWorking = new byte[iLastRemaining + baDataIn.Length];
            //int iPrevWorkPointer = 0;

            if (iLastRemaining > 0)
            {
                baLastRemaining.CopyTo(baWorking, 0);

                /* for (int i = 0; i < iLastRemaining; i++)
                {
                    baWorking[i] = baLastRemaining[i];
                    iPrevWorkPointer++;
                } */
            }
            if (baDataIn.Length > 0)
            {
                baDataIn.CopyTo(baWorking, iLastRemaining);
                /*
                for (int i = 0; i < baDataIn.Length; i++)
                {
                    baWorking[iPrevWorkPointer + i] = baDataIn[i];
                } */
            }

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

                    iaTaxelValues[iChannel] = 4095 - (256 * (0x0F & baWorking[iStartPointer + 2]) + baWorking[iStartPointer + 3]);

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
                iLastRemaining = 0;
                for (int i = 0; i < baWorking.Length - iStartPointer; i++)
                {
                    baLastRemaining[i] = baWorking[iStartPointer + i];
                    iLastRemaining++;
                }
            }
            else
            {
                iLastRemaining = 0;
            }
#if LOG
            twLog.WriteLine("{0:hh:mm:ss.ffff} Ending processing.", DateTime.Now);
#endif
            Paint_Taxels();
        }

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

            iaTaxelValues.CopyTo(iaOldTaxelValues, 0);
        }

        private void Paint_Patch(int iID, System.Windows.Shapes.Path pPatch)
        {
            const int iTHRESHOLD = 10; // Detection threshold in scale 0-4095 (0-no contact, 4095-full pressure)

            // Update patch only if
            // * old value and new value differ
            // AND
            // * (old OR new values are over the detection threshold (no need to update for signal noise))

            if ((iaTaxelValues[iID] != iaOldTaxelValues[iID]) &&
                ((iaOldTaxelValues[iID] > iTHRESHOLD) || (iaTaxelValues[iID] > iTHRESHOLD)))
                pPatch.Fill = Gradient(iaTaxelValues[iID]);
        }

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
    }
}
