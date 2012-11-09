// Graphical user interface for left tactile dataglove v1

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

        private List<byte> lRxData = new List<byte>();

        private int[] iaTaxelValues = new int[64];

        // Lock object to regulate access from multiple threads:
        static private readonly object locker = new object();

        public MainWindow()
        {
            InitializeComponent();
            btConnectDisconnect.Content = sCONNECT;
            spUSB.DataReceived += new SerialDataReceivedEventHandler(spUSB_DataReceived);

            dispatcherTimer.Tick += new EventHandler(dispatcherTimer_Tick);
            dispatcherTimer.Interval = new TimeSpan(0,0,0,0,20); // 20ms -> 50 Hz Update rate

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
                try
                {
                    for (int i = 0; i < iaTaxelValues.Length; i++)
                        iaTaxelValues[i] = 0;

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
                    spUSB.ReceivedBytesThreshold = 1;
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
                try
                {
                    dispatcherTimer.Stop();
                    if (spUSB.IsOpen)
                        spUSB.Close();
                    cbSerialPort.IsEnabled = true;
                    btConnectDisconnect.Content = sCONNECT;

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

        private delegate void UpdateUiByteInDelegate(byte[] byDataIn);

        private void spUSB_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            // This function runs in a different thread as the frmMain elements
            byte[] bySerialPortIn = new byte[spUSB.BytesToRead];
            int iDataLength = bySerialPortIn.Length;
            if (iDataLength > 0)
            {
                spUSB.Read(bySerialPortIn, 0, iDataLength);
                Dispatcher.Invoke(DispatcherPriority.Send, new UpdateUiByteInDelegate(DataFromGlove), bySerialPortIn);
            }
        }

        private SolidColorBrush Gradient(int iTexelValue)
        {
            // Limit the input between 0 and 4095
            iTexelValue = Math.Max(0, iTexelValue);
            iTexelValue = Math.Min(4095, iTexelValue);

            SolidColorBrush mySolidColorBrush = new SolidColorBrush();

            if (iTexelValue < 1366)
                mySolidColorBrush.Color = Color.FromArgb(255, 0, Math.Min((byte)255, (byte)(50 + ((double)iTexelValue / 6.65))), 0);
            else
                if (iTexelValue < 2731)
                    mySolidColorBrush.Color = Color.FromArgb(255, Math.Min((byte)255, (byte)((double)(iTexelValue - 1365) / 5.35)), 255, 0);
                else
                    mySolidColorBrush.Color = Color.FromArgb(255, 255, Math.Min((byte)255, (byte)(255 - ((double)(iTexelValue - 1365) / 5.35))), 0);

            return (mySolidColorBrush);
        }

        private void DataFromGlove(byte[] byDataIn)
        {
            const int iBYTESINPACKET = 5;
            lRxData.AddRange(byDataIn);

            // Continue only if queue has at least the size of a full frame (5 bytes)
            if (lRxData.Count >= iBYTESINPACKET)
            {
                bool bDoneProcessing = false;
                do
                {
                    byte[] byOnePacket = new byte[iBYTESINPACKET];
                    // Start reading from beginning

                    lRxData.CopyTo(0, byOnePacket, 0, iBYTESINPACKET);

                    // Plausibility check
                    // First byte must be between 0x3C and 0x7B
                    // Second byte must be 0x01
                    // Fifth byte must be 0x00
                    if (((byOnePacket[0] >= 0x3C) && (byOnePacket[0] <= 0x7B)) &&
                        (byOnePacket[1] == 0x01) &&
                        (byOnePacket[4] == 0x00))
                    {
                        // Save the data
                        int iChannel = byOnePacket[0] - 0x3C;
                        if ((iChannel < 0) && (iChannel >= 64))
                            throw new System.ArgumentException("Unvalid Taxel number received");

                        lock (locker)
                            iaTaxelValues[iChannel] = 4095 - (256 * (0x0F & byOnePacket[2]) + byOnePacket[3]);

                        // Remove all processed bytes from queue (one packet);
                        lRxData.RemoveRange(0, iBYTESINPACKET);
                    }
                    else
                    {
                        // Failed plausibility check. Remove first byte from queue.
                        lRxData.RemoveAt(0);
                    }

                    // If we have less than one packet in queue, quit for now
                    if (lRxData.Count < iBYTESINPACKET)
                        bDoneProcessing = true;
                } while (!bDoneProcessing);
            }

        }

        private void dispatcherTimer_Tick(object sender, EventArgs e)
        {
            Paint_Taxels();
        }

        private void Paint_Taxels()
        {
            lock (locker)
            {
                THDPTIP.Fill = Gradient(iaTaxelValues[0]);
                THDPTH.Fill = Gradient(iaTaxelValues[1]);
                THDPMID.Fill = Gradient(iaTaxelValues[2]);
                THDPFF.Fill = Gradient(iaTaxelValues[3]);
                THMPTH.Fill = Gradient(iaTaxelValues[4]);
                THMPFF.Fill = Gradient(iaTaxelValues[5]);
                FFDPTIP.Fill = Gradient(iaTaxelValues[6]);
                FFDPTH.Fill = Gradient(iaTaxelValues[7]);
                FFDPMID.Fill = Gradient(iaTaxelValues[8]);
                FFDPMF.Fill = Gradient(iaTaxelValues[9]);
                FFMPTH.Fill = Gradient(iaTaxelValues[10]);
                FFMPMID.Fill = Gradient(iaTaxelValues[11]);
                FFMPMF.Fill = Gradient(iaTaxelValues[12]);
                FFPPTH.Fill = Gradient(iaTaxelValues[13]);
                FFPPMF.Fill = Gradient(iaTaxelValues[14]);
                MFDPTIP.Fill = Gradient(iaTaxelValues[15]);
                MFDPFF.Fill = Gradient(iaTaxelValues[16]);
                MFDPMID.Fill = Gradient(iaTaxelValues[17]);
                MFDPRF.Fill = Gradient(iaTaxelValues[18]);
                MFMPFF.Fill = Gradient(iaTaxelValues[19]);
                MFMPMID.Fill = Gradient(iaTaxelValues[20]);
                MFMPRF.Fill = Gradient(iaTaxelValues[21]);
                MFPP.Fill = Gradient(iaTaxelValues[22]);
                RFDPTIP.Fill = Gradient(iaTaxelValues[23]);
                RFDPMF.Fill = Gradient(iaTaxelValues[24]);
                RFDPMID.Fill = Gradient(iaTaxelValues[25]);
                RFDPLF.Fill = Gradient(iaTaxelValues[26]);
                RFMPMF.Fill = Gradient(iaTaxelValues[27]);
                RFMPMID.Fill = Gradient(iaTaxelValues[28]);
                RFMPLF.Fill = Gradient(iaTaxelValues[29]);
                RFPP.Fill = Gradient(iaTaxelValues[30]);
                LFDPTIP.Fill = Gradient(iaTaxelValues[31]);
                LFDPRF.Fill = Gradient(iaTaxelValues[32]);
                LFDPMID.Fill = Gradient(iaTaxelValues[33]);
                LFDPLF.Fill = Gradient(iaTaxelValues[34]);
                LFMPRF.Fill = Gradient(iaTaxelValues[35]);
                LFMPMID.Fill = Gradient(iaTaxelValues[36]);
                LFMPLF.Fill = Gradient(iaTaxelValues[37]);
                LFPPRF.Fill = Gradient(iaTaxelValues[38]);
                LFPPLF.Fill = Gradient(iaTaxelValues[39]);
                PalmUpFF.Fill = Gradient(iaTaxelValues[40]);
                PalmUpMF.Fill = Gradient(iaTaxelValues[41]);
                PalmUpRF.Fill = Gradient(iaTaxelValues[42]);
                PalmUpLF.Fill = Gradient(iaTaxelValues[43]);
                PalmTHL.Fill = Gradient(iaTaxelValues[44]);
                PalmTHU.Fill = Gradient(iaTaxelValues[45]);
                PalmTHD.Fill = Gradient(iaTaxelValues[46]);
                PalmTHR.Fill = Gradient(iaTaxelValues[47]);
                PalmMIDU.Fill = Gradient(iaTaxelValues[48]);
                PalmMIDL.Fill = Gradient(iaTaxelValues[49]);
                PalmMIDR.Fill = Gradient(iaTaxelValues[50]);
                PalmMIDBL.Fill = Gradient(iaTaxelValues[51]);
                PalmMIDBR.Fill = Gradient(iaTaxelValues[52]);
                PalmLF.Fill = Gradient(iaTaxelValues[53]);
            }
        }
    }
}
