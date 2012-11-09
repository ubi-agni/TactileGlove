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

        List<byte> lRxData = new List<byte>();

        //private const int iRXBUFSIZE = 1024 * 1024;
        //private LinkedList<byte> llRXBuffer = new LinkedList<byte>(iRXBUFSIZE);
        //private byte[] byRXBuffer = new byte[iRXBUFSIZE];

        public MainWindow()
        {
            InitializeComponent();
            btConnectDisconnect.Content = sCONNECT;
            spUSB.DataReceived += new SerialDataReceivedEventHandler(spUSB_DataReceived);



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
                    if (spUSB.IsOpen)
                        spUSB.Close();
                    cbSerialPort.IsEnabled = true;
                    btConnectDisconnect.Content = sCONNECT;
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
                        // Process the data
                        int bChannel = byOnePacket[0] - 0x3C + 1;
                        int uiCellValue = 4095 - (256 * (0x0F & byOnePacket[2]) + byOnePacket[3]);

                        switch (bChannel)
                        {
                            case (1): THDPTIP.Fill = Gradient(uiCellValue); break;
                            case (2): THDPTH.Fill = Gradient(uiCellValue); break;
                            case (3): THDPMID.Fill = Gradient(uiCellValue); break;
                            case (4): THDPFF.Fill = Gradient(uiCellValue); break;
                            case (5): THMPTH.Fill = Gradient(uiCellValue); break;
                            case (6): THMPFF.Fill = Gradient(uiCellValue); break;
                            case (7): FFDPTIP.Fill = Gradient(uiCellValue); break;
                            case (8): FFDPTH.Fill = Gradient(uiCellValue); break;
                            case (9): FFDPMID.Fill = Gradient(uiCellValue); break;
                            case (10): FFDPMF.Fill = Gradient(uiCellValue); break;

                        }
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
    }
}
