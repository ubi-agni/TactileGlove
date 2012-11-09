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

        public MainWindow()
        {
            InitializeComponent();
            btConnectDisconnect.Content = sCONNECT;

            string[] saAvailableSerialPorts = SerialPort.GetPortNames();
            foreach (string sAvailableSerialPort in saAvailableSerialPorts)
                cbSerialPort.Items.Add(sAvailableSerialPort);
            if (cbSerialPort.Items.Count > 0)
            {
                cbSerialPort.Text = cbSerialPort.Items[0].ToString();
                for (int i = 0; i < cbSerialPort.Items.Count; i++)
                    // default to COM1 initially if available
                    if (cbSerialPort.Items[i].ToString() == "COM1")
                        cbSerialPort.SelectedIndex = i;
            }
        }

        private void slider1_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            // Color goes from RGB (0,50,0) to (0,255,0) to (255,255,0) to (255,0,0)

            SolidColorBrush mySolidColorBrush = new SolidColorBrush();
            mySolidColorBrush.Color = Color.FromArgb((byte)slider1.Value, 255, 255, 0);
            PalmUpRF.Fill = mySolidColorBrush;
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

    }
}
