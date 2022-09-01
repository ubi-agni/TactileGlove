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
//using System.IO;
//using System.IO.Ports;

namespace TactileDataglove
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        // Constant declarations
        private const int MAXCHANNELS = 64;

        //private TextWriter tw;
        private string[] saNameMapping = new string[64];

        // Variable declarations
        private int[] iaTaxelValues = new int[MAXCHANNELS]; // Array holding the parsed taxel values (value range is 0 to 4095)

        // MainWindow constructor gets called when the window is created - at the start of the program
        public MainWindow()
        {
            InitializeComponent();

            saNameMapping[5] = "RightLFDPLF"; iaTaxelValues[5] = 4095;
            saNameMapping[6] = "RightLFDPTIP"; iaTaxelValues[6] = 4095;
            saNameMapping[7] = "RightLFDPMID"; iaTaxelValues[7] = 4095;
            saNameMapping[8] = "RightLFMPRF"; iaTaxelValues[8] = 4095;
            saNameMapping[9] = "RightLFDPRF"; iaTaxelValues[9] = 4095;
            saNameMapping[10] = "RightLFMPLF"; iaTaxelValues[10] = 4095;
            saNameMapping[11] = "RightLFMPMID"; iaTaxelValues[11] = 4095;
            saNameMapping[12] = "RightLFPPRF"; iaTaxelValues[12] = 4095;
            saNameMapping[13] = "RightLFPPLF"; iaTaxelValues[13] = 4095;
            saNameMapping[16] = "RightFFDPMF"; iaTaxelValues[16] = 4095;
            saNameMapping[17] = "RightFFDPTIP"; iaTaxelValues[17] = 4095;
            saNameMapping[18] = "RightFFDPTH"; iaTaxelValues[18] = 4095;
            saNameMapping[19] = "RightFFMPTH"; iaTaxelValues[19] = 4095;
            saNameMapping[20] = "RightFFMPMF"; iaTaxelValues[20] = 4095;
            saNameMapping[21] = "RightTHDPMID"; iaTaxelValues[21] = 4095;
            saNameMapping[22] = "RightTHDPTIP"; iaTaxelValues[22] = 4095;
            saNameMapping[23] = "RightTHDPFF"; iaTaxelValues[23] = 4095;
            saNameMapping[24] = "RightFFPPMF"; iaTaxelValues[24] = 4095;
            saNameMapping[25] = "RightTHDPTH"; iaTaxelValues[25] = 4095;
            saNameMapping[26] = "RightTHMPTH"; iaTaxelValues[26] = 4095;
            saNameMapping[27] = "RightTHMPFF"; iaTaxelValues[27] = 4095;
            saNameMapping[28] = "RightFFMPMID"; iaTaxelValues[28] = 4095;
            saNameMapping[29] = "RightFFDPMID"; iaTaxelValues[29] = 4095;
            saNameMapping[30] = "RightFFPPTH"; iaTaxelValues[30] = 4095;
            saNameMapping[31] = "RightRFMPMF"; iaTaxelValues[31] = 4095;
            saNameMapping[32] = "RightRFDPMID"; iaTaxelValues[32] = 4095;
            saNameMapping[33] = "RightRFDPLF"; iaTaxelValues[33] = 4095;
            saNameMapping[34] = "RightRFDPTIP"; iaTaxelValues[34] = 4095;
            saNameMapping[35] = "RightRFMPMID"; iaTaxelValues[35] = 4095;
            saNameMapping[36] = "RightRFMPLF"; iaTaxelValues[36] = 4095;
            saNameMapping[37] = "RightMFMPFF"; iaTaxelValues[37] = 4095;
            saNameMapping[38] = "RightMFMPRF"; iaTaxelValues[38] = 4095;
            saNameMapping[39] = "RightMFDPTIP"; iaTaxelValues[39] = 4095;
            saNameMapping[40] = "RightMFDPRF"; iaTaxelValues[40] = 4095;
            saNameMapping[41] = "RightMFPP"; iaTaxelValues[41] = 4095;
            saNameMapping[42] = "RightMFDPFF"; iaTaxelValues[42] = 4095;
            saNameMapping[43] = "RightMFMPMID"; iaTaxelValues[43] = 4095;
            saNameMapping[44] = "RightMFDPMID"; iaTaxelValues[44] = 4095;
            saNameMapping[46] = "RightRFDPMF"; iaTaxelValues[46] = 4095;
            saNameMapping[47] = "RightRFPP"; iaTaxelValues[47] = 4095;
            saNameMapping[48] = "RightPalmMIDL"; iaTaxelValues[48] = 4095;
            saNameMapping[49] = "RightPalmMIDR"; iaTaxelValues[49] = 4095;
            saNameMapping[50] = "RightPalmMIDBR"; iaTaxelValues[50] = 4095;
            saNameMapping[51] = "RightPalmLF"; iaTaxelValues[51] = 4095;
            saNameMapping[53] = "RightPalmUpFF"; iaTaxelValues[53] = 4095;
            saNameMapping[54] = "RightPalmUpLF"; iaTaxelValues[54] = 4095;
            saNameMapping[55] = "RightPalmUpMF"; iaTaxelValues[55] = 4095;
            saNameMapping[56] = "RightPalmTHL"; iaTaxelValues[56] = 4095;
            saNameMapping[57] = "RightPalmTHD"; iaTaxelValues[57] = 4095;
            saNameMapping[58] = "RightPalmTHU"; iaTaxelValues[58] = 4095;
            saNameMapping[59] = "RightPalmUpRF"; iaTaxelValues[59] = 4095;
            saNameMapping[60] = "RightPalmTHR"; iaTaxelValues[60] = 4095;
            saNameMapping[61] = "RightPalmMIDU"; iaTaxelValues[61] = 4095;
            saNameMapping[62] = "RightPalmMIDBL"; iaTaxelValues[62] = 4095;

            Paint_Taxels();
        }

        // Applies the correct taxel states to GUI
        private void Paint_Taxels()
        {
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

        // Update a single taxel patch, but only if it meets the criteria
        private void Paint_Patch(int iTaxelID, System.Windows.Shapes.Path pPatch)
        {
            pPatch.Fill = Gradient(iaTaxelValues[iTaxelID]);
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

    }
}
