#include <SPI.h>  // include the new SPI library:
#include <Wire.h>
// install the version in guihomework repository https://github.com/guihomework/SparkFun_BNO080_Arduino_Library
#include "BNO08x_Library.h"
// install the library from the svn https://svn.techfak.uni-bielefeld.de/citec/projects/AGNIserialProtocol/src/teensy/SerialProtocol
#include "SerialProtocol.h"
BNO080 myIMU;

// Serial Protocol setting
#define SP_TACTDATA_LEN  0xC0  // 192 = 64x (uint8 + uint16)
#define SP_BENDDATA_LEN  0x03  // 3 = 1x (uint8 + uint16)
#define SP_IMU_DATA_LEN  0x28  // 40 = 3x4 + 3x4 + 4x4 = 40
#define IMU_REFRESH_PERIOD  2 // in ms = 250 Hz  one cannot set 2.5 ms to get 400 Hz so using the next nice digit
/*max IMU speed
Gyro rotation Vector 1 kHz
Rotation Vector  400 Hz
Gaming Rotation Vector 400 Hz
Geomagnetic Rotation Vector 90 Hz
Gravity 400 Hz
Linear Acceleration 400 Hz
Accelerometer 500 Hz
Gyroscope 400 Hz
Magnetometer 100 Hz
*/

#define IMU_MASK_ALLNEWDATA 0x07
#define IMU_MASK_ACC        0x01
#define IMU_MASK_GYRO       0x02
#define IMU_MASK_QUAT       0x04


#define ADC_CONFIG_WRITE_REG 0x0831<<4
#define ADC_REFRESH_PERIOD  1 // in ms

#define NUM_TAXELS 64 //80
#define NUM_ADC 4             //5
#define BEND_SENSOR_ADC_ID 1  //4
#define BEND_SENSOR_ADC_CHANNEL 14  //TO BE CHECKED WITH ROS
#define NUM_CHANNEL 16

// using two incompatible SPI devices, A and B
const byte slaveA0Pin = 10; //Device A: ADC: AD7490 The order may be changed !?!! (Max11131A in newer ADC-Boards)
const byte slaveA1Pin = 9;  //The order may be changed !?!!
const byte slaveA2Pin = 20; //The order may be changed !?!!
const byte slaveA3Pin = 21; //The order may be changed !?!! / Device: Flex sensors integrated somewhere
const byte slaveBPin = 15; //Device B: IMU: BNO085 // const byte slaveCPin = 16;  //Device C: IMU: BNO085
//Further IMU pins
const byte imuWAKPin = 23;  //PS0
const byte imuINTPin = 17;  //INT
const byte imuRSTPin = 22;  //RST

bool imu_initialized = false;

const byte ledPin = 13; //orange led (Teensy3.2)
int ledState = LOW;
int imu_missing_count = 0;

elapsedMicros elapsedtime;

// buffer initialization
char tactile_buf[SP_DATA_OFFSET+SP_TIMESTAMP_LEN+SP_TACTDATA_LEN+1];
char bend_buf[SP_DATA_OFFSET+SP_TIMESTAMP_LEN+SP_BENDDATA_LEN+1];
char imu_buf[SP_DATA_OFFSET+SP_TIMESTAMP_LEN+SP_IMU_DATA_LEN+1];

// internal copies of the IMU data to sync them and send when all are new
float ax, ay, az, gx, gy, gz, qx, qy, qz, qw; // (qx, qy, qz, qw = to i,j,k, real)
byte new_imu_reports = 0;

uint16_t AnalogData[NUM_TAXELS]; // analog sensor data
byte ADC_CS[NUM_ADC] = {slaveA0Pin,slaveA1Pin,slaveA2Pin,slaveA3Pin}; //15 removed

const unsigned long message_period = 1000000;

SP_SensorConfiguration sensorConfigs[2];

IntervalTimer messageTimer;
IntervalTimer myTimer;

SPISettings settingsADC(20000000, MSBFIRST, SPI_MODE1); //AD7490  //SPI Clock Speeds:
//SPISettings settingsADC(48000000, MSBFIRST, SPI_MODE3); //Max11131  //SPI Clock Speeds:
SPISettings settingsB(20000000, MSBFIRST, SPI_MODE3); //BNO085

bool timer_started = false;
bool message_timer_started = false;

void setup() {

  // Sensor configuration including default pub period (can be changed through the serial protocol)
  sensorConfigs[0] =  SP_makeSensorConfig(SensorTypeIDEnum::TactileSensorResistive, 0x35,  SP_TACTDATA_LEN, ADC_REFRESH_PERIOD * 10);  // x0.1ms  10=1kHz, 50=200Hz, 100=100Hz
   // TODO see if one should have different period for bend sensors, as a multiple of the period of the tactile sensor period
  //sensorConfigs[1] =  SP_makeSensorConfig(SensorTypeIDE::PositionSensorAngular, 0x35, SP_BENDDATA_LEN, 100);  // x0.1ms  10=1kHz, 50=200Hz, 100=100Hz // indirectly using the same ADC loop so one single frequency there
  sensorConfigs[1] =  SP_makeSensorConfig(SensorTypeIDEnum::IMU, 0x30, SP_IMU_DATA_LEN, IMU_REFRESH_PERIOD * 10);  // x0.1ms  10=1kHz, 50=200Hz, 100=100Hz


  // set the Slave Select Pins as outputs:
  pinMode (slaveA0Pin, OUTPUT);
  pinMode (slaveA1Pin, OUTPUT);
  pinMode (slaveA2Pin, OUTPUT);
  pinMode (slaveA3Pin, OUTPUT);
  pinMode (slaveBPin, OUTPUT);

  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  Serial.setTimeout(500); //timeout of 500 ms
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

  // store the configuration for the SerialProtocol
  SP.setConfigFromStruct(DeviceTypeIDEnum::TactileGlove, 2, sensorConfigs);



  SPI.setMOSI(7);
  SPI.setMISO(8);
  SPI.setSCK(14);
  // initialize SPI:
  SPI.begin();

  // set up the SPI pins utilized on Teensy 3.2


  // activate the integrated LED
  pinMode(ledPin, OUTPUT);

  // needed to get the clock idle HIGH before talking to the BNO085
  spi_dummy_transfer();

  //Setup BNO080 to use SPI interface with default SPI port and max BNO080 clk speed of 3MHz
  imu_initialized = myIMU.beginSPI(slaveBPin, imuWAKPin, imuINTPin, imuRSTPin); //changed from slaveCPin
  // Default periodicity (IMU_REFRESH_PERIOD ms)
  if (imu_initialized)
  {
    myIMU.enableLinearAccelerometer(IMU_REFRESH_PERIOD); // m/s^2 no gravity
    myIMU.enableRotationVector(IMU_REFRESH_PERIOD);  // quat
    myIMU.enableGyro(IMU_REFRESH_PERIOD); // rad/s
  }
  // initialize ADCs (copied from old MCU)
  unsigned char adc_nr = 0;
  while(adc_nr < NUM_ADC){
    spi_transfer(adc_nr, 0xFFFF);
    spi_transfer(adc_nr, (ADC_CONFIG_WRITE_REG | (0<<10) ));
    adc_nr++;
  }

  interrupts();
  // no code below this line in the setup
}

uint8_t stat, val1, val2, result;
long counter = 0;

void loop() {
  while(1)
  {
    // display a welcome message as long as the SerialProtocol
    // did not receive valid requests from the host
    if (SP.hasClientTalked())
    {
        messageTimer.end();
        break;
    }
    else
    {
       if (!message_timer_started)
       {
         messageTimer.begin(print_message, message_period); // timer want us
         message_timer_started = true;
       }
    }
     // handle communication with the SerialProtocol
    SP.update();
  }

  while(1)
  {
    // if SerialProtocol is in streaming mode
    if (SP.isStreaming())
    {
      // if not yet started, start the acquisition timer
       if (!timer_started || SP.hasPeriodChanged())
       {
          uint16_t pub_period;
          int8_t result = SP.getPeriod(&pub_period, 1);
          if (result != SP_SENSOR_UNKNOWN_ID && result != SP_PERIOD_INACTIVE)
          {
            if (result == SP_VARIABLE_CHANGED)
            {
              myTimer.end();
            }
            // 1 kHz timer start
            myTimer.begin(read_tactile, pub_period * 100.0); // period is given in 100 us, timer wants us
            timer_started = true;
          }
       }
    }
    else
    {
      // indicate disconnected if serial not open
      if(Serial)
      {
        digitalWrite(ledPin, LOW);
      }
      else
        digitalWrite(ledPin, HIGH);
      if (timer_started)
      {
        myTimer.end();
        timer_started = false;
      }
    }

    // TODO handle periodicity change of other devices
    /*  myIMU.enableLinearAccelerometer(50);
    myIMU.enableRotationVector(50);
    myIMU.enableGyro(50);*/
    // handle communication with the SerialProtocol
    SP.update();
  }
}

// idle message for terminal debugging
void print_message()
{
  Serial.println("SerialProtocol Device: send 0xF0 C4 00 C0 F4 for config and 0xF0 C4 00 F1 C5 to start streaming" );
  if (!imu_initialized)
  {
    Serial.println("  IMU did not start properly" );
  }
}


void read_tactile()
{
  elapsedtime=0;
  // make the LED blink dim to indicate running
  counter++;
  if (counter > 500/ADC_REFRESH_PERIOD) // ON after 0.5 sec
  {
    if (counter > 1000/ADC_REFRESH_PERIOD) // for 0.5 sec
      counter = 0;
    // dim
    digitalWrite(ledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
    delayMicroseconds(10);
    digitalWrite(ledPin, LOW);    // turn the LED off by making the voltage LOW
  }

  byte taxel_counter = 0;
  byte bend_sensor_counter = 0;
  //byte SelectADC = 2;
  for (byte SelectADC=0; SelectADC < NUM_ADC; SelectADC++)
  {
    //byte channel=0;
    for (unsigned short channel=0; channel < NUM_CHANNEL; channel++)
    {
      AnalogData[taxel_counter] = spi_transfer(SelectADC, ( ADC_CONFIG_WRITE_REG | (channel<<10) ));
      // separate the bend sensors on a second "serial protocol device"     Single goniomtert in Glove P2 channel 14
     /* if (SelectADC == BEND_SENSOR_ADC_ID)
      {
        // prepare the bend buffer
        pack_adc_buffer(bend_buf+3*bend_sensor_counter, bend_sensor_counter, AnalogData[taxel_counter]);
        bend_sensor_counter++;
      }*/
      if(1) //else
      {
        // prepare the tactile buffer
        pack_adc_buffer(tactile_buf+3*taxel_counter, taxel_counter, AnalogData[taxel_counter]);
      }
      taxel_counter++;
    }
  }
  // pack and send the data in a first and second datagram
  SP.packData((void *)tactile_buf, SP_TACTDATA_LEN, 1);
  SP.publish();
 // SP.packData((void *)bend_buf, SP_BENDDATA_LEN, 2);
 // SP.publish();

  // to ensure the clock is idle high before CS of the IMU is low, we need to do a dummy transfer with the same settings as the IMU
  // but without chipselect
  spi_dummy_transfer();
  //Look for reports from the IMU
  if (imu_initialized)
  {
    if (myIMU.dataAvailable() == true)
    {
      byte linAccuracy = 0;
      if (myIMU.getOnNewLinAccel(ax, ay, az, linAccuracy))
      {
        new_imu_reports |= IMU_MASK_ACC;
      }
      byte gyroAccuracy = 0;
      if (myIMU.getOnNewGyro(gx, gy, gz, gyroAccuracy))
      {
        new_imu_reports |= IMU_MASK_GYRO;
      }

      float quatRadianAccuracy = 0;
      byte quatAccuracy = 0;
      if (myIMU.getOnNewQuat(qx, qy, qz, qw, quatRadianAccuracy, quatAccuracy))
      {
        new_imu_reports |= IMU_MASK_QUAT;
      }

      // only publish when all imu data were acquired new
      if ((new_imu_reports & IMU_MASK_ALLNEWDATA) == IMU_MASK_ALLNEWDATA)
      {
        // prepare the IMU data in a buffer
        pack_all_imudata(imu_buf);
        // erase new flag
        new_imu_reports = 0;
        // pack and send the IMU data in a 3rd datagram
        SP.packData((void *)imu_buf, SP_IMU_DATA_LEN, 2); // was 3
        SP.publish();
      }
      else
      {
        //SP.text_error("IMU data there but incomplete");
        if(new_imu_reports != 0x0)
        {
          //SP.textError(" available data is : ");
          /*if ((new_imu_reports & IMU_MASK_ACC) == IMU_MASK_ACC)
            SP.text_error("ACC ");
          if ((new_imu_reports & IMU_MASK_GYRO) == IMU_MASK_GYRO)
            SP.text_error("GYR ");
          if ((new_imu_reports & IMU_MASK_QUAT) == IMU_MASK_QUAT)
            SP.text_error("QUAT");
            */
        }


      }
    }
  }
  else
  {
    imu_missing_count++;
    if (imu_missing_count > 1000)
    {
      SP.textError("IMU not initialized");
      imu_missing_count=0;
    }
  }
}


uint16_t spi_transfer(unsigned int nr, unsigned short data){
  // initialize SPI Bus for tactile reading
  SPI.beginTransaction(settingsADC);
  //select new ADC chip
  digitalWrite (ADC_CS[nr], LOW);
  uint16_t result = SPI.transfer16(data);
  digitalWrite (ADC_CS[nr], HIGH);
  SPI.endTransaction();
  return result;
}

uint16_t spi_dummy_transfer(){
  unsigned short data;
  // initialize SPI Bus for resetting the correct mode for IMU reading
  SPI.beginTransaction(settingsB);
  uint16_t result = SPI.transfer16(data);
  SPI.endTransaction();
  return result;
}

void pack_adc_buffer(char *buf, uint8_t id, uint16_t data){
  memcpy(buf, (char*)&id, sizeof(uint8_t));
  // separate to two bytes
  // little endian
  byte c_data = byte(data & 0x00FF);
  memcpy(buf+1, (char*)&c_data, sizeof(char));
  // do not discard the channel number, it can be useful for debugging at PC side
  c_data = byte((data & 0xFF00) >> 8);
  memcpy(buf+2, (char*)&c_data, sizeof(char));
}

void pack_all_imudata(char *buf){
  memcpy(buf, (char*)&ax, sizeof(float));
  memcpy(buf+4, (char*)&ay, sizeof(float));
  memcpy(buf+8, (char*)&az, sizeof(float));
  memcpy(buf+12, (char*)&gx, sizeof(float));
  memcpy(buf+16, (char*)&gy, sizeof(float));
  memcpy(buf+20, (char*)&gz, sizeof(float));
  memcpy(buf+24, (char*)&qw, sizeof(float));
  memcpy(buf+28, (char*)&qx, sizeof(float));
  memcpy(buf+32, (char*)&qy, sizeof(float));
  memcpy(buf+36, (char*)&qz, sizeof(float));
}
