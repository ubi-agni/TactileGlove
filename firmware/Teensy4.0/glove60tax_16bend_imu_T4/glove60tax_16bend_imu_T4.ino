#define REV 1

#include <SPI.h>  // include the new SPI library:
//firmware version update of CEArl from Teensy3.2 to Teensy4.0 -> SPI channels changed!
// using now 2 SPI ports/modules
// used in P6Leander
#include <Wire.h>
// use the latest library upstream
#include <SparkFun_BNO080_Arduino_Library.h>
// install the library from the svn https://svn.techfak.uni-bielefeld.de/citec/projects/AGNIserialProtocol/src/teensy/SerialProtocol
#include "SerialProtocol.h"
BNO080 myIMU;

//#define Guillaume1
//#define IMU_SPI1
//#define NO_ADC
#define P6Leander
#define IMU_SPI0
//#define NO_ADC
#define ADC_SPI1

#define NUM_TAXELS 64 //60 used channels

// Serial Protocol setting
#define BEND_SENSOR_ADC_ID 4
#define NUM_BEND_SENSORS 16
#define SP_BENDDATA_LEN  (0x03*NUM_BEND_SENSORS)  // 3 = 1x (uint8 + uint16)
#define SP_TACTDATA_LEN  (0x03*NUM_TAXELS) //  0xC0  // 192 = 64x (uint8 + uint16)
#define SP_IMU_DATA_LEN  0x28  // 40 = 3x4 + 3x4 + 4x4 = 40

#define IMU_INTERNAL_REFRESH_PERIOD  6 // in ms = 142 Hz
#define IMU_REFRESH_PERIOD  10 // in ms 50 Hz

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

// NUM_ADC * NUM_CHANNEL should match NUM_TAXELS + NUM_BEND_SENSORS
#define NUM_ADC 5
#define NUM_CHANNEL 16

// prepare the serial number
String serial_string = String("TG_T4_") + String(NUM_ADC,DEC) + String("_") + String(NUM_BEND_SENSORS,DEC) +
  String("_") + String(ARDUINO,DEC) + String("_") + String(REV,DEC) + String("_") + String(__DATE__).replace(" ", "-") + String("_glove60tax_16bend_imu_T4.ino");
/*
// using two incompatible SPI devices, A and B
const byte peripheralA0Pin = 9; //Device A: ADC MAX11131A    or older ADC: AD7490 The order may be changed !?!!
const byte peripheralA1Pin = 10;  //The order may be changed !?!!
const byte peripheralA2Pin = 20; //The order may be changed !?!!
const byte peripheralA3Pin = 21; //The order may be changed !?!!
const byte peripheralA4Pin = 0; //(bens-sensor and Calib-Tool ADC )
*/
const byte peripheralA0Pin = 10; //Device A: ADC MAX11131A    or older ADC: AD7490 The order may be changed !?!!
const byte peripheralA1Pin = 9;  //The order may be changed !?!!
const byte peripheralA2Pin = 20; //The order may be changed !?!!
const byte peripheralA3Pin = 21; //The order may be changed !?!!
const byte peripheralA4Pin = 0; //(bens-sensor and Calib-Tool ADC )


// pin config IMU SPI1 Guillaume
#ifdef Guillaume1
const byte imuWAKPin = 24;  //PS0
const byte imuINTPin = 25;  //INT
const byte imuRSTPin = 2;  //RST
const byte imuCSPin = 0;
#endif
#ifdef Guillaume2
// pin config IMU SPI2 Guillaume
const byte imuWAKPin = 23;  //PS0
const byte imuINTPin = 17;  //INT
const byte imuRSTPin = 22;  //RST
const byte imuCSPin = 36;
#endif
#ifdef P6Leander
// pin config IMU P6Leander
const byte imuWAKPin = 23;  //PS0
const byte imuINTPin = 17;  //INT
const byte imuRSTPin = 22;  //RST
const byte imuCSPin = 36;
#endif

#ifndef NO_ADC
  #ifdef ADC_SPI0
  const byte adcCOPIPin = 11;
  const byte adcCIPOPin = 12;
  const byte adcSCKPin = 13;
  SPIClass &adcSPI = SPI;
  #endif
  #ifdef ADC_SPI1
  const byte adcCOPIPin = 26;
  const byte adcCIPOPin = 1;
  const byte adcSCKPin = 27;
  SPIClass &adcSPI = SPI1;
  #endif
#endif

#ifdef IMU_SPI0
// SPI0 on Teensy 4.0 uses COPI Pin = 11 CIPO Pin = 12, SCK Pin = 13
const byte imuCOPIPin = 11;
const byte imuCIPOPin = 12;
const byte imuSCKPin = 13;
SPIClass &imuSPI = SPI;
#endif
#ifdef IMU_SPI1
// SPI1 on Teensy 4.0 uses COPI Pin = 26 CIPO Pin = 1, SCK Pin = 27
const byte imuCOPIPin = 26;
const byte imuCIPOPin = 1;
const byte imuSCKPin = 27;
SPIClass &imuSPI = SPI1;
#endif
#ifdef IMU_SPI2
// SPI2 on Teensy 4.0 uses COPI Pin = 35 CIPO Pin = 34, SCK Pin = 37
const byte imuCOPIPin = 35;
const byte imuCIPOPin = 34;
const byte imuSCKPin = 37;
SPIClass &imuSPI = SPI2;
#endif

//Further IMU pins
bool imu_initialized = false;

#ifndef IMU_SPI0
const byte ledPin = 13; //orange led  = 13(Teensy3.2 and Teensy4.0)
int ledState = LOW;
#endif
int imu_missing_count = 0;

elapsedMicros elapsedtime;

// buffer initialization
char tactile_buf[SP_DATA_OFFSET+SP_TIMESTAMP_LEN+SP_TACTDATA_LEN+SP_CHKSUM_LEN];
char bend_buf[SP_DATA_OFFSET+SP_TIMESTAMP_LEN+SP_BENDDATA_LEN+SP_CHKSUM_LEN];
char imu_buf[SP_DATA_OFFSET+SP_TIMESTAMP_LEN+SP_IMU_DATA_LEN+SP_CHKSUM_LEN];

// internal copies of the IMU data to sync them and send when all are new
float ax, ay, az, gx, gy, gz, qx, qy, qz, qw; // (qx, qy, qz, qw = to i,j,k, real)
// indicators of new data availability
volatile byte newQuat = 0;
volatile byte newLinAcc = 0;
volatile byte newGyro = 0;
byte new_imu_reports = 0;

uint16_t AnalogData[NUM_TAXELS+NUM_BEND_SENSORS]; // analog sensor data
byte ADC_CS[NUM_ADC] = {peripheralA0Pin,peripheralA1Pin,peripheralA2Pin,peripheralA3Pin,peripheralA4Pin}; //included peripheralA4Pin
unsigned short SPI_channelselect[NUM_CHANNEL] = {
  0x0800,
  0x0880,
  0x0900,
  0x0980,
  0x0A00,
  0x0A80,
  0x0B00,
  0x0B80,
  0x0C00,
  0x0C80,
  0x0D00,
  0x0D80,
  0x0E00,
  0x0E80,
  0x0F00,
  0x0F80
  };

const unsigned long message_period = 1000000;
elapsedMicros elapsedTimeSensor2;
elapsedMillis elapsedError;

IntervalTimer messageTimer;
IntervalTimer myTimer;

SP_SensorConfiguration sensorConfigs[3];

SPISettings settingsADC(20000000, MSBFIRST, SPI_MODE3); //Max11131  //SPI Clock Speeds: 396000000
//SPISettings settingsB(20000000, MSBFIRST, SPI_MODE3); //BNO085 dummy sending

bool timer_started = false;
bool message_timer_started = false;

void setup() {
  // Sensor configuration including default pub period (can be changed through the serial protocol)
  sensorConfigs[0] =  SP_makeSensorConfig(SensorTypeIDEnum::TactileSensorResistive, 0x35,  SP_TACTDATA_LEN, ADC_REFRESH_PERIOD * 10);  // x0.1ms  10=1kHz, 50=200Hz, 100=100Hz
   // TODO see if one should have different period for bend sensors, as a multiple of the period of the tactile sensor period
  sensorConfigs[1] =  SP_makeSensorConfig(SensorTypeIDEnum::PositionSensorAngular, 0x35, SP_BENDDATA_LEN, ADC_REFRESH_PERIOD * 10);  // x0.1ms  10=1kHz, 50=200Hz, 100=100Hz // indirectly using the same ADC loop so one single frequency there
  sensorConfigs[2] =  SP_makeSensorConfig(SensorTypeIDEnum::IMU, 0x30, SP_IMU_DATA_LEN, IMU_REFRESH_PERIOD * 10);  // x0.1ms  10=1kHz, 50=200Hz, 100=100Hz // CURRENTLY in poll mode cannot handle faster than 20ms

  // set the Slave Select Pins as outputs:
#ifndef NO_ADC
  pinMode (peripheralA0Pin, OUTPUT);
  pinMode (peripheralA1Pin, OUTPUT);
  pinMode (peripheralA2Pin, OUTPUT);
  pinMode (peripheralA3Pin, OUTPUT);
  pinMode (peripheralA4Pin, OUTPUT);
#endif

  pinMode (imuCSPin, OUTPUT);


  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  Serial.setTimeout(500); //timeout of 500 ms
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

  Serial.println(serial_string);
  // store the configuration for the SerialProtocol
  SP.setConfigFromStruct(DeviceTypeIDEnum::TactileGlove, 3, sensorConfigs);

  // set up the SPI pins utilized on Teensy 3.2
  //SPI.setMOSI(11);   //Teensy3.2 MOSI(7)
  //SPI.setMISO(12);   //Teensy3.2 MISO(8)
  //SPI.setSCK(37);   //Teensy3.2 SCK(14)   Teensy4.0 SCK(13) SCK1(27) SCK2(37)
  // initialize SPI:
  //SPI.begin();
  // set up the SPI pins utilized on Teensy 4.0

//myIMU.enableDebugging(Serial); //Pipe debug messages to Serial port

  Serial.println("setting up IMU on SPI");
  imuSPI.setMOSI(imuCOPIPin);
  imuSPI.setMISO(imuCIPOPin);
  imuSPI.setSCK(imuSCKPin);
  // initialize SPI:
  imuSPI.begin();

#ifndef NO_ADC
  Serial.println("setting up ADC on SPI");
  adcSPI.setMOSI(adcCOPIPin);
  adcSPI.setMISO(adcCIPOPin);
  adcSPI.setSCK(adcSCKPin);
  // initialize SPI:
  adcSPI.begin();
#endif

#if !defined IMU_SPI0 && !defined ADC_SPI0
  // activate the integrated LED
  pinMode(ledPin, OUTPUT);
#endif


  spi_dummy_transfer();
   // initialize ADCs (copied from old MCU)
  unsigned char adc_nr = 0;
  /*while(adc_nr < NUM_ADC){
    spi_transfer(adc_nr, 0xFFFF);
    spi_transfer(adc_nr, (ADC_CONFIG_WRITE_REG | (0<<10) ));
    adc_nr++;
  }*/

  Serial.println("initializing IMU");
  //Setup BNO080 to use SPI interface with default SPI port and max BNO080 clk speed of 3MHz
  imu_initialized = myIMU.beginSPI(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin, 3000000, imuSPI);
  Serial.println("IMU config requested");
  // Default internal periodicity (IMU_INTERNAL_REFRESH_PERIOD ms)
  if (imu_initialized)
  {
    myIMU.enableLinearAccelerometer(IMU_INTERNAL_REFRESH_PERIOD); // m/s^2 no gravity
    myIMU.enableRotationVector(IMU_INTERNAL_REFRESH_PERIOD);  // quat
    myIMU.enableGyro(IMU_INTERNAL_REFRESH_PERIOD); // rad/s
    Serial.println("IMU initialized");
  }
  else
    Serial.println("IMU failed to initialize");

  // prepare interrupt on falling edge (= signal of new data available)
  attachInterrupt(digitalPinToInterrupt(imuINTPin), imu_interrupt_handler, FALLING);
  Serial.println("interrupts now set up");
  interrupts();
  // no code below this line in the setup
}

uint8_t stat, val1, val2, result;
long counter = 0;

bool processIMU()
{
  if(newLinAcc) {
    byte linAccuracy = 0;
    myIMU.getLinAccel(ax, ay, az, linAccuracy);
    newLinAcc = 0;  // mark data as read
    new_imu_reports |= IMU_MASK_ACC;
    //Serial.println("acc");

  }
  if(newQuat) {
    float quatRadianAccuracy = 0;
    byte quatAccuracy = 0;
    myIMU.getQuat(qx, qy, qz, qw, quatRadianAccuracy, quatAccuracy);
    newQuat = 0;  // mark data as read
    new_imu_reports |= IMU_MASK_QUAT;
    //Serial.println("qua");
  }
  if(newGyro) {
    byte gyroAccuracy = 0;
    myIMU.getGyro(gx, gy, gz, gyroAccuracy);
    newGyro = 0;  // mark data as read
    new_imu_reports |= IMU_MASK_GYRO;
    //Serial.println("gyr");
  }

  // only publish when all imu data were acquired new
  if ((new_imu_reports & IMU_MASK_ALLNEWDATA) == IMU_MASK_ALLNEWDATA)
  {
    //Serial.println("ALL");
    // prepare the IMU data in a buffer
    pack_all_imudata(imu_buf);
    // erase new flag
    new_imu_reports = 0;
    // pack and send the IMU data in a 3rd datagram
    SP.packData((void *)imu_buf, 3);
    SP.publish();
  }
  else
  {
    //SP.textError("IMU data there but incomplete");
    if(new_imu_reports != 0x0)
    {
      /*SP.textError(" available data is : ");
      if ((new_imu_reports & IMU_MASK_ACC) == IMU_MASK_ACC)
        SP.textError("ACC ");
      if ((new_imu_reports & IMU_MASK_GYRO) == IMU_MASK_GYRO)
        SP.textError("GYR ");
      if ((new_imu_reports & IMU_MASK_QUAT) == IMU_MASK_QUAT)
        SP.textError("QUAT");
       */
    }
    return false;
  }
  return true;
}

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
      // if not yet started, of period changed, start the ADC acquisition timer
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

      // handle IMU
      if (imu_initialized)
      {
        uint16_t ui16_period;
        int8_t i8_result = SP.getPeriod(&ui16_period, 3);
        if (i8_result >= SP_VARIABLE_UNCHANGED) // sensor is active
        {
          // if period of publishing was reached
          //TimeUsed = micros() - ul_timeStartSensor2;
          if(elapsedTimeSensor2 >= ui16_period*100)
          {
            // handle timing
            elapsedTimeSensor2 = 0;
            // process IMU data

            noInterrupts();
            bool ret = processIMU();
            interrupts();
            if (ret!= true)
            {
              // processIMU() produced data but not a full frame
              if (elapsedError > 1000)
              {
                SP.textError("Requested period is too short for the IMU");
                elapsedError = 0;
              }
            }
          }
        }

      }
    }
    else
    {
      // indicate disconnected if serial not open
      if(Serial)
      {
        #if !defined IMU_SPI0 && !defined ADC_SPI0
        digitalWrite(ledPin, LOW);
        #endif
      }
      else
      {
        #if !defined IMU_SPI0 && !defined ADC_SPI0
        digitalWrite(ledPin, HIGH);
        #endif
      }
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
    //delay(1); // shorter than 1 ms makes the publication stall
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
  noInterrupts();
  elapsedtime=0;
  // make the LED blink dim to indicate running
  counter++;
  if (counter > 500/ADC_REFRESH_PERIOD) // ON after 0.5 sec
  {
    if (counter > 1000/ADC_REFRESH_PERIOD) // for 0.5 sec
      counter = 0;
    // dim
    #if !defined IMU_SPI0 && !defined ADC_SPI0
    digitalWrite(ledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
    delayMicroseconds(10);
    digitalWrite(ledPin, LOW);    // turn the LED off by making the voltage LOW
    #endif
  }

#ifndef NO_ADC

  //read all Max11131 ADC chips/channels
  byte taxel_counter = 0;
  byte bend_sensor_counter = 0;
  //byte channel=0;

  for (byte channel=0; channel < NUM_CHANNEL; channel++)
  {
    //byte SelectADC = 2;
    for (byte SelectADC=0; SelectADC < NUM_ADC; SelectADC++)
    {
      // initialize SPI Bus for tactile reading
      adcSPI.beginTransaction(settingsADC);

      //select new ADC chip
      digitalWrite (ADC_CS[SelectADC], LOW);
      AnalogData[taxel_counter + bend_sensor_counter] = adcSPI.transfer16(SPI_channelselect[channel]);
      // for some reason the bit order of the answer is starting with the data (not the channel) and the last 3 bits are always zero
      // shifting should solve for now
      AnalogData[taxel_counter + bend_sensor_counter] = AnalogData[taxel_counter + bend_sensor_counter] >> 3;

      digitalWrite (ADC_CS[SelectADC], HIGH);
     // Serial.println(AnalogData[taxel_counter]);
       // separate the bend sensors on a second "serial protocol device"
      if (SelectADC == BEND_SENSOR_ADC_ID)
      {
        // prepare the bend buffer
        pack_adc_buffer(bend_buf+3*bend_sensor_counter, bend_sensor_counter, AnalogData[taxel_counter + bend_sensor_counter]);
        bend_sensor_counter++;
      }
      else
      {
        // prepare the tactile buffer
        pack_adc_buffer(tactile_buf+3*taxel_counter, taxel_counter, AnalogData[taxel_counter + bend_sensor_counter]);
        taxel_counter++;
      }
      adcSPI.endTransaction();
   }
  }
  // pack and send the data in a first and second datagram
  int32_t ret = SP.packData((void *)tactile_buf, 1);
  if (ret < 0)
    SP.textError("pack error for tax");
  SP.publish();
  ret = SP.packData((void *)bend_buf, 2);
  if (ret < 0)
    SP.textError("pack error for bend");
  SP.publish();
  interrupts();
 #endif
}



void imu_interrupt_handler()
{
  noInterrupts();
  //Serial.println("wasthere");
  //Look for reports from the IMU
  switch (myIMU.getReadings())
  {
    case SENSOR_REPORTID_LINEAR_ACCELERATION: {
      newLinAcc = 1;
      //Serial.println("a0");
    }
    break;
    case SENSOR_REPORTID_GYROSCOPE: {

         newGyro = 1;
      //Serial.println("a1");
    }
    break;

    case SENSOR_REPORTID_ROTATION_VECTOR:
    case SENSOR_REPORTID_GAME_ROTATION_VECTOR: {

        newQuat = 1;
      //Serial.println("a2");
    }
    break;

    case 0:
      // no data
      break;

    default:
      // Unhandled Input Report
      break;
  }
  interrupts();
}

#ifndef NO_ADC
uint16_t spi_transfer(unsigned int nr, unsigned short data){
  // initialize SPI Bus for tactile reading
  adcSPI.beginTransaction(settingsADC);
  //select new ADC chip
  digitalWrite (ADC_CS[nr], LOW);
  uint16_t result = adcSPI.transfer16(data);
  digitalWrite (ADC_CS[nr], HIGH);
  adcSPI.endTransaction();
  return result;
}


uint16_t spi_dummy_transfer(){
  unsigned short data=0;
  // initialize SPI Bus for resetting the correct mode for IMU reading
  adcSPI.beginTransaction(settingsADC);
  uint16_t result = adcSPI.transfer16(data);
  adcSPI.endTransaction();
  return result;
}
#endif

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
