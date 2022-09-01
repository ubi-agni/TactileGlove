#define REV 2
//firmware version update of CEArl from Teensy3.2 to Teensy4.0
// using now 2 SPI ports/modules
// used in P6Leander and P5 pair

// selection the correct pair using a define
#define P5pair
//#define P6Leander

// select if right or left glove
//#define IS_LEFT

#include <SPI.h>  // include the new SPI library:
#include <Wire.h>
// install the library from the svn https://svn.techfak.uni-bielefeld.de/citec/projects/AGNIserialProtocol/src/teensy/SerialProtocol
#include "SerialProtocol.h"

// fitted peripherals
#ifdef P5pair
// comment out the 2 following lines if you want to use the ADC instead of the sim
//  #define NO_ADC
//  #define SIM_ADC
  #define ADC_AD7490B
  #define ADC_SPI1
//  #define BEND_SENSORS off as no Bendsensors are integrated (but some channels might be free)
//  #define HAVE_IMU
//  #define IMU_SPI0
#endif

#ifdef P6Leander
  //#define NO_ADC
  //#define SIM_ADC
  #define ADC_SPI1
  #define ADC_MAX11131
  #define BEND_SENSORS
  #define HAVE_IMU
  #define IMU_SPI0
#endif


#ifdef HAVE_IMU
// use the latest library upstream
#include <SparkFun_BNO080_Arduino_Library.h> //Change( SparkFun_BNO080_Arduino_Library.h  || (firmware version 11.03.2020 needed BNO08x_Library.h (robomantic branch))
BNO080 myIMU;
#endif

#ifdef BEND_SENSORS
  #define NUM_BEND_SENSORS 7 // define here the number of channels used for bend sensors
  // define here the list of taxel numbers used for bend sensors. taxel numer = (channel_number+16*ADC_number) starting counting from 0
  char bend_taxels_ids[NUM_BEND_SENSORS]={55,56,57,58,59,60,62};
  #define BEND_SENSOR_ADC_ID 4 // ? clean if there is no ADC
#else
  #define NUM_BEND_SENSORS  0
#endif

#define NUM_TAXELS 64 //80
// NUM_ADC * NUM_CHANNEL should match NUM_TAXELS + NUM_BEND_SENSORS
#define NUM_ADC 4
#define NUM_CHANNEL 16

// Serial Protocol setting
#define SP_TACTDATA_LEN  (0x03*NUM_TAXELS) //  0xC0  // 192 = 64x (uint8 + uint16)
#ifdef BEND_SENSORS
  #define SP_BENDDATA_LEN  (0x03*NUM_BEND_SENSORS)  // 3 = 1x (uint8 + uint16)
#endif
#ifdef HAVE_IMU
  #define SP_IMU_DATA_LEN  0x28  // 40 = 3x4 + 3x4 + 4x4 = 40
#endif

// imu settings
#ifdef HAVE_IMU
  #define IMU_INTERNAL_REFRESH_PERIOD  6 // in ms = 142 Hz
  #define IMU_REFRESH_PERIOD  10 // in ms 50 Hz

  /*max IMU speed: Gyro rotation Vector 1 kHz, Rotation Vector  400 Hz, Gaming Rotation Vector 400 Hz, Geomagnetic Rotation Vector 90 Hz, Gravity 400 Hz, Linear Acceleration 400 Hz, Accelerometer 500 Hz, Gyroscope 400 Hz, Magnetometer 100 Hz
  */
  #define IMU_MASK_ALLNEWDATA 0x07
  #define IMU_MASK_ACC        0x01
  #define IMU_MASK_GYRO       0x02
  #define IMU_MASK_QUAT       0x04
#endif

#ifdef ADC_AD7490B
  #define ADC_CONFIG_WRITE_REG 0x0831<<4
#endif

#define ADC_REFRESH_PERIOD  1 // in ms

// prepare the serial number
#ifdef IS_LEFT
String side_string = String("L_");
#else
String side_string = String("R_");
#endif
String serial_string = side_string + String("TG_T4_")
#ifndef NO_ADC
+  String(NUM_ADC,DEC)
 #ifdef ADC_AD7490B
 + String("ADC-AD74_")
 #endif
 #ifdef ADC_MAX11131
 + String("ADC-MAX_")
 #endif
#else
 #ifdef SIM_ADC
 + String(NUM_ADC,DEC)
 + String("ADC-SIM_")
 #endif
#endif
#ifdef BEND_SENSORS
+ String(NUM_BEND_SENSORS,DEC) + String("BEND_")
#endif
#ifdef HAVE_IMU
+ String("IMU_")
#endif
+ String(ARDUINO,DEC) + String("_") + String(REV,DEC) + String("_") + String(__DATE__).replace(" ", "-") + String("_glove_generic_T4.ino");

#ifdef P5pair
  // pin config IMU
  const byte imuWAKPin = 23;  //PS0
  const byte imuINTPin = 17;  //INT
  const byte imuRSTPin = 22;  //RST
  const byte imuCSPin = 36;
  // CS peripheral config
  const byte peripheralA0Pin = 10; //Device A: ADC MAX11131A    or older ADC: AD7490 The order may be changed !?!!
  const byte peripheralA1Pin = 9;  //The order may be changed !?!!
  const byte peripheralA2Pin = 20; //The order may be changed !?!!
  const byte peripheralA3Pin = 21; //The order may be changed !?!!
  // const byte peripheralA4Pin = 0; //(bens-sensor and Calib-Tool ADC )
  byte ADC_CS[NUM_ADC] = {peripheralA0Pin,peripheralA1Pin,peripheralA2Pin,peripheralA3Pin}; //should match number of ADC's // excluded peripheralA4Pin becaus no Bend sensors and no 5th ADC
#endif
#ifdef P6Leander
  // pin config IMU P6Leander
  const byte imuWAKPin = 23;  //PS0
  const byte imuINTPin = 17;  //INT
  const byte imuRSTPin = 22;  //RST
  const byte imuCSPin = 36;
  // CS peripheral config
  const byte peripheralA0Pin = 10; //Device A: ADC MAX11131A    or older ADC: AD7490 The order may be changed !?!!
  const byte peripheralA1Pin = 9;  //The order may be changed !?!!
  const byte peripheralA2Pin = 20; //The order may be changed !?!!
  const byte peripheralA3Pin = 21; //The order may be changed !?!!
  const byte peripheralA4Pin = 0; //(bens-sensor and Calib-Tool ADC )
  byte ADC_CS[NUM_ADC] = {peripheralA0Pin,peripheralA1Pin,peripheralA2Pin,peripheralA3Pin, peripheralA4Pin}; //should match number of ADC's
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

 #ifdef ADC_MAX11131
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
  #endif
#endif

// keep out of the NO_ADC to permit SIM_ADC
uint16_t AnalogData[NUM_TAXELS+NUM_BEND_SENSORS]; // analog sensor data

#ifdef HAVE_IMU
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
int imu_missing_count = 0;
#endif

// initialize status LED if possible
#ifndef IMU_SPI0
  const byte ledPin = 13; //orange led  = 13(Teensy3.2 and Teensy4.0)
  int ledState = LOW;
#endif

// buffer initialization
char tactile_buf[SP_DATA_OFFSET+SP_TIMESTAMP_LEN+SP_TACTDATA_LEN+SP_CHKSUM_LEN];
#ifdef BEND_SENSORS
  char bend_buf[SP_DATA_OFFSET+SP_TIMESTAMP_LEN+SP_BENDDATA_LEN+SP_CHKSUM_LEN];
#endif
#ifdef HAVE_IMU
char imu_buf[SP_DATA_OFFSET+SP_TIMESTAMP_LEN+SP_IMU_DATA_LEN+SP_CHKSUM_LEN];
// internal copies of the IMU data to sync them and send when all are new
float ax, ay, az, gx, gy, gz, qx, qy, qz, qw; // (qx, qy, qz, qw = to i,j,k, real)
// indicators of new data availability
volatile byte newQuat = 0;
volatile byte newLinAcc = 0;
volatile byte newGyro = 0;
byte new_imu_reports = 0;
#endif


// Timing
const unsigned long message_period = 1000000;
elapsedMicros elapsedtime;
elapsedMicros elapsedTimeSensor2;
elapsedMillis elapsedError;
IntervalTimer messageTimer;
IntervalTimer myTimer;
bool timer_started = false;
bool message_timer_started = false;

// using two incompatible SPI devices, A and B
#ifdef ADC_MAX11131
  SPISettings settingsADC(20000000, MSBFIRST, SPI_MODE3); //MAX11131  //SPI Clock Speeds: 396000000
#endif
#ifdef ADC_AD7490B
  SPISettings settingsADC(20000000, MSBFIRST, SPI_MODE1); //AD7490B  //SPI Clock Speeds:
#endif
#ifdef HAVE_IMU
  SPISettings settingsB(20000000, MSBFIRST, SPI_MODE3); //BNO085 dummy sending
  #ifdef BEND_SENSORS
    #define SP_BEND_ID 2
    #define SP_IMU_ID 3
    #define SP_CONFIG_SIZE 3
  #else
    #define SP_IMU_ID 2
    #define SP_CONFIG_SIZE 2
  #endif
#else
  #ifdef BEND_SENSORS
    #define SP_BEND_ID 2
    #define SP_CONFIG_SIZE 2
  #else
    #define SP_CONFIG_SIZE 1
  #endif
#endif

SP_SensorConfiguration sensorConfigs[SP_CONFIG_SIZE];

void setup() {
  // Sensor configuration including default pub period (can be changed through the serial protocol)
  sensorConfigs[0] =  SP_makeSensorConfig(SensorTypeIDEnum::TactileSensorResistive, 0x35,  SP_TACTDATA_LEN, ADC_REFRESH_PERIOD * 10);  // x0.1ms  10=1kHz, 50=200Hz, 100=100Hz
#ifdef BEND_SENSORS
   // TODO see if one should have different period for bend sensors, as a multiple of the period of the tactile sensor period
  sensorConfigs[1] =  SP_makeSensorConfig(SensorTypeIDEnum::PositionSensorAngular, 0x35, SP_BENDDATA_LEN, ADC_REFRESH_PERIOD * 10);  // x0.1ms  10=1kHz, 50=200Hz, 100=100Hz // indirectly using the same ADC loop so one single frequency there
  #ifdef HAVE_IMU
    sensorConfigs[2] =  SP_makeSensorConfig(SensorTypeIDEnum::IMU, 0x30, SP_IMU_DATA_LEN, IMU_REFRESH_PERIOD * 10);  // x0.1ms  10=1kHz, 50=200Hz, 100=100Hz // CURRENTLY in poll mode cannot handle faster than 20ms
  #endif
#else
  #ifdef HAVE_IMU
   sensorConfigs[1] =  SP_makeSensorConfig(SensorTypeIDEnum::IMU, 0x30, SP_IMU_DATA_LEN, IMU_REFRESH_PERIOD * 10);  // x0.1ms  10=1kHz, 50=200Hz, 100=100Hz // CURRENTLY in poll mode cannot handle faster than 20ms
  #endif
#endif

  // set the Slave Select Pins as outputs:
#ifndef NO_ADC
  pinMode (peripheralA0Pin, OUTPUT);
  pinMode (peripheralA1Pin, OUTPUT);
  pinMode (peripheralA2Pin, OUTPUT);
  pinMode (peripheralA3Pin, OUTPUT);
  #ifdef BEND_SENSORS
    pinMode (peripheralA4Pin, OUTPUT);
  #endif
#endif

#ifdef HAVE_IMU
  pinMode (imuCSPin, OUTPUT);
#endif

  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  Serial.setTimeout(500); //timeout of 500 ms
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

  Serial.println(serial_string);
  // store the configuration for the SerialProtocol
  SP.setConfigFromStruct(DeviceTypeIDEnum::TactileGlove, SP_CONFIG_SIZE, sensorConfigs);
  // store the serial num
  SP.setSerialNumber(serial_string.c_str(), serial_string.length());

  // set up the SPI pins utilized on Teensy 4.0
#ifdef HAVE_IMU
//myIMU.enableDebugging(Serial); //Pipe debug messages to Serial port

  Serial.println("setting up IMU on SPI");
  imuSPI.setMOSI(imuCOPIPin);
  imuSPI.setMISO(imuCIPOPin);
  imuSPI.setSCK(imuSCKPin);
  // initialize SPI:
  imuSPI.begin();
#endif

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

#ifdef HAVE_IMU
  // needed to get the clock idle HIGH before talking to the BNO085
  spi_dummy_transfer();
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
#endif

#ifndef NO_ADC
   // initialize ADCs (copied from old MCU)
  unsigned char adc_nr = 0;
  #ifdef ADC_AD7490B
    while(adc_nr < NUM_ADC){
      spi_transfer(adc_nr, 0xFFFF);
      spi_transfer(adc_nr, (ADC_CONFIG_WRITE_REG | (0<<10) ));
      adc_nr++;
    }
  #endif
#endif

  interrupts();
  // no code below this line in the setup
}

uint8_t stat, val1, val2, result;
long counter = 0;

#ifdef HAVE_IMU
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
    SP.packData((void *)imu_buf, SP_IMU_ID);
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
#endif

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
#ifdef HAVE_IMU
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
#endif
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
#ifdef HAVE_IMU
  if (!imu_initialized)
  {
    Serial.println("  IMU did not start properly" );
  }
#endif
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

#if not defined(NO_ADC) || defined(SIM_ADC)
  //read all Max11131 ADC chips/channels
  byte taxel_counter = 0;
  byte bend_sensor_counter = 0;

  for (byte channel=0; channel < NUM_CHANNEL; channel++)
  {
    for (byte SelectADC=0; SelectADC < NUM_ADC; SelectADC++)
    {
  #ifndef SIM_ADC
      // initialize SPI Bus for tactile reading, in cas
      adcSPI.beginTransaction(settingsADC);
        //select new ADC chip
      digitalWrite (ADC_CS[SelectADC], LOW);
      #ifdef ADC_AD7490B
      AnalogData[taxel_counter + bend_sensor_counter] = adcSPI.transfer16(ADC_CONFIG_WRITE_REG | (channel<<10));
      #endif
      #ifdef ADC_MAX11131
      AnalogData[taxel_counter + bend_sensor_counter] = adcSPI.transfer16(SPI_channelselect[channel]);
      // for some reason the bit order of the answer is starting with the data (not the channel) and the last 3 bits are always zero
      // shifting should solve for now
      AnalogData[taxel_counter + bend_sensor_counter] = AnalogData[taxel_counter + bend_sensor_counter] >> 3;
      #endif
      // deselect the ADC
      digitalWrite (ADC_CS[SelectADC], HIGH);
      // free the SPI Bus
      adcSPI.endTransaction();
  #else
      AnalogData[taxel_counter + bend_sensor_counter] = taxel_counter;
  #endif
     // Serial.println(AnalogData[taxel_counter]);
       // separate the bend sensors on a second "serial protocol device"
  #ifdef BEND_SENSORS
      if (SelectADC == BEND_SENSOR_ADC_ID)
      {
        // prepare the bend buffer
        pack_adc_buffer(bend_buf+3*bend_sensor_counter, bend_sensor_counter, AnalogData[taxel_counter + bend_sensor_counter]);
        bend_sensor_counter++;
      }
      else
  #endif
      {
        // prepare the tactile buffer
        pack_adc_buffer(tactile_buf+3*taxel_counter, taxel_counter, AnalogData[taxel_counter + bend_sensor_counter]);
        taxel_counter++;
      }
    }
  }
  // pack and send the data in a first datagram
  int32_t ret = SP.packData((void *)tactile_buf, 1);
  if (ret < 0)
    SP.textError("pack error for tax");
  SP.publish();
 #ifdef BEND_SENSORS
  // pack and send bend data in a second datagram
  ret = SP.packData((void *)bend_buf, SP_BEND_ID);
  if (ret < 0)
    SP.textError("pack error for bend");
  SP.publish();
 #endif
#endif
  interrupts();
}


#ifdef HAVE_IMU
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
#endif

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

#ifdef HAVE_IMU
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
#endif
