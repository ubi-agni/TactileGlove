#include <SPI.h>  // include the new SPI library:
#include <Wire.h>
#include "BNO08x_Library.h"
#include "SerialProtocol.h"
BNO080 myIMU;

// Serial Protocol setting
#define SP_TACTDATA_LEN  0xC0  // 192 = 64x (uint8 + uint16)
#define SP_BENDDATA_LEN  0x30  // 48 = 16x (uint8 + uint16)
#define SP_IMU_DATA_LEN  0x28  // 40 = 3x4 + 3x4 + 4x4 = 40

char sp_configuration[14] = {0x28, //device type
                              0x03, //3 sub-devices
                              0x35, 0x02,  // sub-device 1 type (tactile resistive)
                              0xC0, 0x00,  // sub-device 1 data length  64 taxels
                              0x35, 0xD2,  // sub-device 2 type (bend sensor = position angular
                              0x30, 0x00,  // sub-device 2 data length 16 bend sensors
                              0x30, 0xDC,  // sub-device 3 type (IMU BNO08x)
                              0x28, 0x00}; // sub-device 3 data length 

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

#define ADC_REFRESH_PERIOD  1 // in ms

#define NUM_TAXELS 80
#define NUM_ADC 5
#define BEND_SENSOR_ADC_ID 4
#define NUM_CHANNEL 16

// using two incompatible SPI devices, A and B
const byte slaveA0Pin = 10; //Device A: ADC: Max11131A
const byte slaveA1Pin = 9;
const byte slaveA2Pin = 20;
const byte slaveA3Pin = 21;
const byte slaveBPin = 15; //Device B: Flex sensors
const byte slaveCPin = 16;  //Device C: IMU: BNO085
//Further IMU pins
const byte imuWAKPin = 23;
const byte imuINTPin = 17;
const byte imuRSTPin = 22;

const byte ledPin = 13; //orange led (Teensy3.2)
int ledState = LOW;

elapsedMicros elapsedtime;

// buffer initialization
char tactile_buf[SP_DATA_OFFSET+SP_TIMESTAMP_LEN+SP_TACTDATA_LEN+1];
char bend_buf[SP_DATA_OFFSET+SP_TIMESTAMP_LEN+SP_BENDDATA_LEN+1];
char imu_buf[SP_DATA_OFFSET+SP_TIMESTAMP_LEN+SP_IMU_DATA_LEN+1];

// internal copies of the IMU data to sync them and send when all are new
float ax, ay, az, gx, gy, gz, qx, qy, qz, qw; // (qx, qy, qz, qw = to i,j,k, real)
byte new_imu_reports = 0;

uint16_t AnalogData[NUM_TAXELS]; // analog sensor data
byte ADC_CS[NUM_ADC] = {10,9,20,21,15}; //15 added
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

IntervalTimer myTimer;

SPISettings settingsADC(48000000, MSBFIRST, SPI_MODE3); //Max11131  //SPI Clock Speeds:
//SPISettings settingsB(115200, LSBFIRST, SPI_MODE3); //BNO085

bool timer_started = false;

void setup() {
  // set the Slave Select Pins as outputs:
  pinMode (slaveA0Pin, OUTPUT);
  pinMode (slaveA1Pin, OUTPUT);
  pinMode (slaveA2Pin, OUTPUT);
  pinMode (slaveA3Pin, OUTPUT);
  pinMode (slaveBPin, OUTPUT);  
  pinMode (slaveCPin, OUTPUT);

  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  Serial.setTimeout(500); //timeout of 500 ms
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

  // store the configuration for the SerialProtocol 
  SP.set_conf(sp_configuration, sizeof(sp_configuration));
  // default pub period (can be changed through the serial protocol)
  SP.set_period(1, ADC_REFRESH_PERIOD * 10); // x0.1ms  10=1kHz, 50=200Hz, 100=100Hz
  // TODO see if one should have different period for bend sensors, as a multiple of the period of the tactile sensor period
  //SP.set_period(2, 100); // x0.1ms  10=1kHz, 50=200Hz, 100=100Hz // indirectly using the same ADC loop so one single frequency there
  SP.set_period(3, IMU_REFRESH_PERIOD * 10); // x0.1ms  10=1kHz, 50=200Hz, 100=100Hz
    
  // initialize SPI:
  SPI.begin(); 
  
  // set up the SPI pins utilized on Teensy 3.2
  SPI.setMOSI(7);
  SPI.setMISO(8);
  SPI.setSCK(14);

  // activate the integrated LED
  pinMode(ledPin, OUTPUT);
 
  //Setup BNO080 to use SPI interface with default SPI port and max BNO080 clk speed of 3MHz
  myIMU.beginSPI(slaveCPin, imuWAKPin, imuINTPin, imuRSTPin);
  // Default periodicity (IMU_REFRESH_PERIOD ms)
  myIMU.enableLinearAccelerometer(IMU_REFRESH_PERIOD); // m/s^2 no gravity
  myIMU.enableRotationVector(IMU_REFRESH_PERIOD);  // quat
  myIMU.enableGyro(IMU_REFRESH_PERIOD); // rad/s
  interrupts();
  // no code below this line in the setup
}

uint8_t stat, val1, val2, result;
long counter = 0;

void loop() {
  if (SP.is_streaming())
  {
     if (!timer_started)
     {
        int pub_period = SP.get_period(1);
        if (pub_period != SP_PERIOD_UNKNOWN_ID && pub_period != SP_PERIOD_INACTIVE)
        {
          // 1 kHz timer start
          timer_started = true;
          myTimer.begin(read_tactile, pub_period * 100.0); // period is given in 100 us timer wants us
        }
     }
  }
  else
  {
    // indicate disconnected if serial not open
    if(Serial)
      digitalWrite(ledPin, LOW);
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
  SP.update();   

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
  //byte channel=0;
  for (byte channel=0; channel < NUM_CHANNEL; channel++)
  {
    //byte SelectADC = 2;
    for (byte SelectADC=0; SelectADC < NUM_ADC; SelectADC++)
    {
      // initialize SPI Bus for tactile reading
      SPI.beginTransaction(settingsADC);
  
      //select new ADC chip  
      digitalWrite (ADC_CS[SelectADC], LOW);
      AnalogData[taxel_counter] = SPI.transfer16(SPI_channelselect[channel]);
      digitalWrite (ADC_CS[SelectADC], HIGH);
     // Serial.println(AnalogData[taxel_counter]);
      SPI.endTransaction();

      // separate the bend sensors on a second "serial protocol device"
      if (SelectADC == BEND_SENSOR_ADC_ID)
      {
        // prepare the bend buffer
        pack_adc_buffer(bend_buf+3*bend_sensor_counter, bend_sensor_counter, AnalogData[taxel_counter]);
        bend_sensor_counter++;
      }
      else
      {
        // prepare the tactile buffer
        pack_adc_buffer(tactile_buf+3*taxel_counter, taxel_counter, AnalogData[taxel_counter]);
      }
      taxel_counter++;
    }
  }
  // pack and send the data in a first and second datagram
  SP.pack_data((void *)tactile_buf, SP_TACTDATA_LEN, 1);
  SP.publish();
  SP.pack_data((void *)bend_buf, SP_BENDDATA_LEN, 2);
  SP.publish();

  //Look for reports from the IMU
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
      SP.pack_data((void *)imu_buf, SP_IMU_DATA_LEN, 3);  
      SP.publish();
    }
  }
}

void pack_adc_buffer(char *buf, uint8_t id, uint16_t data){
  memcpy(buf, (char*)&id, sizeof(uint8_t));
  // separate to two bytes
  // little endian
  byte c_data = byte(data & 0x00FF);
  memcpy(buf+1, (char*)&c_data, sizeof(char));
  c_data = byte((data & 0x0F00) >> 8);
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



