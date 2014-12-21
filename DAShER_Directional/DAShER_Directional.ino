char s1[6];


// 9/26 AK: Removed write to SD because it interferes with
// talking to mpu6050.  Use serial logger with 
//  Arduino TX -> Logger RXI

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates (of the library) should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

// 1/30/2014
// This sketch was modified by Luis Ródenas <luisrodenaslorda@gmail.com> to include magnetometer and gyro readings
// while DMP is working, in case your magnetometer is attached to aux MPU's I2C lines. It should work with
// any magnetometer, but it has only been tested with HMC5883L.

// Original DMP example sketch has been simplified to my needs, check the original one to see more options.

// This sketch does NOT use the HMC5883L library, but it could be used instead of writing to registers directly.

// Credits to GitHub user muzhig for sharing the code to read magnetometer.


/* ============================================
 I2Cdev device library code is placed under the MIT license
 Copyright (c) 2012 Jeff Rowberg
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ===============================================
 */

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif



// **********************  CONFIGURATION   **********************************
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here


// Pins ---ITG/MPU to SD Sheild --- : VCC(5V), SDA(A4), SCL(A5), int(Pin 2), GND(GND)
// Pins From --- HMC5883L to ITG/MPU--- : SCL(4700ohms-->3.3,XCL), SDA(XDA,4700oms), VCC(3.3,SD Sheild),GRD(GRD,SD Sheild)

// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu(0x68); // Most common is 0x68

/*


 This sketch depends on the MPU-6050's INT pin being connected to the Arduino's
 external interrupt #0 pin.
 
 On the Arduino Uno and Mega 2560, this is digital I/O pin 2. On DUE you can 
 use ANY pin. Remember to modify this some lines below.
 * ========================================================================= */





// ================================================================
// ===               HMC5883L              ===

// Variables used
#define HMC5883L_DEFAULT_ADDRESS    0x1E
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAZ_H         0x05
#define HMC5883L_RA_DATAY_H         0x07
int16_t m_raw[3], m_proc[3];     //To store magnetometer readings
/*float HardIronOffset[3] = {50, -99, -84.5};
float MagnMagnitude[3] = {502, 496, 479.5};*/
float HardIronOffset[3] = {49.5,-243.5,-445.5};
float MagnMagnitude[3] = {625.5, 626.5, 571.5};



// ================================================================
// ===               MPU6050              ===


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
int dmpInterrupt_pin=0;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float newypr[3];


// ================================================================
// ===               TEMPERATURE-BASED CONSTANTS             ===


// Temperature-based PMT supply control constants
const int tp1 = 1;
const int tp2 = 0;
const int stype = 0;
const int pwrPin = 9;
const float vRef = 3.3;
// Min and Max Temperatures from SensTech_PMT_P30CW5_iss01.pdf
const float minT = 5.0;
const float maxT = 55.0;
// Record Temperatures
float temp1, temp2;

// ================================================================
// ===               DATA COLLECTION              ===


// To do data recording
unsigned long Ndata; //note: needs to be long
unsigned long seconds, milliSinceLastSecond; 
boolean newdata;  //used for testing, may not be necessary
unsigned long timePerData = 100;  //in long to match other variables
float decAngleAdj = 0.064984; //Not needed, but reminder that declination needs correction
volatile int crCount;
int Scintillator_pin=1; // Check where your INT pin is connected. Check below.
bool blinkState = false;





// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}





// ================================================================
// ===           MAGNETOMETER CONFIGURATION ROUTINE             ===
// ================================================================

void configMagAxis(int chan) {
  mpu.setSlaveAddress(chan, HMC5883L_DEFAULT_ADDRESS | 0x80); // 0x80 turns 7th bit ON, according to datasheet, 7th bit controls Read/Write direction
  mpu.setSlaveEnabled(chan, true);
  mpu.setSlaveWordByteSwap(chan, false);
  mpu.setSlaveWriteMode(chan, false);
  mpu.setSlaveWordGroupOffset(chan, false);
  mpu.setSlaveDataLength(chan, 2);
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  
  // Flag for errors in setup
  int errorFlag = 0;

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2;
  //Serial.println(TWBR);
  // **************************************************************
  // It is best to configure I2C to 400 kHz. 
  // If you are using an Arduino DUE, modify the variable TWI_CLOCK to 400000, defined in the file:
  // c:/Program Files/Arduino/hardware/arduino/sam/libraries/Wire/Wire.h
  // If you are using any other Arduino instead of the DUE, uncomment the following line:
 //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)  //This line should be commented if you are using Arduino DUE
  // **************************************************************
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  Serial.begin(38400);  //115200 is too fast for Arduino Mini Pro, 38400 is max it can handle

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer

  // initialize device
  mpu.initialize();

  // verify connection
  if (!mpu.testConnection()) { errorFlag = 1;}
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // If you don't know yours, you can find an automated sketch for this task from: http://www.i2cdevlib.com/forums/topic/96-arduino-sketch-to-automatically-calculate-mpu6050-offsets/

  //scaled for MPU-6050 provided by Lou using offset detection software found online
  //AK 29May2014

  /*mpu.setXAccelOffset(-4061);  
  mpu.setYAccelOffset(-867);   
  mpu.setZAccelOffset(1288);    

  mpu.setXGyroOffset(106);
  mpu.setYGyroOffset(-32);
  mpu.setZGyroOffset(20);*/
  
  //scaled for GY521 with pins made to set it flat using offset detection software found online
  //AK 31Oct2014

  mpu.setXAccelOffset(-1850);  
  mpu.setYAccelOffset(-153);   
  mpu.setZAccelOffset(1387);    

  mpu.setXGyroOffset(65);
  mpu.setYGyroOffset(-65);
  mpu.setZGyroOffset(-8);


  // In case you want to change MPU sensors' measurements ranges, you should implement it here. This has not been tested with DMP.

  // Magnetometer configuration

  mpu.setI2CMasterModeEnabled(0);
  mpu.setI2CBypassEnabled(1);

  Wire.beginTransmission(HMC5883L_DEFAULT_ADDRESS);
  Wire.write(0x02); 
  Wire.write(0x00);  // Set continuous mode
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(HMC5883L_DEFAULT_ADDRESS);
  Wire.write(0x00);
  Wire.write(B00011000);  // 75Hz
  Wire.endTransmission();
  delay(5);

  mpu.setI2CBypassEnabled(0);

  mpu.setSlaveRegister(0, HMC5883L_RA_DATAX_H);
  mpu.setSlaveRegister(1, HMC5883L_RA_DATAY_H);
  mpu.setSlaveRegister(2, HMC5883L_RA_DATAZ_H);
  configMagAxis(0);
  configMagAxis(1);
  configMagAxis(2);
  
  mpu.setI2CMasterModeEnabled(1);


  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(dmpInterrupt_pin, dmpDataReady, FALLING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else {
    // ERROR! if:
    // devstatus=1 => initial memory load failed
    // devstatus=2 => DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    errorFlag = 2;
  }

  // configure pin to simulate scintillator detections (testing only!)
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
  digitalWrite(6, LOW);

  
  // Attach interrupt for scintillator detection
  attachInterrupt(Scintillator_pin, increment, FALLING);
  
  // Attach temperature sensor
  pinMode( pwrPin, OUTPUT );

  
  //Initialize counting for data collection
  Ndata = 0;
  crCount=0;
  
  
  if (errorFlag != 0) {
    Serial.print("Error ");
    Serial.print(errorFlag);
  } else {
     //Serial.print("Setup complete.");
  }
}















// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  
  temp1 = readTemp( tp1, stype );
  temp2 = readTemp( tp2, stype );
  digitalWrite( pwrPin, temp1 > minT && temp1 < maxT && 
                            temp2 > minT && temp2 < maxT );
  
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
   }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    Serial.println(F("FIFO overflow!"));
    mpu.resetFIFO();
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  
  
  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }

    // Keep reading out buffer until newest data is retreived
    while (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
      newdata = true;  //flag that new data is available.
    }

    if (millis() > Ndata*timePerData && newdata) { 
      
      calcDirection();
      //printSerial();
      Serial.print("x ");
      float MagnMag = 500;
     Serial.print(MagnMag * cos(newypr[0]) * cos(newypr[1]));
    Serial.print(" ");
    Serial.print(-MagnMag * sin(newypr[0]) * cos(newypr[1]));
    Serial.print(" ");
    Serial.println(MagnMag * sin(newypr[1]));
      /*Serial.print("x ");
      Serial.print(m_raw[0]);
      Serial.print(" ");
      Serial.print(m_raw[1]);
      Serial.print(" ");
      Serial.println(m_raw[2]);*/
      newdata = false;    // flag current data as old.
      blinkState = !blinkState;
      digitalWrite(7, blinkState);
      Ndata=Ndata+1;
      
    }
    
  }

}











// ================================================================
// ===                    SUB-ROUTINES                       ===
// ================================================================







// Read HMC5883L and MPU6050 data and calculate pitch, roll, heading

void calcDirection() {
              //Read mpu6050 and calculate heading
              mpu.dmpGetQuaternion(&q, fifoBuffer);
              mpu.dmpGetGravity(&gravity, &q);
              mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

              //Read magnetometer
              m_raw[0]=mpu.getExternalSensorWord(0);
              m_raw[1]=mpu.getExternalSensorWord(2);
              m_raw[2]=mpu.getExternalSensorWord(4);
              m_proc[0] = (m_raw[0] - HardIronOffset[0])/MagnMagnitude[0]*500.;
              m_proc[1] = (m_raw[1] - HardIronOffset[1])/MagnMagnitude[1]*500.;
              m_proc[2] = (m_raw[2] - HardIronOffset[2])/MagnMagnitude[2]*500.;
           
              newypr[2] = atan2(-gravity.y, -gravity.z);
              float roll = newypr[2];
              newypr[1] = atan2(gravity.x , -gravity.y * sin(roll) - gravity.z * cos(roll));
              float pitch = newypr[1];
             // float XhScaled = m_proc[0] * cos(pitch) + m_proc[1] * sin(pitch) * sin(roll) + m_proc[2] * sin(pitch) * cos(roll);
              float XhScaled = m_proc[1] * cos(pitch) - m_proc[0] * sin(pitch) * sin(roll) + m_proc[2] * sin(pitch) * cos(roll);
             // float YhScaled = m_proc[1] * cos(roll) - m_proc[2] * sin(roll);
              float YhScaled = -m_proc[0] * cos(roll) - m_proc[2] * sin(roll);
              newypr[0] = atan2(YhScaled, XhScaled)+decAngleAdj;
                 //Optional displays for scatter3d_dataDisplay
                 //displayData(gravity.x,gravity.y,gravity.z);
                 //displayData(0,pitch*180./M_PI, roll*180./M_PI);
                 //displayData(0,XhScaled,YhScaled);
                //displayData(m_proc[0],m_proc[1],m_proc[2]);
             // float MagnMag = 500;
             // displayData(MagnMag * cos(newypr[0]) * cos(newypr[1]), MagnMag * sin(newypr[0]) * cos(newypr[1]), MagnMag * sin(newypr[1]));
}
  
  



void printSerial() {
    Serial.print(millis());
    Serial.print("\t");
    /*Serial.print(crCount);
    crCount=0;
    Serial.print("\t");*/
    Serial.print(newypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(newypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.print(newypr[2] * 180/M_PI);
    Serial.print("\t");
    Serial.print(m_raw[0]);
    Serial.print("\t");
    Serial.print(m_raw[1]);
    Serial.print("\t");
    Serial.print(m_raw[2]);
    Serial.print("\t");
    Serial.print(gravity.x,4);
    Serial.print("\t");
    Serial.print(gravity.y,4);
    Serial.print("\t");
    Serial.print(gravity.z,4);
    /*Serial.print("\t");
    Serial.print(temp1);
    Serial.print("\t");
    Serial.print(temp2);*/
    Serial.println();

}



void increment()
{
      crCount++;      
}


float readTemp( int pin, int sensType ) {
  // Temperature Sensor constants:
  //   0  LM60
  //   1  MAX6605
  //   2  TMP36
  int mVoltsAtRefTemp[] = { 424, 744, 750 };
  int refTempC[] = { 0, 0, 25 };
  float mVperDegC[] = { 6.25, 11.9, 10.0 };

  int reading = analogRead(pin);
  float mVolts = reading * vRef / 1.024;

  return( ( mVolts - mVoltsAtRefTemp[sensType] ) / 
            ( mVperDegC[sensType] ) + 
            refTempC[sensType]);
}



