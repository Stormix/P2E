#include <Cmd.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
union {
  float val;
  byte arr[4];
} x;
union {
  float val;
  byte arr[4];
} y;
union {
  float val;
  byte arr[4];
} z;
union {
  float val;
  byte arr[4];
} w;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

int buttonPins[4] = {4,5,6,7};
bool buttons[8] = {false, false, false, false, false, false, false, false}; // contains the 5 buttons status
bool buttonActive[8] = {false, false, false, false, false, false, false, false}; // contains the 5 buttons status
bool longPressActive[8] = {false, false, false, false, false, false, false, false}; // contains the 5 buttons status
long buttonTimer[4] = {0,0,0,0};
long longPressTime = 250;

float voltage = 9.9;
bool start = false;

void setup()
{
  randomSeed(millis());

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // init the command line and set it for a speed of 115200
  Serial.begin(115200);
  cmdInit(&Serial);

  //Note: Commands are case sensitive
  cmdAdd("START", cmd_start);
  cmdAdd("STOP", cmd_stop);
  cmdAdd("VERSION?", cmd_version);
  cmdAdd("SERIAL?", cmd_serial);

  // initialize device
  // Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  //Serial.println(F("Testing device connections..."));
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    //Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // Setup buttons
  for (int i = 0; i < sizeof(buttonPins); i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  cmdPoll();
  if (start) {

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
      if (mpuInterrupt && fifoCount < packetSize) {
        // try to get out of the infinite loop
        fifoCount = mpu.getFIFOCount();
      }
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      fifoCount = mpu.getFIFOCount();
      //Serial.println(F("FIFO overflow!"));

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;


      // display quaternion values in easy matrix form: w x y z
      mpu.dmpGetQuaternion(&q, fifoBuffer);
    }
    communicateData(q);
  }
}

void hello(int arg_cnt, char **args)
{
  Serial.println("Hello world.");
}
//called for START command
void cmd_start(int arg_cnt, char **args)
{
  start = true;
}

//called for STOP command
void cmd_stop(int arg_cnt, char **args)
{
  start = false;
}

//called for VERSION? command
void cmd_version(int arg_cnt, char **args)
{
  cmdGetStream()->println("V0.1");
}

//called for SERIAL? command
void cmd_serial(int arg_cnt, char **args)
{
  cmdGetStream()->println("P2E_38");
}

byte BoolArrayToByte(bool boolArray[8])
{
  byte result = 0;

  for (int i = 0; i < 8; i++)
  {
    if (boolArray[i])
    {
      result = result | (1 << i);
    }
  }

  return result;
}

void communicateData(Quaternion q) {
  byte mode = 85;
  byte sep = 0;
  for (int i = 0; i < sizeof(buttonPins); i++) {
    buttons[i] = !digitalRead(buttonPins[i]);
  }
  for (int i = 1; i < 3; i++) {
    if(buttons[i]){
      // button is pressed
      if (buttonActive[i] == false) {
          buttonActive[i] = true;
          buttonTimer[i] = millis();
        }
        if ((millis() - buttonTimer[i] > longPressTime) && (longPressActive[i] == false)) {
          longPressActive[i] = true;
        }
    }else{
      if (buttonActive[i] && !longPressActive[i]) {
        buttonActive[i] = false;
      }
      if (longPressActive[i] == true) {
        longPressActive[i] = false;
      }
    }
  }
  byte b = BoolArrayToByte(buttons);
  //Serial.println(b,BIN);
  //Serial.println("-------------");
  byte v = 69;//(int) voltage * 10;
  // Quaternion data
  x.val = q.x;
  y.val = q.y;
  z.val = q.z;
  w.val = q.w;
  Serial.write(&mode, 1);
  Serial.write(&b, 1);
  Serial.write(x.arr, 4);
  Serial.write(y.arr, 4);
  Serial.write(z.arr, 4);
  Serial.write(w.arr, 4);
  Serial.write(&v, 1);
  Serial.write(&mode, 1);
}
