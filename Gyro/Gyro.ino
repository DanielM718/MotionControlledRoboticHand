//#include <MPU6050.h>
#include "BluetoothSerial.h"

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
// class default I2C address is 0x68
MPU6050 mpu(0x68);
MPU6050 mpu2(0x69);

#define OUTPUT_READABLE_EULER

#define OUTPUT_READABLE_YAWPITCHROLL


#define INTERRUPT_PIN 19  // use pin 2 on Arduino Uno & most boards

#define LED_PIN 13 

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t mpuIntStatus2;
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t devStatus2;
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float BTPackage[12]      // [yaw, pitch, roll, psi, theta, phi]-repeat  data sent over BT

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


BluetoothSerial SerialBt;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // initialize serial communication
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

   // Pause for 2 seconds
  // Clear the buffer

  delay(5000);
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  pinMode(INTERRUPT_PIN, INPUT);
  delay(2000); // Pause for 2 seconds


  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  delay(50);
  mpu.initialize();
  delay(1000); // pause for 1 second
  mpu2.initialize();

  delay(2000); // Pause for 2 seconds

  
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("First MPU6050 connection successful") : F("First MPU6050 connection failed"));
  delay(1000); // pause for 1 second  
  Serial.println(mpu.testConnection() ? F("Second MPU6050 connection successful") : F("Second MPU6050 connection failed"));
  delay(2000);
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  delay(1000); 
  mpu2.dmpInitialize();
  delay(1000);

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); 
  
  mpu2.setXGyroOffset(220);
  mpu2.setYGyroOffset(76);
  mpu2.setZGyroOffset(-85);
  mpu2.setZAccelOffset(1788); 

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      Serial.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();


      mpu2.CalibrateAccel(6);
      mpu2.CalibrateGyro(6);
      mpu2.PrintActiveOffsets();
      mpu2.setDMPEnabled(true);

      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      Serial.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus2 = mpu2.getIntStatus();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }

  delay(1000)
  SerialBT.begin("ESP32"); //Bluetooth device name

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    return;
    #ifdef OUTPUT_READABLE_QUATERNION
      // display quaternion values in easy matrix form: w x y z
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      Serial.print("quat\t");
      Serial.print(q.w);
      Serial.print("\t");
      Serial.print(q.x);
      Serial.print("\t");
      Serial.print(q.y);
      Serial.print("\t");
      Serial.println(q.z);
    #endif

    #ifdef OUTPUT_READABLE_EULER
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetEuler(euler, &q);
      Serial.print("euler\t");
      Serial.print(euler[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(euler[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(euler[2] * 180/M_PI);

      BTPackage[3] = euler[0] * 180/M_PI;
      BTPackage[4] = euler[1] * 180/M_PI;
      BTPackage[5] = euler[2] * 180/M_PI;
    #endif

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);

      BTPackage[0] = ypr[0] * 180/M_PI;
      BTPackage[1] = ypr[1] * 180/M_PI;
      BTPackage[2] = ypr[2] * 180/M_PI;
    #endif

    #ifdef OUTPUT_READABLE_REALACCEL
      // display real acceleration, adjusted to remove gravity
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      Serial.print("areal\t");
      Serial.print(aaReal.x);
      Serial.print("\t");
      Serial.print(aaReal.y);
      Serial.print("\t");
      Serial.println(aaReal.z);
    #endif
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }

  if (mpu2.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    #ifdef OUTPUT_READABLE_QUATERNION
      // display quaternion values in easy matrix form: w x y z
      mpu2.dmpGetQuaternion(&q, fifoBuffer);
      Serial.print("quat2\t");
      Serial.print(q.w);
      Serial.print("\t");
      Serial.print(q.x);
      Serial.print("\t");
      Serial.print(q.y);
      Serial.print("\t");
      Serial.println(q.z);
    #endif

    #ifdef OUTPUT_READABLE_EULER
      // display Euler angles in degrees
      mpu2.dmpGetQuaternion(&q, fifoBuffer);
      mpu2.dmpGetEuler(euler, &q);
      Serial.print("euler2\t");
      Serial.print(euler[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(euler[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(euler[2] * 180/M_PI);

      BTPackage[9] = euler[0] * 180/M_PI;
      BTPackage[10] = euler[1] * 180/M_PI;
      BTPackage[11] = euler[2] * 180/M_PI;
    #endif

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu2.dmpGetQuaternion(&q, fifoBuffer);
      mpu2.dmpGetGravity(&gravity, &q);
      mpu2.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr2\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);

      BTPackage[6] = ypr[0] * 180/M_PI;
      BTPackage[7] = ypr[1] * 180/M_PI;
      BTPackage[8] = ypr[2] * 180/M_PI;
    #endif

    #ifdef OUTPUT_READABLE_REALACCEL
      // display real acceleration, adjusted to remove gravity
      mpu2.dmpGetQuaternion(&q, fifoBuffer);
      mpu2.dmpGetAccel(&aa, fifoBuffer);
      mpu2.dmpGetGravity(&gravity, &q);
      mpu2.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      Serial.print("areal2\t");
      Serial.print(aaReal.x);
      Serial.print("\t");
      Serial.print(aaReal.y);
      Serial.print("\t");
      Serial.println(aaReal.z);
    #endif

    //sends sensor reading packet over Bluetooth
    SerialBT.println(BTPackage);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

