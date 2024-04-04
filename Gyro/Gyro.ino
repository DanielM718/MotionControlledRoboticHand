#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define SERVICE_UUID "3e694c3e-9087-4d47-afce-cd08dbde1286"
#define CHARACTERISTIC_UUID "c122a3d6-8d8a-4b89-b21f-4d91a260aeee"
#define CHARACTERISTIC_UUID2 "bcd9f8bf-b4da-4301-9c38-20fe24e9efe7"

bool deviceConnected = false;
// IMU data
BLECharacteristic imuCharacteristics(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor imuDescriptor(BLEUUID((uint16_t)0x2902));
// IMU identity
BLECharacteristic identityCharacteristics(CHARACTERISTIC_UUID2, BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor idenityDescriptor(BLEUUID((uint16_t)0x2901));

//Setup callbacks onConnect and onDisconnect
class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

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

int DataID;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // initialize serial communication
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // Create the BLE Device
  BLEDevice::init("IMUESP32");
  // sets the devices as a BLE server with the UUID
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // creating the BLE service
  BLEService *imuService = pServer->createService(SERVICE_UUID);

  // this defines the characteristics of the BLE service
  imuService->addCharacteristic(&imuCharacteristics);
  imuDescriptor.setValue("IMU sensor readings");
  imuCharacteristics.addDescriptor(&imuDescriptor);

  imuService->addCharacteristic(&identityCharacteristics);
  idenityDescriptor.setValue("IMU identity");
  identityCharacteristics.addDescriptor(&idenityDescriptor);

  // starts the service
  imuService->start();

  // starts advertising service
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");

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

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO & checks BLE connection
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer) && deviceConnected) { // Get the Latest packet 
    #ifdef OUTPUT_READABLE_EULER
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetEuler(euler, &q);
      Serial.println("euler\t");
      
      DataID = 11;
      identityCharacteristics.setValue(DataID);
      identityCharacteristics.notify();
      euler[0] *= 180/M_PI;
      //Serial.print(euler[0]);
      imuCharacteristics.setValue(euler[0]);
      imuCharacteristics.notify();

      DataID = 12;
      identityCharacteristics.setValue(DataID);
      identityCharacteristics.notify();
      euler[1] *= 180/M_PI;
      //Serial.print(euler[1]);
      imuCharacteristics.setValue(euler[1]);
      imuCharacteristics.notify();

      DataID = 13;
      identityCharacteristics.setValue(DataID);
      identityCharacteristics.notify();
      euler[2] *= 180/M_PI;
      //Serial.print(euler[2]);
      imuCharacteristics.setValue(euler[2]);
      imuCharacteristics.notify();

    #endif

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.println("ypr\t");
      DataID = 14;
      identityCharacteristics.setValue(DataID);
      identityCharacteristics.notify();
      ypr[0] *= 180/M_PI;
      //Serial.print(ypr[0]);
      imuCharacteristics.setValue(ypr[0]);
      imuCharacteristics.notify();

      DataID = 15;
      identityCharacteristics.setValue(DataID);
      identityCharacteristics.notify();
      euler[1] *= 180/M_PI;
      //Serial.print(ypr[1]);
      imuCharacteristics.setValue(ypr[1]);
      imuCharacteristics.notify();

      DataID = 16;
      identityCharacteristics.setValue(DataID);
      identityCharacteristics.notify();
      euler[2] *= 180/M_PI;
      //Serial.print(ypr[2]);
      imuCharacteristics.setValue(ypr[2]);
      imuCharacteristics.notify();


    #endif
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }

  if (mpu2.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    #ifdef OUTPUT_READABLE_EULER
      // display Euler angles in degrees
      mpu2.dmpGetQuaternion(&q, fifoBuffer);
      mpu2.dmpGetEuler(euler, &q);
      Serial.println("euler2\t");
      DataID = 21;
      identityCharacteristics.setValue(DataID);
      identityCharacteristics.notify();
      euler[0] *= 180/M_PI;
      //Serial.print(euler[0]);
      imuCharacteristics.setValue(euler[0]);
      imuCharacteristics.notify();

      DataID = 22;
      identityCharacteristics.setValue(DataID);
      identityCharacteristics.notify();
      euler[1] *= 180/M_PI;
      //Serial.print(euler[1]);
      imuCharacteristics.setValue(euler[1]);
      imuCharacteristics.notify();

      DataID = 23;
      identityCharacteristics.setValue(DataID);
      identityCharacteristics.notify();
      euler[2] *= 180/M_PI;
      //Serial.print(euler[2]);
      imuCharacteristics.setValue(euler[2]);
      imuCharacteristics.notify();


    #endif

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu2.dmpGetQuaternion(&q, fifoBuffer);
      mpu2.dmpGetGravity(&gravity, &q);
      mpu2.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.println("ypr2\t");
      DataID = 24;
      identityCharacteristics.setValue(DataID);
      identityCharacteristics.notify();
      ypr[0] *= 180/M_PI;
      //Serial.print(ypr[0]);
      imuCharacteristics.setValue(ypr[0]);
      imuCharacteristics.notify();

      DataID = 25;
      identityCharacteristics.setValue(DataID);
      identityCharacteristics.notify();
      euler[1] *= 180/M_PI;
      //Serial.print(ypr[1]);
      imuCharacteristics.setValue(ypr[1]);
      imuCharacteristics.notify();

      DataID = 26;
      identityCharacteristics.setValue(DataID);
      identityCharacteristics.notify();
      euler[2] *= 180/M_PI;
      //Serial.print(ypr[2]);
      imuCharacteristics.setValue(ypr[2]);
      imuCharacteristics.notify();


    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

