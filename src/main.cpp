#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050_6Axis_MotionApps612.h" // Uncomment this library to work with DMP 6.12 and comment on the above library.

#include "eye.h"
#include "eyes.h"

/*--------------------*/
/* ----- CONFIG ----- */
/*--------------------*/

/* PINS */
int eye_left_hor_pin = 4;   // Eye left horizontal servo pin
int eye_left_vert_pin = 5;  // Eye left vertical servo pin
int eye_left_lid_pin = 6;   // Eye left eyelid servo pin

int eye_right_hor_pin = 7;  // Eye right horizontal servo pin
int eye_right_vert_pin = 8; // Eye right vertical servo pin
int eye_right_lid_pin = 9;  // Eye right eyelid servo pin

/* RANGES */
// These values represent the absolute servo positions
// the range given will be translated to 0 to 180 in the eye class

int eye_left_hor_min = 30;    // Eye left horizontal minimum
int eye_left_hor_max = 120;   // Eye left horizontal maximum
int eye_left_vert_min = 40;   // Eye left vertical minimum
int eye_left_vert_max = 140;  // Eye left vertical maximum
int eye_left_lid_min = 40;    // Eye left eyelid minimum
int eye_left_lid_max = 140;   // Eye left eyelid maximum

int eye_right_hor_min = 30;    // Eye right horizontal minimum
int eye_right_hor_max = 120;   // Eye right horizontal maximum
int eye_right_vert_min = 40;   // Eye right vertical minimum
int eye_right_vert_max = 140;  // Eye right vertical maximum
int eye_right_lid_min = 40;    // Eye right eyelid minimum
int eye_right_lid_max = 140;   // Eye right eyelid maximum

/* Bent over */
int eyes_vertical_bent_over = 140;  // Vertical position of the eyes when bent over

/* Intervals */
int blink_interval = 7500;   // Blink interval time in milli seconds

/*------------------------*/
/* ----- END CONFIG ----- */
/*------------------------*/

Eye eye_left;
Eye eye_right;
Eyes eyes;
bool blinkState;
unsigned long lastBlinkTime;
unsigned long currentTime;
int randomHorizontalPoint;
int randomVerticalPoint;
int currentHorizontalPoint;
int currentVerticalPoint;

/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;
//MPU6050 mpu(0x69); //Use for AD0 high
//MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.

#define OUTPUT_READABLE_YAWPITCHROLL

int const INTERRUPT_PIN = 2;  // Define the interruption #0 pin

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/ 
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

void setup_mpu()
{
  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else {
    Serial.println("MPU6050 connection successful");
  }

  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  } 
  else {
    Serial.print(F("DMP Initialization failed (code ")); //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
  pinMode(LED_BUILTIN, OUTPUT);
}

void setup()
{
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200);
  while (!Serial);

  setup_mpu();

  eye_left.begin(eye_left_hor_pin, eye_left_vert_pin, eye_left_lid_pin);
  eye_left.setLimitsHorizontal(eye_left_hor_min, eye_left_hor_max);
  eye_left.setLimitsVertical(eye_left_vert_min, eye_left_vert_max);
  eye_left.setLimitsLid(eye_left_lid_min, eye_left_lid_max);

  eye_right.begin(eye_right_hor_pin, eye_right_vert_pin, eye_right_lid_pin);
  eye_right.setLimitsHorizontal(eye_right_hor_min, eye_right_hor_max);
  eye_right.setLimitsVertical(eye_right_vert_min, eye_right_vert_max);
  eye_right.setLimitsLid(eye_right_lid_min, eye_right_lid_max);

  eyes.begin(eye_left, eye_right);

  // "init" and center eyes;
  randomHorizontalPoint = 90;
  randomVerticalPoint = 90;
  currentHorizontalPoint = 90;
  currentVerticalPoint = 90;
}

void programUpRight()
{
  if (randomVerticalPoint > currentVerticalPoint) {
    currentVerticalPoint++;
  } else if (randomVerticalPoint < currentVerticalPoint) {
    currentVerticalPoint--;
  }
  eyes.lookAtVertical(currentVerticalPoint);
  delay(10);

  if (randomHorizontalPoint > currentHorizontalPoint) {
    currentHorizontalPoint++;
  } else if (randomHorizontalPoint < currentHorizontalPoint) {
    currentHorizontalPoint--;
  }
  eyes.lookAtHorizontal(currentHorizontalPoint);
  delay(10);

  if ((currentTime - lastBlinkTime) >= blink_interval)
  {
    eyes.blink();
    // delay(200);
    lastBlinkTime = millis(); // note last blink time
  }

  if (randomVerticalPoint ==  currentVerticalPoint && randomHorizontalPoint == currentHorizontalPoint)
  {
    randomHorizontalPoint = random(0, 180);    // new random horizontal values
    randomVerticalPoint = random(0, 180);      // new random vertical vallues
  }
}

void programBentOver()
{
  // make sure the eyes are somewhat closed
  eyes.lidAt(90);

  // in bend over we don't look up/down, only left/right and down
  eyes.lookAtVertical(eyes_vertical_bent_over);

  if (randomHorizontalPoint > currentHorizontalPoint) {
    currentHorizontalPoint++;
  } else if (randomHorizontalPoint < currentHorizontalPoint) {
    currentHorizontalPoint--;
  }
  eyes.lookAtHorizontal(currentHorizontalPoint);
  delay(10);

  if ((currentTime - lastBlinkTime) >= blink_interval)
  {
    eyes.blink();
    // delay(200);
    lastBlinkTime = millis(); // note last blink time
  }

  if (randomHorizontalPoint == currentHorizontalPoint)
  {
    randomHorizontalPoint = random(0, 180);
  }

  if ((millis() - lastBlinkTime) >= blink_interval)
  {
    // not a "regular" blink but keep the eyes half shut
    eyes.lidAt(180);
    delay(500);
    eyes.lidAt(90);
    // delay(200);
    lastBlinkTime = millis();
  }
}

bool is_bent_over()
{
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
      /* Display Euler angles in degrees */
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      float x = ypr[1] * 180/M_PI;
      float y = ypr[2] * 180/M_PI;
      bool x_bend = x < -45. || x > 45.;
      bool y_bend = y < -45. || y > 45.;
      return x_bend || y_bend;
   }
  return false;
}

void debug_mpu_loop() {
   if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
      /* Display Euler angles in degrees */
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      float x = ypr[1] * 180/M_PI;
      float y = ypr[2] * 180/M_PI;
      bool x_bend = x < -45. || x > 45.;
      bool y_bend = y < -45. || y > 45.;
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[2] * 180/M_PI);
      Serial.print("\t");
      Serial.println(x_bend || y_bend);
   }
}

void loop()
{
  if (!DMPReady) return;

  debug_mpu_loop();

  if(is_bent_over()) {
    programBentOver();
  } else {
    programUpRight();
  }

  /* Blink LED to indicate activity */
  //blinkState = !blinkState;
  blinkState = is_bent_over();
  digitalWrite(LED_BUILTIN, blinkState);
}