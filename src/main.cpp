#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050_6Axis_MotionApps612.h" // Uncomment this library to work with DMP 6.12 and comment on the above library.

#include "eye.h"
#include "eyes.h"

// Bewegingen zijn van vooraf gezien !!!
#define HLEFTLIMIT 30   // define left limit on horizontal (left/right) servo (was 30)
#define HRIGHTLIMIT 120 // define right limit on horizontal (left/right) servo (was 180)

#define VTOPLIMIT 40 // define top limit on vertical (up/down) servo (was 60)  (27-01-19: 90)
#define VBOTLIMIT 15 // define bottom limit on horizontal (up/down) servo (was 35) (27-01-19:20)

int eye_left_hor = 4;
int eye_left_vert = 5;
int eye_left_lid = 6;

int eye_right_hor = 7;
int eye_right_vert = 8;
int eye_right_lid = 9;

int randomhor;                 // variable to put random horizontal value
int randomvert;                // variable to put random tertical value
u32 randomdelay;               // variable to put random tertical value
int horbeginvalue;             // variabele voor horizontale beginpositie
int vertbeginvalue;            // variabele voor verticale beginpositie
int blinkbeginvalue;           // variabele voor blink beginpositie
unsigned long lastblinktime;   // tijd dat de vorige keer genipperd is
unsigned long lastlooktime;    // tijd dat de vorige keer naar een andere positie gekeken is
unsigned long currenttime;     // de huidige tijd om de delay te kunnen bepalen
unsigned long lastknipoogtime; // tijd dat de vorige keer geknipoogd werd

/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;
//MPU6050 mpu(0x69); //Use for AD0 high
//MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.

Eye eye_left;
Eye eye_right;
Eyes eyes;

#define OUTPUT_READABLE_YAWPITCHROLL

int const INTERRUPT_PIN = 2;  // Define the interruption #0 pin
bool blinkState;

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

/*-Packet structure for InvenSense teapot demo-*/ 
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

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

  eye_left.begin(eye_left_hor, eye_left_vert, eye_left_lid);
  eye_right.begin(eye_right_hor, eye_right_vert, eye_right_lid);

  eyes.begin(eye_left, eye_right);

  horbeginvalue = 30;                          // beginpositie van servo (was90)
  vertbeginvalue = 30;                         // beginpositie van servo (was90)
  randomhor = random(HLEFTLIMIT, HRIGHTLIMIT); // set limits horizontally
  randomvert = random(VBOTLIMIT, VTOPLIMIT);   // set limits vertically
  randomdelay = random(500, 1000);             // get a random delay value between 1 and 2 seconds
  eyes.blink();                                // blink so the eyelid is in top in starting position
  delay(200);
  lastblinktime = millis();                    // get last binking time
  lastlooktime = millis();                     // get last looking time
  lastknipoogtime = millis();                  // get last knipoog time
}

void program1()
{
  // vertical move up
  if (randomvert > vertbeginvalue)
  {
    if (randomvert - vertbeginvalue == 1)
      vertbeginvalue += 1;
    else
      vertbeginvalue += 2;
    eyes.lookAtVertical(vertbeginvalue);
    delay(10);
  }
  // vertical move down
  else if ((randomvert < vertbeginvalue))
  {
    if (vertbeginvalue - randomvert == 1)
      vertbeginvalue -= 1;
    else
      vertbeginvalue -= 2;

    eyes.lookAtVertical(vertbeginvalue);
    delay(10);
  }

  // horizontal move right
  if (randomhor > horbeginvalue)
  {
    if (randomhor - horbeginvalue == 1)
      horbeginvalue += 1;
    else
      horbeginvalue += 2;
    eyes.lookAtHorizontal(horbeginvalue);
    delay(10);
  }
  // horizontal move left
  else if (randomhor < horbeginvalue)
  {
    if (horbeginvalue - randomhor == 1)
      horbeginvalue -= 1;
    else
      horbeginvalue -= 2;

    eyes.lookAtHorizontal(horbeginvalue);
    delay(10);
  }
  currenttime = millis(); // write the current time to a variable

  // check if 6.5 seconds have passed since the last blink
  if ((currenttime - lastblinktime) >= 7500)
  {
    eyes.blink();
    delay(200);
    lastblinktime = millis(); // note last blink time
  }

  // check if the time since the last new looking position is greater then the delay time
  if ((currenttime - lastlooktime) > randomdelay)
  {
    randomhor = random(HLEFTLIMIT, HRIGHTLIMIT); // assign new random horizontal values
    randomvert = random(VBOTLIMIT, VTOPLIMIT);   // assign new random vertical vallues
    randomdelay = random(500, 1500);             // moves every 0,5 to 1 seconds (was 1 to 3)
    lastlooktime = millis();                     // note the new looking time
  }
}

void program2()
{
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

  //  if(is_bent_over()) {
  //   program1();
  //  } else {
  //   program2();
  //  }

  /* Blink LED to indicate activity */
  //blinkState = !blinkState;
  blinkState = is_bent_over();
  digitalWrite(LED_BUILTIN, blinkState);
}