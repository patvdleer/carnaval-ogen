#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

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

Adafruit_MPU6050 mpu;
Eye eye_left;
Eye eye_right;
Eyes eyes;

void debug_mpu()
{
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

}

void setup_mpu()
{
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // setup motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true); // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  // debug_mpu();
}

void setup()
{
  Serial.begin(115200);
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

bool is_bent_over()
{
  if (mpu.getMotionInterruptStatus())
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    return g.gyro.x > 90;
  }
  return false;
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


void debug_mpu_loop() {
  if (mpu.getMotionInterruptStatus())
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    /* Print out the values */
    Serial.print("AX:");
    Serial.print(a.acceleration.x);
    Serial.print(",");
    Serial.print("AY:");
    Serial.print(a.acceleration.y);
    Serial.print(",");
    Serial.print("AZ:");
    Serial.print(a.acceleration.z);
    Serial.print("  m/s^2, ");
    Serial.print("GX:");
    Serial.print(g.gyro.x);
    Serial.print(",");
    Serial.print("GY:");
    Serial.print(g.gyro.y);
    Serial.print(",");
    Serial.print("GZ:");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");

  }
  // delay(100);
}

void loop()
{
  debug_mpu_loop();

  //  if(is_bent_over()) {
  //   program1();
  //  } else {
  //   program2();
  //  }
}