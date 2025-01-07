  // Autokoplampen Random eye movement Sjteivig 2023 v1.0

#include <Servo.h> //include the servo library for servo control 

Servo horServo; //servo1 for left/right movement
Servo horServo2; //servo2 for left/right movement
Servo vertServo; //servo1 for up/down movement
Servo vertServo2; //servo2 for up/down movement
Servo blinkServo; //servo1 for blinking 
Servo blinkServo2; //servo2 for blinking
int randomhor; //variable to put random horizontal value
int randomvert; //variable to put random tertical value
int randomdelay; //variable to put random tertical value
int horbeginvalue;  //variabele voor horizontale beginpositie
int vertbeginvalue; //variabele voor verticale beginpositie
int blinkbeginvalue;  //variabele voor blink beginpositie
unsigned long lastblinktime;  //tijd dat de vorige keer genipperd is
unsigned long lastlooktime; //tijd dat de vorige keer naar een andere positie gekeken is
unsigned long currenttime;  //de huidige tijd om de delay te kunnen bepalen
unsigned long lastknipoogtime; //tijd dat de vorige keer geknipoogd werd

//Bewegingen zijn van vooraf gezien !!!
#define HLEFTLIMIT 30 //define left limit on horizontal (left/right) servo (was 30)
#define HRIGHTLIMIT 120 //define right limit on horizontal (left/right) servo (was 180)

#define VTOPLIMIT 40 //define top limit on vertical (up/down) servo (was 60)  (27-01-19: 90)
#define VBOTLIMIT 15 //define bottom limit on horizontal (up/down) servo (was 35) (27-01-19:20)


void setup() 
{ 
  Serial.begin(9600);
  horServo.attach(4); //horizontal servo1 on pin 4 (= eye 1)
  vertServo.attach(5); //vertical servo1 on pin 5 (= eye 1)
  blinkServo.attach(6); //vertical servo1 on pin 6 (= eye 1)
  horServo2.attach(7); //horizontal servo2 on pin 7 (= eye 2)rechter oog vanuit kop gezien
  vertServo2.attach(8); //vertical servo2 on pin 8 (= eye 2) rechter oog vanuit kop gezien
  blinkServo2.attach(9); //vertical servo2 on pin 9 (= eye 2)rechter oog vanuit kop gezien

    
  horbeginvalue = 30;   //beginpositie van servo (was90)
  vertbeginvalue = 30; //beginpositie van servo (was90)
  randomhor = random(HLEFTLIMIT, HRIGHTLIMIT); //set limits horizontally
  randomvert = random(VBOTLIMIT, VTOPLIMIT); //set limits vertically
  randomdelay = random(500, 1000); //get a random delay value between 1 and 2 seconds
  Blink();                          //blink so the eyelid is in top in starting position
  lastblinktime= millis();          //get last binking time
  lastlooktime= millis();           //get last looking time
  lastknipoogtime=millis();         //get last knipoog time
} 


void loop() 
{  
   if(randomvert> vertbeginvalue){            //vertical move up 
    if(randomvert - vertbeginvalue == 1){     //check if the difference is only 1 
      vertbeginvalue+=1;                       // vertbeginvalue = vertbeginvalue + 1         
      }
      else{
        vertbeginvalue+=2;
        } 
   vertServo.write(vertbeginvalue);           //write the new value to servo1 //correctie ivm afwijking servo
   vertServo2.write(vertbeginvalue);           //write the new value to servo2
   
   delay(10);
    }
  else if((randomvert< vertbeginvalue)){      //vertical move down
    if(vertbeginvalue - randomvert == 1){
      vertbeginvalue-=1;
      }
      else{
        vertbeginvalue-=2;
        } 
   vertServo.write(vertbeginvalue);
   vertServo2.write(vertbeginvalue);
   delay(10);
    }
    
  if(randomhor > horbeginvalue){          // horizontal move right
    if(randomhor - horbeginvalue  == 1){
      horbeginvalue+=1;
      }
      else{
        horbeginvalue+=2;
        } 
   horServo.write(horbeginvalue);
   horServo2.write(horbeginvalue);//+ 20 waarde ivm teveel naar links geeken vanuitachterkop naar voren gezien gezien
   delay(10);
    }
  else if(randomhor < horbeginvalue){     // horizontal move left
    if(horbeginvalue - randomhor  == 1){
      horbeginvalue-=1;
      }
      else{
        horbeginvalue-=2;
        } 
   horServo.write(horbeginvalue);
   horServo2.write(horbeginvalue);
   delay(10);
    }
    currenttime = millis();                 //write the current time to a variable


if((currenttime - lastblinktime) >=7500){      //check if 6.5 seconds have passed since the last blink
  Blink();                                       
  lastblinktime = millis();                     //note last blink time
  } 



if((currenttime - lastlooktime) > randomdelay){     //check if the time since the last new looking position is greater then the delay time

  randomhor = random(HLEFTLIMIT, HRIGHTLIMIT);          //assign new random horizontal values
  randomvert = random(VBOTLIMIT, VTOPLIMIT);            //assign new random vertical vallues
 // Serial.print("Randomvertical  ");
 // Serial.println(randomvert);
 // Serial.print("Randomhorizontal  ");
 // Serial.println(randomhor);
 // Serial.print("lastknipoogtime ");
 // Serial.println(lastknipoogtime); 
  randomdelay = random(500, 1500);               //moves every 0,5 to 1 seconds (was 1 to 3) 
  lastlooktime = millis();                        //note the new looking time
  }
    
}



void Blink(void){                                                       //blink function
    blinkServo.write(40);                                               //close the eye (was 40)
    blinkServo2.write(40);
    
  
    delay(1000);                                                         //some delay to allow the eye to close
    blinkServo.write(180); //write to the vertical servo 1               //open eye
    blinkServo2.write(150); //write to the vertical servo 1               //open eye (was 170)
    delay(200);
  }

  
