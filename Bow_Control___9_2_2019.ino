#include <ServoTimer2.h> // MUST use this library, not Servo.h, to not conflict with Timer1 library 
#include <TimerOne.h>
#include <SoftwareSerial.h>
#include <MIDI.h>
#define PIN_RAW_INPUT 2

//Serial initialization
MIDI_CREATE_DEFAULT_INSTANCE();

//rotating encoder constants
const float delta_time = 20000; //period of timer interrupt in microseconds)
const float counts_per_rev = 464.64; //number of encoder counts per revolution
const float count2RPS = (1 / (delta_time*pow(10, -6) * counts_per_rev)); //converts number of counts to RPS

// Define H-bridge pins and Encoder pins
const int Hleft = 10;
const int Hright = 9; //must use pins 9 and 10 when also using the ServoTimer2 library
//Hleft and Hright must be analog pins (denoted on arduino by 10~ and 11~)
const int pinA = 2; // you MUST use pins 2 and 3
const int pinB = 3; // this will not work on other pins!
volatile long encoder = 0; //encoder count that is reset in integral control
volatile long encTotal = 0; //total encoder count
volatile bool CCW = true; //defaut direction is counter-clockwise
volatile bool isOn = false; //is the motor running? default to off

// Integral Control Variables
volatile long oldPos = 0;
volatile long newPos = 0;
volatile float spd = 0; //speed
int refSpd = 25; // default motor speed
int refSpdCheck = 0;
float error = refSpd - spd;
float int_e = 0; //integral of error
const float kp = 3;
const float ki = 0.6;
volatile int out = 0;

// Setting the hall effect sensors to change the direction of the motor
const int heff1 = 11;
const int heff2 = 12;

//SERVO initialization
ServoTimer2 myServo; //define myServo variable
int angle = 0; //doesn't matter for continous servo
int pos = 1100;
int noMo = 1520;

void setup() {
  //Pin setup
  pinMode(Hleft, OUTPUT);
  pinMode(Hright, OUTPUT);//using an H bridge allows the motor to switch directions
  pinMode(heff1, INPUT); // set up for the hall effect sensors
  pinMode(heff2, INPUT);

  //  pinMode(D6, OUTPUT); //lights up when homing position is reached
  //  pinMode(D7, OUTPUT); //lights up while homing code is running
  //  digitalWrite(D6, HIGH);//With the MIDI shield, LOW turns LEDs on, and HIGH turns them OFF
  //  pinMode(buttonPinA, INPUT); //homing button

  //MIDI setup
  //MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.begin(16); //sets MIDI channel 
  Serial.begin(31250);
  /*default MIDI baud rate -- arduino cannot monitor this baud rate,
    so you will need to use an external serial monitor such as PuTTY*/
  Serial.println("ready");

  //Timer setup
  attachInterrupt(digitalPinToInterrupt(pinA), isrA, CHANGE); //interrupt which increments encoder tick count

  //SERVO setup
  myServo.attach(5); //servo PWM pin attached to arduino pin 5
  // myServo.write(noMo);

  //  analogWrite(Hleft, LOW);
  //  digitalWrite(Hright, LOW);
}
void loop() {
  if (MIDI.read()) //if MIDI sends messages
  {
    Serial.println(MIDI.getType()); //prints first 4 bits of Status byte (type of message)
    Serial.println(MIDI.getData1()); //prints Data byte 1
    Serial.println(MIDI.getData2()); //prints Data byte 2
    //both data bytes change meaning based on the type of message received
    Serial.println("_____________");
    switch (MIDI.getType())
    {
      case midi::NoteOn:
        {
          switch (MIDI.getData1())
          {
            
            case 36: // to bot
              {
               digitalWrite(Hleft, LOW);
               analogWrite(Hright, 175);
               
               //while loop until sensor reads something 
               while (digitalRead(heff1) == HIGH) 
               {
               }
               digitalWrite(Hleft, HIGH);
               digitalWrite(Hright, HIGH);
               CCW = true;
               
              }
              break;
            case 37: // to top
              {
               digitalWrite(Hright, LOW);
               analogWrite(Hleft, 175);
               
               //while loop until sensor reads something 
               while (digitalRead(heff2) == HIGH) 
               {
               }
               digitalWrite(Hleft, HIGH);
               digitalWrite(Hright, HIGH);
               CCW = false;
              }
              break;
            case 40:
              {
                for (pos; pos >= 1100; pos -= 3) {
                  myServo.write(pos);
                  delay(2);
                  Serial.println(myServo.read());
                  MagnetSwitch();
                }
              }
              break;
            case 41:
              {
                if (pos < 1300)
                {
                  for (pos; pos <= 1300; pos += 3) {
                    myServo.write(pos);
                    delay(2);
                    Serial.println(myServo.read());
                    MagnetSwitch();
                  }
                }
                else if (pos > 1300)
                {
                  for (pos; pos >= 1300; pos -= 3) {
                    myServo.write(pos);
                    delay(2);
                    Serial.println(myServo.read());
                    MagnetSwitch();
                  }
                }
              }
              break;
            case 42:
              {
                if (pos < 1500)
                {
                  for (pos; pos <= 1500; pos += 3) {
                    myServo.write(pos);
                    delay(2);
                    Serial.println(myServo.read());
                    MagnetSwitch();
                  }
                }
                else if (pos > 1500)
                {
                  for (pos; pos >= 1500; pos -= 3) {
                    myServo.write(pos);
                    delay(2);
                    Serial.println(myServo.read());
                    MagnetSwitch();
                  }
                }
              }
              break;
            case 43:
              {
                for (pos; pos <= 1700; pos += 5) {
                  myServo.write(pos);
                  delay(2);
                  Serial.println(myServo.read());
                  MagnetSwitch();
                }
              }
              break;
            case 44:
              {
                for (pos; pos >= 1100; pos -= 3) {
                  myServo.write(pos);
                  delay(15);
                  Serial.println(myServo.read());
                  MagnetSwitch();
                }
              }
              break;
            case 45:
              {
                if (pos < 1300)
                {
                  for (pos; pos <= 1300; pos += 3) {
                    myServo.write(pos);
                    delay(15);
                    Serial.println(myServo.read());
                    MagnetSwitch();
                  }
                }
                else if (pos > 1300)
                {
                  for (pos; pos >= 1300; pos -= 3) {
                    myServo.write(pos);
                    delay(15);
                    Serial.println(myServo.read());
                    MagnetSwitch();
                  }
                }
              }
              break;
            case 46:
              {
                if (pos < 1500)
                {
                  for (pos; pos <= 1500; pos += 3) {
                    myServo.write(pos);
                    delay(15);
                    Serial.println(myServo.read());
                    MagnetSwitch();
                  }
                }
                else if (pos > 1500)
                {
                  for (pos; pos >= 1500; pos -= 3) {
                    myServo.write(pos);
                    delay(15);
                    Serial.println(myServo.read());
                    MagnetSwitch();
                  }
                }
              }
              break;
            case 47:
              {
                for (pos; pos <= 1700; pos += 3) {
                  myServo.write(pos);
                  delay(15);
                  Serial.println(myServo.read());
                  MagnetSwitch();
                }
              }
              break;
            case 48://on/off
              {
                if (isOn)//turn off
                {
                  digitalWrite(Hleft, HIGH);
                  digitalWrite(Hright, HIGH);
                  spd = 0;
                  out = 0;
                }
                else//turn on
                {
                  if (out == 0) intControl();
                  else
                  {
                    if (CCW == true) {
                      analogWrite(Hleft, out);
                      digitalWrite(Hright, LOW);
                    }
                    else if (CCW == false) {
                      digitalWrite(Hleft, LOW);
                      analogWrite(Hright, out);
                    }
                  }
                }
                isOn = !isOn;
                Serial.println("on/off CHANGE");
              }
              break;
            case 49://switch direction
              {
                CCW = !CCW; //switches bow direction
                Serial.println("Direction CHANGE");
                if (isOn) //if on, will switch direction
                {
                  if (CCW == true)
                  {
                    analogWrite(Hleft, out);
                    digitalWrite(Hright, LOW);
                  }
                  else if (CCW == false)
                  {
                    digitalWrite(Hleft, LOW);
                    analogWrite(Hright, out);
                  }
                }
              }
              break;
            case 50://set ccw
              { CCW = true;
                if (isOn)
                {
                  analogWrite(Hleft, out);
                  digitalWrite(Hright, LOW);
                }
              }
              break;
            case 51://set cw
              {
                CCW = false;
                if (isOn)
                {
                  digitalWrite(Hleft, LOW);
                  analogWrite(Hright, out);
                }
              }
              break;
          }
        }
        break;

      case midi::ControlChange :
        {
          refSpdCheck = MIDI.getData2();
          if (refSpdCheck == 0) refSpdCheck = 0;
          //else if (refSpdCheck <= 20) refSpdCheck = 15;
          else if (refSpdCheck <= 40) refSpdCheck = 18;
          else if (refSpdCheck <= 60) refSpdCheck = 21;
          else if (refSpdCheck <= 80) refSpdCheck = 24;
          else if (refSpdCheck <= 100) refSpdCheck = 28;
          else if (refSpdCheck < 127) refSpdCheck = 33;
          else refSpdCheck = 40;
          if (refSpdCheck != refSpd)// only if refSpd has changed
          {
            refSpd = refSpdCheck;
            Serial.print(refSpd);
            Serial.println("_____");
            if (refSpd == 0)
            {
              analogWrite(Hleft, LOW);
              digitalWrite(Hright, LOW);
              oldPos = 0;
              newPos = 0;
              spd = 0;
            }
            else if (isOn) intControl();
          }
        }
        break;
    }
  }

  //  if (abs(encTotal) > 2400) // checks to make sure that bow will not run off track
  //  {
  //    digitalWrite(Hleft, HIGH);
  //    digitalWrite(Hright, HIGH);
  //    analogWrite(Hleft, 120);
  //    digitalWrite(Hright, LOW); // up bow
  //    delay(250);
  //    digitalWrite(Hleft, HIGH);
  //    digitalWrite(Hright, HIGH);
  //    isOn = false;
  //  }
  //  if (digitalRead(buttonPinA) == HIGH)
  //  {
  //    digitalWrite(Hleft, HIGH);
  //    digitalWrite(Hright, HIGH);
  //    analogWrite(Hleft, LOW);
  //    digitalWrite(Hright, 120); // down bow
  //    delay(250);
  //    digitalWrite(Hleft, HIGH);
  //    digitalWrite(Hright, HIGH);
  //    isOn = false;
  //  }

  MagnetSwitch();
}


  //hall-effect sensors
  void MagnetSwitch(){
    if ( CCW == false) {
      if ( (digitalRead(heff1) == LOW)) {
        CCW = true;
        Serial.println("h1L CW to CCW");
  
        analogWrite(Hleft, out);
        digitalWrite(Hright, LOW);
  
      }
    }
  if (CCW == true) {
    if ( (digitalRead(heff2) == LOW)) {
      CCW = false;
      Serial.println("h2L CCW to CW");

      digitalWrite(Hleft, LOW);
      analogWrite(Hright, out);

    }
  }

}

void isrA() //this is called whenever pinA (2) on the encoder changes value (i.e. when the encoder turns)
{
  char A = digitalRead(pinA);
  char B = digitalRead(pinB);
  //determine if clockwise or counter clockwise, and increments encoder accordingly
  if (A == HIGH)
  {
    if (B == HIGH)
    {
      encoder --;
      encTotal --;
    }
    else
    {
      encoder ++;
      encTotal ++;
    }
  }
  else //if A is LOW
  {
    if (B == HIGH)
    {
      encoder ++;
      encTotal ++;
    }
    else
    {
      encoder --;
      encTotal --;
    }
  }
}

void intControl() {
  error = refSpd - spd;
  oldPos = abs(encoder);
  delay(20);
  while (abs(error) >= 1) { //maybe think of a better way to do this
    //INTEGRAL CONTROL OF SPEED
    //READ: store current encoder position and speed calculation
    newPos = abs(encoder);
    Serial.print("New Pos:");
    Serial.print(newPos);
    spd = abs(newPos - oldPos) * count2RPS * 2 * PI;
    Serial.print(" Spd:");
    Serial.print(spd);
    //ERROR
    error = refSpd - spd;
    Serial.print(" Err:");
    Serial.println(error);
    //DECIDE
    int_e = int_e + error;//compute integgral of error
    Serial.print(" Int Err:");
    Serial.println(int_e);
    out = (ki * int_e) + (kp * error); //compute output
    //oldPos = newPos;
    Serial.print(" Out:");
    out = constrain(out, 120, 255);
    Serial.println(out);
    Serial.println("_____");

    ///hall effect sensors

    if ( CCW == false) {
      if ( (digitalRead(heff1) == LOW)) {
        CCW = true;

        Serial.println("h1i CW to CCW");

      }
    }
    if (CCW == true) {
      if ( (digitalRead(heff2) == LOW)) {
        CCW = false;


        Serial.println("h2i CCW to CW");

      }
    }

    //DIRECTION CONTROL
    if (CCW == true) {
      analogWrite(Hleft, out);
      digitalWrite(Hright, LOW);
    }
    else if (CCW == false) {
      digitalWrite(Hleft, LOW);
      analogWrite(Hright, out);
    }
    oldPos = newPos;
    delay(20);

  }
}
