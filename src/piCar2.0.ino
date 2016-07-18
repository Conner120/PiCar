//libary declarations
#include <SoftwareSerial.h>
#include <SPI.h>
#include "Adafruit_BLE_UART.h"
bool siren = false;
// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9
long eLight = 0;
int redLight = 0;
int blueLight = 0;
long TLC = 0;
Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
int frontUltra = 0;
int frontUltraPin = 0;

//definitons
#define ESTOP 0
#define DEBUG 0
#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100

//variable declarations
int horn = 23;
/*  VNH2SP30 pin definitions
  xxx[0] controls '1' outputs
  xxx[1] controls '2' outputs */
int inApin[2] = {7, 4};  // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input
int cspin[2] = {2, 3}; // CS: Current sense ANALOG input
int enpin[2] = {0, 1}; // EN: Status of switches output (Analog pin)
int RRWL = 35;
int LRWL = 32;
int RFWL = 36;
int LFWL = 37;
int RRRL = 33;
int LRRL = 34;
int seirnRed[5] = {22, 24, 26, 28, 30};
int seirnBlue[5] = {23, 25, 27, 29, 31};
int statpin = 13;
void setup() {
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);
  Serial.begin(57600);
  Serial1.begin(57600);
  pinMode(RRWL, OUTPUT);
  pinMode(LRWL, OUTPUT);
  pinMode(RRRL, OUTPUT);
  pinMode(LRRL, OUTPUT);
  pinMode(RFWL, OUTPUT);
  pinMode(LFWL, OUTPUT);
  pinMode(statpin, OUTPUT);
  Serial.println("startup");
  // Initialize digital pins as outputs
  for (int i = 0; i < 2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // Initialize braked
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }

}
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;
void alert(int level) {
  if (level == 1) {
    digitalWrite(horn, HIGH);
    // lcd.setCursor(0,0);
    // lcd.print("ESTOP level 1!");
  }
}
bool eMode = false;
void loop() {
  updateSensors();
  //seirn();
  if (eMode) {
    seirn();
  }
  if (Serial.available()) {
    serialProcess(0);
  }
  if (Serial1.available()) {
    serialProcess(1);
  }
}
void motorOff(int motor) {
  // Initialize braked
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  analogWrite(pwmpin[motor], 0);
}
/* motorGo() will set a motor going in a specific direction
  the motor will continue going in that direction, at that speed
  until told to do otherwise.

  motor: this should be either 0 or 1, will selet which of the two
  motors to be controlled

  direct: Should be between 0 and 3, with the following result
  0: Brake to VCC
  1: Clockwise
  2: CounterClockwise
  3: Brake to GND

  pwm: should be a value between ? and 1023, higher the number, the faster
  it'll go
*/

void serinLight(int red, int blue) {
  digitalWrite(RFWL, red);
  digitalWrite(LFWL, red);
  digitalWrite(26, red);
  digitalWrite(28, red);
  digitalWrite(30, red);
  digitalWrite(23, blue);
  digitalWrite(25, blue);
  digitalWrite(27, blue);
  digitalWrite(29, blue);
  digitalWrite(31, blue);
  //  digitalWrite(RRRL, red);
  //  digitalWrite(LRRL, red);
  redLight = red;
  blueLight = blue;
}
void seirn() {
  Serial.println(millis());
  if (millis() % 50) {
    tone(44, 1000);
  }
  if (millis() > eLight) {
    if (redLight == 0) {
      serinLight(1, 0);
    } else {
      serinLight(0, 1);
    }
    eLight = millis() + 250;
  }
}
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm) {

  if (motor <= 1)
  {
    if (direct <= 4)
    {
      // Set inA[motor]
      if (direct <= 1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct == 0) || (direct == 2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}
int parsein(int type) {
  if (type == 0) {
    int c = Serial.parseInt();
    return c;
  }
  if (type == 1) {
    int c = Serial1.parseInt();
    Serial.println(c);
    Serial1.read();
    return c;

    //    case 2:
    //      return BTLEserial.parseInt();
    //      break;
  }
}
char sRead(int type) {
  if (type == 0) {
    char c = Serial.read();
    Serial.println(c);
    return c;
  }  if (type == 1) {
    char c = Serial1.read();
    Serial.println(c);
    Serial1.read();

    return c;
    //    case 2:
    //      return BTLEserial.parseInt();
    //      break;
  }
}
void setHeadLight(int type) {
  int RRW = parsein(type);
  Serial.println("RRW: " + String(RRW));
  int LRR = parsein(type);
  int RFW = parsein(type);
  Serial.println("RRR: " + String(RFW));
  int LFW = parsein(type);
  Serial.println("LRR: " + String(LFW));
  int LRW = parsein(type);
  Serial.println("LRW: " + String(LRW));
  int RRR = parsein(type);
  Serial.println("RRR: " + String(RRR));
  Serial.println("LRR: " + String(LRR));
  Serial.read();
  digitalWrite(RRWL, RRW);
  digitalWrite(LRWL, LRW);
  digitalWrite(RFWL, RFW);
  digitalWrite(LFWL, LFW);
  digitalWrite(RRRL, LRR);
  digitalWrite(LRRL, RRR);
}
void flashHeadLight(int type) {

  int RRW = parsein(type);
  Serial.println("RRW: " + String(RRW));
  int LRW = parsein(type);
  Serial.println("LRW: " + String(LRW));
  int RRR = parsein(type);
  Serial.println("RRR: " + String(RRR));
  int LRR = parsein(type);
  Serial.println("LRR: " + String(LRR));
  int delayr = parsein(type);
  Serial.println("Delay: " + String(delayr));
  int times = parsein(type);
  Serial.println("times: " + String(times));
  Serial.read();
  for (int count = 0; count < times; count++) {
    Serial.println("set Lights");
    digitalWrite(RRWL, RRW);
    digitalWrite(LRWL, LRW);
    digitalWrite(RRRL, RRR);
    digitalWrite(LRRL, LRR);
    delay(delayr);
    digitalWrite(RRWL, LOW);
    digitalWrite(LRWL, LOW);
    digitalWrite(RRRL, LOW);
    digitalWrite(LRRL, LOW);
    delay(delayr);
  }
}
void serialProcess(int type) {
  TLC = millis();
  char r = sRead(type);
  Serial.println("Command Code: " + String(r));
  if (r == 'd') {
    int mm1 = parsein(type);
    int mm2 = parsein(type);
    int m1 = parsein(type);
    int m2 = parsein(type);

    if (m1 == 0) {
      motorGo(0, 0, 0);
    } else if (mm1 == 2) {
      motorGo(0, 1, m1 / 2);
    } else if (mm1 == 1) {
      motorGo(0, 2, m1 / 2);
    }
    if (m2 == 0) {
      motorGo(1, 0 , 0);
    } else if (mm2 == 1) {
      motorGo(1, 1, m2 / 2);
    } else if (mm2 == 2) {
      motorGo(1, 2, m2 / 2);
    }
    //    if (m1 == 0) {
    //      motorGo(0, 0, 0);
    //    } else if (m1 < 0) {
    //      motorGo(0, 1, m1);
    //    } else  {
    //      motorGo(0, 2, m1);
    //    }
    //    if (m2 == 0) {
    //      motorGo(1, 0, 0);
    //
    //    } else if (m2 < 0) {
    //      motorGo(1, 1, m2);
    //
    //    } else  {
    //      motorGo(1, 2, m2);
    //    }
  } else if (r == 'l') {
    char moder = sRead(type);
    if (moder == 's') {
      setHeadLight(type);
    } else if (moder == 'f') {
      flashHeadLight(type);
    } else if (moder == 'r') {
      Serial.println("Standered Lighting");
      if (eMode) {
        eMode = false;
        serinLight(0, 0);
      } else {
        eMode = true;
        serinLight(0, 0);
      }

    }
  }else if (r=='r'){
    char moder = sRead(type);
    if (moder=='s') {
      int sensor = parsein(type);
      switch (sensor) {
        case 0:
        //front ultrasonic
        if (type==0){
          int frontDist = frontUltra;
          Serial.println(frontDist);
        }
        break;
      };
    } else if (moder=='c') {
      /* code */
    }
  }
}
int debug() {
  int level; // level is return telling the code the severity of the error and issues the command to fix the isuue of
  // the code and if nessecary shutdown and the robot and stop all movment and actavate ESTOP and alert code
  if (DEBUG == 4) {

  }


  return (level);
}

void updateSensors() {
  //front ultra sonic
  frontUltra = readUltra();
  if (frontUltra<7){
    motorGo(0, 0, 0);
    motorGo(1, 0, 0);
  }
}
int readUltra(){
  return 0;
}
