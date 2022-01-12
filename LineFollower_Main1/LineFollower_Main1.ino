#include <QTRSensors.h>

enum direction{
  forward,
  backward,
  right,
  left
};

//sensor variables
QTRSensors qtr;

const uint8_t SensorCount = 2;
uint16_t sensorValues[SensorCount];

//DC motor variables
int forwardSpeed=100;
int turningSpeed=80;

//motorStanga
int motor1pin1=2;//fata
int motor1pin2=3;//spate

//motorDreapta
int motor2pin1=4;//spate
int motor2pin2=5;//fata

//LED
int ledRosuStanga = 53;
int ledRosuDreapta = 51;

int LEDTimer = 10; //the time,in ms,in which an LED blink takes place
//currently not used,seems that a delay would be detrimental to the bot's reaction speed

//miscellaneous variables
direction toMove;
direction way;

bool doTest = false;  //debug purpose only,should be TRUE by default
//it controls wether or not the DC motors do a test sequence

void setup() {
  setupLED();
  setupMotors();
  if(doTest == true){testMotor();}
  setupSensors();
}

void setupLED(){
  pinMode(ledRosuStanga,OUTPUT);
  digitalWrite(ledRosuStanga,LOW);

  pinMode(ledRosuDreapta,OUTPUT);
  digitalWrite(ledRosuDreapta,LOW);
}

void setupMotors(){
  pinMode(motor1pin1,OUTPUT);
  pinMode(motor1pin2,OUTPUT);
  
  pinMode(motor2pin1,OUTPUT);
  pinMode(motor2pin2,OUTPUT);

  Stop();
}

void testMotor(){
  digitalWrite(ledRosuStanga,HIGH);
  digitalWrite(ledRosuDreapta,HIGH);
  
  turnLeft();
  delay(1000);
  
  turnRight();
  delay(2000);
  Stop();
  
  digitalWrite(ledRosuStanga,LOW);
  digitalWrite(ledRosuDreapta,LOW);
}

void setupSensors(){
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1}, SensorCount);
  qtr.setEmitterPin(10); //nu pune un pin pe care sunt puse si motoarele!!!

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void loop() {
  oneDirection();
}

uint16_t readSensors(){
  return qtr.readLineBlack(sensorValues);
}

direction determineMovement(){
  uint16_t whereToMove = readSensors();
  Serial.println(whereToMove);

  if(whereToMove >=300 && whereToMove <=700){
    return forward;
  }if(whereToMove<300){
    return right;
  }if(whereToMove>700){
    return left;
  }
}

void oneDirection(){
 way = determineMovement();

  if(way == forward){
        Serial.println("  FORWARD");
        return;
  }if(way == right){
        Stop();
        turnRight();
        Serial.println("  RIGHT");
        return;
  }if(way == left){
        Serial.println("  LEFT");
        Stop();
        turnLeft();
        return;
  }
}

void moveForward(){
  analogWrite(motor1pin1,forwardSpeed);
  analogWrite(motor1pin2,0);

  analogWrite(motor2pin1,0);
  analogWrite(motor2pin2,forwardSpeed);
}

void Stop(){
  analogWrite(motor1pin1,0);
  analogWrite(motor1pin2,0);

  analogWrite(motor2pin1,0);
  analogWrite(motor2pin2,0);
}

//functiile de intoarcere stanga/dreapta sunt construite recursiv pentru a nu fi nevoie parcurgerea intregii functii principale de prea multe ori
void turnLeft(){
  analogWrite(motor2pin2,turningSpeed);
  digitalWrite(ledRosuStanga,HIGH);
  if(way == left){
    way = determineMovement();
    digitalWrite(ledRosuStanga,LOW);
    turnLeft();
    }
  digitalWrite(ledRosuStanga,LOW);
}
void turnRight(){
  analogWrite(motor1pin1,turningSpeed);
  digitalWrite(ledRosuDreapta,HIGH);
  if(way == right){
    way = determineMovement();
    digitalWrite(ledRosuDreapta,LOW);
    turnRight();
  }
  digitalWrite(ledRosuDreapta,LOW);
}
