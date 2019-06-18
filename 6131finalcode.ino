#include "Adafruit_MAX31855.h";

//LCD Screen Magic
  int BigB = 0;
  int BigLR = 1;
  int BigLL = 2;
  int BigUL = 3;
  int BigT = 4;
  int BigM = 5;
  int BigUR = 6;
  int LittleT = 7;
  int LittleUL = 8;
  int LittleUR = 9;
  int LittleM = 11;
  int LittleLR = 18;
  int sign = 13;
  int LittleLL = 16;
  int LittleB = 15;

// Duty Cycle Output Pins
int dutyPin = A9; // to delay circuit

float V_in = 12.0;

// User Potentiometer - Wiper
int potPin = A8; 
int potVal;


// SPI Thermocouple Read In Pins
int8_t sclk = 14;
int8_t cs = 10;
int8_t miso = 12;

// Thermocouple (K-Type) SPI Read In
Adafruit_MAX31855 thermo = Adafruit_MAX31855(sclk, cs, miso);

//PID variables
float PID;
int timeElapsed;
int lastError_temp = 0;
int lastError_curr = 0;
int t_temp;
float error;
float temperature;
float maxD = 0.9;
float minD = 0.1;
float ratio;
float delta_t;
float V;
int comm;
float errorSum;
int dt = 50;

//Buck Converter Control variables
int Vd; //duty cycle signal
int Vplus; //+ Threshold (74HC14)
int Vminus;

//PID tuned constants 

//Temperature
float Kp_temp = 3;
float Kd_temp = 0;
float Ki_temp = 0.9;//.;h.;/;


//Target constants
int desiredTemp = 5;
int desiredCurr;
int buff; //buffer margin to prevent crossing of totem minimal duty cycle (10% - 90%)

float tempControl(float e){
  errorSum = errorSum + (e*(dt*(0.001)));
  if (errorSum > 2.5){errorSum = 2.5;}
  float p = Kp_temp*e + Kd_temp*(e - lastError_temp) + Ki_temp*errorSum;
  lastError_temp = e;
  return p;
}


//current = PID output from temp error
//delta_t = (27 - POTSET) + error 
float V_func(float delta, float current){
  return 0.0552*delta + 0.8166*current;
  
}

// V to duty cycle to command signal
int V2command(float voltage){
  return int((voltage/V_in) * 255.0);
  
  }

void disp(int desTemp){
  int sig;
  if (desTemp>=0){sig = 1;}else{sig = 0;}
  desTemp = abs(desTemp);
  int topNum = (int) floor((desTemp/10.0));
  int botNum = (int) (desTemp - (10*topNum));

  switch (abs(topNum)) {
    case 0:
      digitalWrite(BigT,HIGH);
      digitalWrite(BigM,LOW);
      digitalWrite(BigB,HIGH);
      digitalWrite(BigUL,HIGH);
      digitalWrite(BigLL,HIGH);
      digitalWrite(BigUR,HIGH);
      digitalWrite(BigLR,HIGH);
      break;
    case 1:
      digitalWrite(BigT,LOW);
      digitalWrite(BigM,LOW);
      digitalWrite(BigB,LOW);
      digitalWrite(BigUL,LOW);
      digitalWrite(BigLL,LOW);
      digitalWrite(BigUR,HIGH);
      digitalWrite(BigLR,HIGH);
      break;
    case 2:
      digitalWrite(BigT,HIGH);
      digitalWrite(BigM,HIGH);
      digitalWrite(BigB,HIGH);
      digitalWrite(BigUL,LOW);
      digitalWrite(BigLL,HIGH);
      digitalWrite(BigUR,HIGH);
      digitalWrite(BigLR,LOW);
      break;
    default:
      digitalWrite(BigT,LOW);
      digitalWrite(BigM,LOW);
      digitalWrite(BigB,LOW);
      digitalWrite(BigUL,LOW);
      digitalWrite(BigLL,LOW);
      digitalWrite(BigUR,LOW);
      digitalWrite(BigLR,LOW);
      break;
  }
    switch (abs(botNum)) {
    case 0:
      digitalWrite(LittleT,HIGH);
      digitalWrite(LittleM,LOW);
      digitalWrite(LittleB,HIGH);
      digitalWrite(LittleUL,HIGH);
      digitalWrite(LittleLL,HIGH);
      digitalWrite(LittleUR,HIGH);
      digitalWrite(LittleLR,HIGH);
      break;
    case 1:
      digitalWrite(LittleT,LOW);
      digitalWrite(LittleM,LOW);
      digitalWrite(LittleB,LOW);
      digitalWrite(LittleUL,LOW);
      digitalWrite(LittleLL,LOW);
      digitalWrite(LittleUR,HIGH);
      digitalWrite(LittleLR,HIGH);
      break;
    case 2:
      digitalWrite(LittleT,HIGH);
      digitalWrite(LittleM,HIGH);
      digitalWrite(LittleB,HIGH);
      digitalWrite(LittleUL,LOW);
      digitalWrite(LittleLL,HIGH);
      digitalWrite(LittleUR,HIGH);
      digitalWrite(LittleLR,LOW);
      break;
    case 3:
      digitalWrite(LittleT,HIGH);
      digitalWrite(LittleM,HIGH);
      digitalWrite(LittleB,HIGH);
      digitalWrite(LittleUL,LOW);
      digitalWrite(LittleLL,LOW);
      digitalWrite(LittleUR,HIGH);
      digitalWrite(LittleLR,HIGH);
      break;
    case 4:
      digitalWrite(LittleT,LOW);
      digitalWrite(LittleM,HIGH);
      digitalWrite(LittleB,LOW);
      digitalWrite(LittleUL,HIGH);
      digitalWrite(LittleLL,LOW);
      digitalWrite(LittleUR,HIGH);
      digitalWrite(LittleLR,HIGH);
      break;
    case 5:
      digitalWrite(LittleT,HIGH);
      digitalWrite(LittleM,HIGH);
      digitalWrite(LittleB,HIGH);
      digitalWrite(LittleUL,HIGH);
      digitalWrite(LittleLL,LOW);
      digitalWrite(LittleUR,LOW);
      digitalWrite(LittleLR,HIGH);
      break;
    case 6:
      digitalWrite(LittleT,HIGH);
      digitalWrite(LittleM,HIGH);
      digitalWrite(LittleB,HIGH);
      digitalWrite(LittleUL,HIGH);
      digitalWrite(LittleLL,HIGH);
      digitalWrite(LittleUR,LOW);
      digitalWrite(LittleLR,HIGH);
      break;
    case 7:
      digitalWrite(LittleT,HIGH);
      digitalWrite(LittleM,LOW);
      digitalWrite(LittleB,LOW);
      digitalWrite(LittleUL,LOW);
      digitalWrite(LittleLL,LOW);
      digitalWrite(LittleUR,HIGH);
      digitalWrite(LittleLR,HIGH);
      break;
    case 8:
      digitalWrite(LittleT,HIGH);
      digitalWrite(LittleM,HIGH);
      digitalWrite(LittleB,HIGH);
      digitalWrite(LittleUL,HIGH);
      digitalWrite(LittleLL,HIGH);
      digitalWrite(LittleUR,HIGH);
      digitalWrite(LittleLR,HIGH);
      break;
    case 9:
      digitalWrite(LittleT,HIGH);
      digitalWrite(LittleM,HIGH);
      digitalWrite(LittleB,LOW);
      digitalWrite(LittleUL,HIGH);
      digitalWrite(LittleLL,LOW);
      digitalWrite(LittleUR,HIGH);
      digitalWrite(LittleLR,HIGH);
      break;
    default:
      digitalWrite(LittleT,LOW);
      digitalWrite(LittleM,LOW);
      digitalWrite(LittleB,LOW);
      digitalWrite(LittleUL,LOW);
      digitalWrite(LittleLL,LOW);
      digitalWrite(LittleUR,LOW);
      digitalWrite(LittleLR,LOW);
      break;
  }
  if (sig == 1){
    digitalWrite(sign,LOW);
  }
  else{digitalWrite(sign,HIGH);}
}

void setup() {
  Serial.begin(9600);
  pinMode(A9, OUTPUT);
  analogWriteFrequency(A9, 25000);
  pinMode(13, OUTPUT);
  thermo.begin();

  pinMode(BigT, OUTPUT);
  pinMode(BigM, OUTPUT);
  pinMode(BigB, OUTPUT);
  pinMode(BigUL, OUTPUT);
  pinMode(BigLL, OUTPUT);
  pinMode(BigUR, OUTPUT);
  pinMode(BigLR, OUTPUT);
  pinMode(LittleT, OUTPUT);
  pinMode(LittleM, OUTPUT);
  pinMode(LittleB, OUTPUT);
  pinMode(LittleUL, OUTPUT);
  pinMode(LittleLL, OUTPUT);
  pinMode(LittleUR, OUTPUT);
  pinMode(LittleLR, OUTPUT);
  pinMode(sign, OUTPUT);
  
}

void loop() {
  potVal = analogRead(potPin);
  //desiredTemp = 15.0;
  desiredTemp = map(potVal, 0, 1023, -10, 20);
  disp(desiredTemp);
  temperature = thermo.readCelsius();
  error = temperature - desiredTemp;
  PID = tempControl(error);
  delta_t = (27.0 - desiredTemp) + error;
  V = V_func(delta_t, PID);
  comm = V2command(V);
  comm = constrain(comm, 0, 250);
  analogWrite(A9, comm);

  //Serial.print("Desired Temperature: ");
  //Serial.println(desiredTemp);
  //Serial.print("PotVal: ");
  //Serial.println(potVal);

  Serial.print("Temperature: ");
  Serial.println(temperature);
  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("PID: ");
  Serial.println(PID);
  Serial.print("delta_t: ");
  Serial.println(delta_t);
  Serial.print("V: ");
  Serial.println(V);
  Serial.print("Comm: ");
  Serial.println(comm);
  delay(dt);
  
    
    
}

