#include "QuickPID.h"

// This optional setting causes Encoder to use more optimized code,
// It must be defined before Encoder.h is included.
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

// Pins
#define ENCA 2
#define ENCB 3
#define A1A 6 // capable of PWM
#define A1B 5 // capable of PWM
#define POT A0

// Constants
#define GR 20 // Gear Reduction of the Motor
#define PPR 12 // Pulses per Revolution of the Motor

//Define Variables we'll be connecting to
float Setpoint, Input, Output;

//Specify the links and initial tuning parameters
float consKp = 0.1, consKi = 5, consKd = 0.1;
float aggKp = 0.1, aggKi = 5, aggKd = 0.2;

float gapConstant = 30;

QuickPID myPID(&Input, &Output, &Setpoint);

int num = 10;
float cpsAvg = 0;
float rpmAvg;

// globals
long prevT = 0;
long posPrev = 0;

long currMill = 0;
long prevMill = 0;

float calcCPS(long pos, long currT) {
  // Compute counts per second (cps)
  float deltaT = ((float) (currT - prevT)) / 1.0e6;

  return (pos - posPrev) / deltaT;
}

float calcRPM(float cps) {
  return cps * 60 / (4 * GR * PPR);
}


Encoder myEnc(ENCA, ENCB);

void setup() {
  Serial.begin(9600);

  pinMode(A1A, OUTPUT);
  pinMode(A1B, OUTPUT);
  pinMode(POT, INPUT);

  analogWrite(A1A, 0);
  analogWrite(A1B, 0);

  Setpoint = 0;
  //turn the PID on
  myPID.SetMode(myPID.Control::automatic);
}

void loop() {
  // put your main code here, to run repeatedly:
  long pos = myEnc.read();

  // Compute counts per second (cps)
  long currT = micros();
  currMill = millis();

  float cps = calcCPS(pos, currT);
  float rpm = calcRPM(cps);

  Input = rpm; // Input to the PiD
  float gap = abs(Setpoint - Input); //distance away from setpoint
  if (gap < gapConstant) { //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  } else {
    //we're far from setpoint, use aggressive tuning parameters
    myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  bool complete = myPID.Compute();

  if (complete) {
    if (Setpoint >= 0) {
      analogWrite(A1A, 0);
      analogWrite(A1B, Output);
    }
    else {
      analogWrite(A1A, Output);
      analogWrite(A1B, 0);
    }    
  }

  posPrev = pos;
  prevT = currT;

  cpsAvg = ((num - 1) * cpsAvg + cps) / num;
  rpmAvg = calcRPM(cpsAvg);

  Serial.print(Setpoint);
  Serial.print(" ");
  Serial.println(Input);

  //Serial.println(pos);
  //delay(1);

  if (currMill - prevMill > 5) {
    //Serial.println("1 sec has gone by");
    prevMill = currMill;
    Setpoint = 135 * sin(micros() / 1000000)+ 135;
  }
  Setpoint = map(analogRead(POT),0,1023,0,270);
}
