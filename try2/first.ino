#include <Arduino.h>
#include <QTRSensors.h>
#include <PID_v1.h>

#define ENA 4
#define ENB 7
#define IN1 6
#define IN2 5
#define IN3 9
#define IN4 8

#define THRESHOLD 1000

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

double Setpoint = 3500, Input, Output;
double Kp = 0.01, Ki = 0.001, Kd = 0.002; // 0.02 0.005
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void motorWrite_Right(int duty)
{
  if (duty >= 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, duty);
}

void motorWrite_Left(int duty)
{
  if (duty >= 0)
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  analogWrite(ENB, duty);
}

void setup()
{
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  myPID.SetSampleTime(10);

  Serial.begin(9600);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7, A6, A5, A4, A3, A2, A1, A0}, SensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < 200; i++)
  {

    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  myPID.SetOutputLimits(-100,100);
  myPID.SetMode(AUTOMATIC);
}



void test1()
{
  // uint16_t position = qtr.readLineBlack(sensorValues);
  uint16_t position = qtr.readLineBlack(sensorValues);
  Input = position;

  Serial.println(position);

  myPID.Compute();
  Serial.println(position);
  motorWrite_Left(100 - Output);
  motorWrite_Right(100 + Output);
}

void loop()
{
  test1();
  
  // uint16_t position = qtr.readLineBlack(sensorValues);

  // Serial.println(position);
  // delay(100);
  // motorWrite_Left(0);
  // motorWrite_Right(0);
}
