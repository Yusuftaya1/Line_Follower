#include "Arduino.h"
#include <QTRSensors.h>
#include <PID_v1.h>
#include "NewPing.h"
#include <Wire.h>
#include "Adafruit_TCS34725.h"

#define ENA 4
#define ENB 7
#define IN1 6
#define IN2 5
#define IN3 9
#define IN4 8

#define redpin 3
#define greenpin 5
#define bluepin 6

#define TRIG_PIN 3
#define ECHO_PIN 2

#define THRESHOLD 1000

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

NewPing sonar(TRIG_PIN, ECHO_PIN);
bool tunel_flag = false;

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

double Setpoint = 3500, Input, Output;
double Kp = 0.02, Ki = 0.002, Kd = 0.0022; // PID sabitleri

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

bool flag = false;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("AAAAA");

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  myPID.SetOutputLimits(-100, 100);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(25);
}

void printSensorValues(uint16_t sensorValues[], uint8_t size) {
  Serial.print("Sensor Values: ");
  for (uint8_t i = 0; i < size; ++i) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void mission_Mavi() {
  if (flag == false) {
    motorWrite_Left(10);
    motorWrite_Right(80);
    flag = true;
  } else if (flag == true) {
    motorWrite_Left(80);
    motorWrite_Right(10);
  }
}

void mission_sari() {
  if (flag == false) {
    motorWrite_Left(10);
    motorWrite_Right(80);
    flag = true;
  } else if (flag == true) {
    motorWrite_Left(80);
    motorWrite_Right(10);
  }
}

void loop() {
  float color = determineColor();
  // Create NewPing objects for each ultrasonic sensor
  test1();


  float bu_nemk = sonar.ping_cm();

  Serial.println(bu_nemk);
  if ( bu_nemk<50 ){
    motorWrite_Left(64);
    motorWrite_Right(70);

  }
  
  if (color == 1.0) mission_red();
  if(color == 3.0) mission_sari();

  if (color == 2.0) mission_Mavi();
  
  if ((sensorValues[0] < 150) && (sensorValues[1] < 150) && (sensorValues[2] < 150 && sensorValues[5] < 150) && (sensorValues[6] < 150) && (sensorValues[7] < 150)) {
    if ((sensorValues[4] > 700) && (sensorValues[5] > 700)) {
      motorWrite_Left(73);
      motorWrite_Right(80);
    }
  }
}

void mission_red(){
  // Sol motoru geriye ve sağ motoru ileriye döndür
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 125); // Motor hızı 100

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, -150); // Motor hızı 100
  
  delay(1000); // 1 saniye bekleyelim
  
  // Dönüş tamamlandıktan sonra motorları ileri yönde devam ettir
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

}

void test1() {
  //uint16_t position = qtr.readLineBlack(sensorValues);
  uint16_t position = qtr.readLineWhite(sensorValues);
  Input = position;

  Serial.println(position);

  myPID.Compute();

  Serial.println(position);
  motorWrite_Left(100 - Output);
  motorWrite_Right(100 + Output);
}


float determineColor() {
  float red, green, blue;
  
  tcs.setInterrupt(false);  // LED'i aç

  delay(6);  // 50ms'de okunur

  tcs.getRGB(&red, &green, &blue);
  
  tcs.setInterrupt(true);  // LED'i kapat

  // Renk aralıkları
  if (red > 140 && green < 70 && blue < 55) {
    Serial.println("\tKırmızı");
    return 1.0; // Kırmızı için 1.0 döndür

  } else if (blue > 60 && red < 100 && green > 90 && green < 105) {
    Serial.println("\tMavi");
    return 2.0; // Mavi için 2.0 döndür

  } else if (red > 100 && red < 120 && green > 80 && green < 90 && blue > 30 && blue < 50) {
    Serial.println("\tSarı");
    return 3.0; // Sarı için 3.0 döndür

  } else {
    Serial.println("\tDiğer");
    return 0.0; // Diğer renkler için 0.0 döndür
  }
}

void motorWrite_Right(int duty) {
  if (duty >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, abs(duty)); // Duty cycle için mutlak değer kullanıyoruz
}

void motorWrite_Left(int duty) {
  if (duty >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  analogWrite(ENB, abs(duty)); // Duty cycle için mutlak değer kullanıyoruz
}