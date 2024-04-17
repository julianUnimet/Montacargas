#include <Arduino.h>
#include <SoftwareSerial.h>

#define PIN_A_MOTOR1 5
#define PIN_B_MOTOR1 6
#define PIN_A_MOTOR2 9
#define PIN_B_MOTOR2 10

#define PIN_ADELANTE_MOTOR1 3
#define PIN_ATRAS_MOTOR1 4
#define PIN_ADELANTE_MOTOR2 7
#define PIN_ATRAS_MOTOR2 8


void setup() {

  Serial.begin(9600);

  pinMode(PIN_A_MOTOR1, OUTPUT);
  pinMode(PIN_B_MOTOR1, OUTPUT);
  pinMode(PIN_A_MOTOR2, OUTPUT);
  pinMode(PIN_B_MOTOR2, OUTPUT);

  pinMode(PIN_ADELANTE_MOTOR1, INPUT_PULLUP);
  pinMode(PIN_ATRAS_MOTOR1, INPUT_PULLUP);
  pinMode(PIN_ADELANTE_MOTOR2, INPUT_PULLUP);
  pinMode(PIN_ATRAS_MOTOR2, INPUT_PULLUP);


}

void loop() {

  
  if (digitalRead(PIN_ADELANTE_MOTOR1) == LOW && digitalRead(PIN_ATRAS_MOTOR1) == HIGH)
  {
    Serial.println("Motor 1 adelante");
    digitalWrite(PIN_A_MOTOR1, HIGH);
    digitalWrite(PIN_B_MOTOR1, LOW);
  }
  else if (digitalRead(PIN_ADELANTE_MOTOR1) == HIGH && digitalRead(PIN_ATRAS_MOTOR1) == LOW)
  {
    Serial.println("Motor 1 atras");
    digitalWrite(PIN_A_MOTOR1, LOW);
    digitalWrite(PIN_B_MOTOR1, HIGH);
  }
  else
  {
    //Serial.println("Motor 1 detenido");
    digitalWrite(PIN_A_MOTOR1, LOW);
    digitalWrite(PIN_B_MOTOR1, LOW);
  }

  if (digitalRead(PIN_ADELANTE_MOTOR2) == LOW && digitalRead(PIN_ATRAS_MOTOR2) == HIGH)
  {
    Serial.println("Motor 2 adelante");
    digitalWrite(PIN_A_MOTOR2, HIGH);
    digitalWrite(PIN_B_MOTOR2, LOW);
  }
  else if (digitalRead(PIN_ADELANTE_MOTOR2) == HIGH && digitalRead(PIN_ATRAS_MOTOR2) == LOW)
  {
    Serial.println("Motor 2 atras");  
    digitalWrite(PIN_A_MOTOR2, LOW);
    digitalWrite(PIN_B_MOTOR2, HIGH);
  }
  else
  {
    //Serial.println("Motor 2 detenido");
    digitalWrite(PIN_A_MOTOR2, LOW);
    digitalWrite(PIN_B_MOTOR2, LOW);
  }
   

}

