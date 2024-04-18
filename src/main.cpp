#include <Arduino.h>
#include <SoftwareSerial.h>

/*Definimos los pines que salen a los motores, los de potencia estableceran la velocidad de los motores
  y los pines A y B determinaran la direccion de movimiento de los motores
*/
#define PIN_POTENCIA_MOTOR1 5
#define PIN_POTENCIA_MOTOR2 6
#define PIN_A_MOTOR1 7
#define PIN_B_MOTOR1 8
#define PIN_A_MOTOR2 9
#define PIN_B_MOTOR2 10

//Definicion de pines de entrada, Estos son provicionales
#define PIN_ADELANTE_MOTOR1 A0
#define PIN_ATRAS_MOTOR1 A1
#define PIN_ADELANTE_MOTOR2 A2
#define PIN_ATRAS_MOTOR2 A3
#define PIN_POTENCIOMETRO A4

float potencia1 = 0;
float potencia2 = 0;
bool adelante1 = false;
bool atras1 = false;
bool adelante2 = false;
bool atras2 = false;

void Movimiento(bool adelante1, bool atras1, float potencia1, bool adelante2, bool atras2, float potencia2);


void setup() {

  Serial.begin(9600);

  //iniliazamos los pines de los motores
  pinMode(PIN_A_MOTOR1, OUTPUT);
  pinMode(PIN_B_MOTOR1, OUTPUT);
  pinMode(PIN_A_MOTOR2, OUTPUT);
  pinMode(PIN_B_MOTOR2, OUTPUT);
  pinMode(PIN_POTENCIA_MOTOR1, OUTPUT);
  pinMode(PIN_POTENCIA_MOTOR2, OUTPUT);

  pinMode(PIN_ADELANTE_MOTOR1, INPUT_PULLUP);
  pinMode(PIN_ATRAS_MOTOR1, INPUT_PULLUP);
  pinMode(PIN_ADELANTE_MOTOR2, INPUT_PULLUP);
  pinMode(PIN_ATRAS_MOTOR2, INPUT_PULLUP);
  pinMode(PIN_POTENCIOMETRO, INPUT);


}

void loop() {

  adelante1 = (digitalRead(PIN_ADELANTE_MOTOR1) == LOW && digitalRead(PIN_ATRAS_MOTOR1) == HIGH);
  atras1 = (digitalRead(PIN_ADELANTE_MOTOR1) == HIGH && digitalRead(PIN_ATRAS_MOTOR1) == LOW);
  adelante2 = (digitalRead(PIN_ADELANTE_MOTOR2) == LOW && digitalRead(PIN_ATRAS_MOTOR2) == HIGH);
  atras2 = (digitalRead(PIN_ADELANTE_MOTOR2) == HIGH && digitalRead(PIN_ATRAS_MOTOR2) == LOW);

  potencia1 = analogRead(PIN_POTENCIOMETRO);
  potencia1 = map(potencia1, 0, 1023, 110, 255);
  potencia2 = potencia1;

  Serial.print("Potencia: ");
  Serial.println(potencia1);

  Movimiento(adelante1, atras1, potencia1, adelante2, atras2, potencia2);
  
}


//Esta funcion determina la direccion y velocidad de movimiento de los motores
void Movimiento(bool adelante1, bool atras1, float potencia1, bool adelante2, bool atras2, float potencia2){

  analogWrite(PIN_POTENCIA_MOTOR1, potencia1);
  analogWrite(PIN_POTENCIA_MOTOR2, potencia2);
  
    if (adelante1)
  {
    Serial.println("Motor 1 adelante");
    digitalWrite(PIN_A_MOTOR1, HIGH);
    digitalWrite(PIN_B_MOTOR1, LOW);
  }
  else if (atras1)
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

  if (adelante2)
  {
    Serial.println("Motor 2 adelante");
    digitalWrite(PIN_A_MOTOR2, HIGH);
    digitalWrite(PIN_B_MOTOR2, LOW);
  }
  else if (atras2)
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
