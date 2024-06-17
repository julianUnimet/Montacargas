#include <Arduino.h>
#include <SoftwareSerial.h>

//Se definen los pines del modulo bluetooth. Provicionales, al final seran el 0 y 1
#define RX 11
#define TX 12

/*Definimos los pines que salen a los motores, los de potencia estableceran la velocidad de los motores
  y los pines A y B determinaran la direccion de movimiento de los motores
*/
#define PIN_POTENCIA_MOTOR1 5
#define PIN_POTENCIA_MOTOR2 6
#define PIN_A_MOTOR1 7
#define PIN_B_MOTOR1 8
#define PIN_A_MOTOR2 9
#define PIN_B_MOTOR2 10

//Definicion de pines del stepper motor
#define PIN_ELEVAR A0
#define PIN_BAJAR A1
#define PIN_FIN_CARRERA_ARRIBA A2
#define PIN_FIN_CARRERA_ABAJO A3

SoftwareSerial miBT(RX, TX); // RX, TX

//Variables para el control de los motores, estas variables se actualizan con los valores que llegan del bluetooth
float potencia1 = 0.0f;
float potencia2 = 0.0f;
bool adelante1 = false;
bool atras1 = false;
bool adelante2 = false;
bool atras2 = false;
byte direccionElevacion = 0;

/*Valores que llegan del bluetooth
    0: 255 indica el comienzo de la secuencia de envio
    1: Dirección del motor 1
    2: Velocidad del motor 1
    3: Dirección del motor 2
    4: Velocidad del motor 2
    5: Direccion de elevacion del mastil
*/
byte recepcionBluetooth[6] = {0, 0, 0, 0, 0, 0};
byte contadorRecepcion = 0; //Contador de la secuencia de recepcion

void recibirDatosBluetooth(); //Funcion que recibe los datos que llegan del bluetooth
void interpretarDatosBluetooth(); //Funcion que interpreta los datos que llegan del bluetooth
void Movimiento(bool adelante1, bool atras1, float potencia1, bool adelante2, bool atras2, float potencia2); //Funcion que determina la direccion y velocidad de movimiento de los motores
void controlElevacion(byte direccionElevacion); //Funcion que controla la elevacion del mastil


void setup() {

  Serial.begin(9600);
  miBT.begin(38400);

  //iniliazamos los pines de los motores
  pinMode(PIN_A_MOTOR1, OUTPUT);
  pinMode(PIN_B_MOTOR1, OUTPUT);
  pinMode(PIN_A_MOTOR2, OUTPUT);
  pinMode(PIN_B_MOTOR2, OUTPUT);
  pinMode(PIN_POTENCIA_MOTOR1, OUTPUT);
  pinMode(PIN_POTENCIA_MOTOR2, OUTPUT);
  pinMode(PIN_ELEVAR, OUTPUT);
  pinMode(PIN_BAJAR, OUTPUT);
  pinMode(PIN_FIN_CARRERA_ARRIBA, INPUT_PULLUP);
  pinMode(PIN_FIN_CARRERA_ABAJO, INPUT_PULLUP);



}

void loop() {


  Serial.print("Potencia: ");
  Serial.println(potencia1);
  
  recibirDatosBluetooth();
  interpretarDatosBluetooth();
  Movimiento(adelante1, atras1, potencia1, adelante2, atras2, potencia2);
  controlElevacion(direccionElevacion);
  
}

//Esta funcion recibe los datos que llegan del bluetooth
void recibirDatosBluetooth()
{
  if (miBT.available())
  {
    int dato = miBT.read();
    
    if (dato == 255)
    {
      contadorRecepcion = 0;
    }
    recepcionBluetooth[contadorRecepcion] = dato;
    contadorRecepcion++;
  }
}

//Esta funcion interpreta los datos que llegan del bluetooth
void interpretarDatosBluetooth()
{
  if (recepcionBluetooth[0] == 255)
  {
    adelante1 = (recepcionBluetooth[1] == 1);
    atras1 = (recepcionBluetooth[1] == 2);
    potencia1 = recepcionBluetooth[2]; // Revizar si es necesario hacer un map

    adelante2 = (recepcionBluetooth[3] == 1);
    atras2 = (recepcionBluetooth[3] == 2);
    potencia2 = recepcionBluetooth[4]; // Revizar si es necesario hacer un map

    direccionElevacion = recepcionBluetooth[5];
  }
}

//Esta funcion determina la direccion y velocidad de movimiento de los motores
//Nunca colocar en HIGH los dos pines de direccion de un motor al mismo tiempo, produce un corto circuito
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

//Esta funcion controla la elevacion del mastil
void controlElevacion(byte direccionElevacion){
  switch (direccionElevacion)
  {
  case 1:
    Serial.println("Elevacion arriba");
    if(digitalRead(PIN_FIN_CARRERA_ARRIBA) == LOW){
      digitalWrite(PIN_ELEVAR, LOW);
      break;
    }
    digitalWrite(PIN_ELEVAR, HIGH);
    digitalWrite(PIN_BAJAR, LOW);
    break;
  case 2:
    Serial.println("Elevacion abajo");
    if(digitalRead(PIN_FIN_CARRERA_ABAJO) == LOW){
      digitalWrite(PIN_BAJAR, LOW);
      break;
    }
    digitalWrite(PIN_ELEVAR, LOW);
    digitalWrite(PIN_BAJAR, HIGH);
    break;
  default:
    Serial.println("Elevacion detenida");
    digitalWrite(PIN_ELEVAR, LOW);
    digitalWrite(PIN_BAJAR, LOW);
    break;
  }
}
