#include <SoftwareSerial.h> // libreria para comunicacion con el bluetooth
#include <NewPing.h> // libreria para manejo de sensores ultrasonicos

/*
  Arduino ejecuta el codigo en cascada
  aca se inicializan dos variables que mas adelante permitiran
  hacer un proceso en paralelo, ya que al ser lineal tiende a haber errores
  en la ejecucion debido a los delays.
*/
unsigned long tiempo1 = 0;
unsigned long tiempo2 = 0;

// Sensores ultrasonicos
/*
  Sensor delantero
*/
int trigPin = A0;
int echoPin = A1;
/*
  Sensor trasero
*/
int trigPinBack = A2;
int echoPinBack = A3;
// Fin de Sensores ultrasonicos

// Pines del motor 1
int power_M1 = 5;
int input1_M1 = 2;
int input2_M1 = 3;

// Pines del motor 2
int power_M2 = 6;
int input1_M2 = 8;
int input2_M2 = 9;

// Velocidad de rotacion de los motores
int maxSpeed = 255;
int midSpeed = 230;

// Leds
int whiteLed = 12;
int greenLed = 11;
int blueLed = 4;
int redLed = 10;
int yellowLed = 7;

// Variables para almacenar el calculo de distancia
int distanceF = 0; // frontal
int distanceB = 0; // trasero

// Inicializacion del Objeto de manejo de Bluetooth
SoftwareSerial bt(0,1); // bluetooth pin 0 (RX ard -> TX bt) pin 1 (TX ard -> RX bt)

// Inicializacion de los Objetos de manejo de sensores ultrasonicos
NewPing sonarFront(trigPin, echoPin, 200); 
NewPing sonarBack(trigPinBack, echoPinBack, 200); 

/*
  Setup del dispositivo, se ejecuta solo una vez, al inicio de ejecucion
*/
void setup(){
  //Inicializar puerto serial velocidad de transmision 9600 baudios
  // baudios = bits/segundos
  Serial.begin(9600);
  //Inicializar comunicacion de modulo bluetooth con arduino velocidad de transmision 9600 baudios
  // baudios = bits/segundos
  bt.begin(9600);

  tiempo1 = millis(); // tiempo en milisegundos de ejecucion del programa
  /*
    Este tiempo se resetea cuando se desconecta el arduino o despues de 50
    dias de estar encendido
  */
  
  // pines de energia a los motores
  pinMode(power_M1,OUTPUT);
  pinMode(power_M2,OUTPUT);
  
  // pines de entrada de senal al motor 1
  pinMode(input1_M1,OUTPUT);
  pinMode(input2_M1,OUTPUT);
  
  // pines de entrada de senal al motor 2
  pinMode(input1_M2,OUTPUT);
  pinMode(input2_M2,OUTPUT);
  // Sensor delantero
  pinMode(trigPin,OUTPUT); // pin trigger salida de senal
  pinMode(echoPin,INPUT); // pin echo entrada de sonido
  // sensor trasero
  pinMode(trigPinBack,OUTPUT); // pin trigger salida de senal
  pinMode(echoPinBack,INPUT); // pin echo entrada de sonido
  // Leds
  pinMode(whiteLed,OUTPUT);
  pinMode(greenLed,OUTPUT);
  pinMode(blueLed,OUTPUT);
  pinMode(redLed,OUTPUT);
  pinMode(yellowLed,OUTPUT);
}

/*
  Loop del dispositivo, se ejecuta en ciclo
*/
void loop(){

  tiempo2 = millis(); // tiempo de calculo al inicializar el ciclo del loop
  /*
    El siguiente bloque es una tarea que se ejecuta en paralelo
    que permite no interferir con el flujo principal del programa
  */
  if(tiempo2 > (tiempo1+100)){  //cada 1 segundo ejecuta el IF
    tiempo1 = millis(); //Actualiza el tiempo actual
    distanceF =  readPingF(); // metodo lectura de distancia adelante
    
    delay(80);
    distanceB =  readPingB(); // metodo lectura de distancia atras
  }
  /*Inicializacion de led apagados*/
  digitalWrite(whiteLed, LOW);
  digitalWrite(greenLed, LOW);
  digitalWrite(blueLed, LOW);
  digitalWrite(redLed, LOW);
  digitalWrite(yellowLed, LOW);

  if ( bt.available() ){ // si la senal de bluetooth esta disponible
    char c = bt.read(); // lee el caracter que envia la app
    
    if(distanceF >= 15 && distanceB >= 25){ 
      // si la distancia sensada adelante es mayor que 15 cm
      // y la distancia trasera es mayor que 25 cm
      controlCommand(c,maxSpeed); // se envia el caracter que envia la app a este metodo para mover el carro a maxima velocidad (5v)

    }else if(distanceF < 15 && distanceF >= 10){
      // si distancia delantera es mayor que 10cm o menor que 15cm 
            
      controlCommand(c,midSpeed); // se envia el caracter que envia la app a este metodo para mover el carro a mediana velocidad
      digitalWrite(whiteLed, LOW);
      digitalWrite(greenLed, LOW);
      digitalWrite(blueLed, LOW);
      digitalWrite(redLed, LOW);
      digitalWrite(yellowLed, HIGH); // enciende led amarillo

    }else if(distanceF < 10){
      // si la distancia es menorque 10cm 
      stop();// el auto se detiene
      digitalWrite(whiteLed, LOW);
      digitalWrite(greenLed, LOW);
      digitalWrite(blueLed, LOW);
      digitalWrite(redLed, HIGH); // enciende el led rojo
      digitalWrite(yellowLed, LOW);  
          
    }else if(distanceB < 25 && distanceB >= 20){
      // si distancia trasera es mayor que 20cm o menor que 25cm 
      controlCommand(c,maxSpeed); // se envia el caracter que envia la app a este metodo para mover el carro a maxima velocidad
      digitalWrite(whiteLed, HIGH); // se enciende led blanco
      digitalWrite(greenLed, LOW);
      digitalWrite(blueLed, LOW);
      digitalWrite(redLed, LOW);
      digitalWrite(yellowLed, LOW);
    }else if(distanceB < 20 && distanceB >= 15){
      // si distancia trasera es mayor que 20cm o menor que 25cm 
      controlCommand(c,midSpeed);  // se envia el caracter que envia la app a este metodo para mover el carro a mediana velocidad
      digitalWrite(whiteLed, LOW);
      digitalWrite(greenLed, HIGH); // se enciende led verde
      digitalWrite(blueLed, LOW);
      digitalWrite(redLed, LOW);
      digitalWrite(yellowLed, LOW);
    }else if(distanceB < 15){
      // si la distancia es menorque 10cm 
      stop(); // se detiene
      digitalWrite(whiteLed, LOW);
      digitalWrite(greenLed, LOW);
      digitalWrite(blueLed, HIGH); // enciende led azul
      digitalWrite(redLed, LOW);
      digitalWrite(yellowLed, LOW);      
    }
    
  } // fin de bt available


}// fin del loop

void forward(int speed){ // metodo para ir hacia adelante

  analogWrite(power_M1, speed); // se envia un voltaje al motor 1
  analogWrite(power_M2, speed);  // se envia un voltaje al motor 2
  // la siguiente configuracion pone en 5v las input 1 de el L293D de cada motor
  // lo que hace que gire hacia un lado (adelante)
  digitalWrite(input1_M1, HIGH); 
  digitalWrite(input2_M1, LOW); 
  
  digitalWrite(input1_M2, HIGH); 
  digitalWrite(input2_M2, LOW);
}

void backward(int speed){ // metodo para ir hacia atras
 
  analogWrite(power_M1, speed);
  analogWrite(power_M2, speed);
    // la siguiente configuracion pone en 0v las input 2 de el L293D de cada motor
  // lo que invierte el giro
  digitalWrite(input1_M1, LOW);
  digitalWrite(input2_M1, HIGH);
  
  digitalWrite(input1_M2, LOW);
  digitalWrite(input2_M2, HIGH);
}

void turnLeft(int speed){// metodo para ir hacia izquierda

  analogWrite(power_M1, 0); // se apaga motor izquierdo
  analogWrite(power_M2, speed); // se envia un voltaje a motor derecho
  
  digitalWrite(input1_M1, HIGH);
  digitalWrite(input2_M1, LOW);
  
  digitalWrite(input1_M2, HIGH);
  digitalWrite(input2_M2, LOW);
}

void turnRight(int speed){// metodo para ir hacia derecha
 
  analogWrite(power_M1, speed); // se envia un voltaje a motor izquierdo
  analogWrite(power_M2, 0); // se apaga motor derecho
  
  digitalWrite(input1_M1, HIGH);
  digitalWrite(input2_M1, LOW);
  
  digitalWrite(input1_M2, HIGH);
  digitalWrite(input2_M2, LOW);
}

void stop(){ // metodo para detenerse
  // no se envia ningun voltaje a los motores
  digitalWrite(input1_M1, LOW);
  digitalWrite(input2_M1, LOW);
  
  digitalWrite(input1_M2, LOW);
  digitalWrite(input2_M2, LOW);
}

void controlCommand(int command , int speed) {
  
  // segun el caracter que envie la app
  switch(command){
  	case 'F': return forward(speed); // F boton arriba
    case 'B': return backward(speed); // B boton abajo
    case 'R': return turnRight(speed); // R boton derecha
    case 'L': return turnLeft(speed); // L boton izquierda
    case 'S': return stop(); // S por defecto, ningun boton
  }
}

int readPingF() { 
  int cmF = sonarFront.ping_cm(); // lectura de sensor delantero
  return cmF;
}
int readPingB() { 
  int cmB = sonarBack.ping_cm(); // lectura de sensor trasero
  return cmB;
}


