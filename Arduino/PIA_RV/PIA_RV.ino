#include <Wire.h>
#include "pitches.h"
//#include <MPU6050.h>
#include "MPU6050_6Axis_MotionApps20.h"
//#include <MadgwickAHRS.h>


#define giroOffset 0.25
#define cambioLuz 2
#define buzzerPin 5
#define ledPinR 10
#define ledPinG 9
#define ledPinB 6
#define BUTTON_PIN 2
#define motorPin 12

MPU6050 mpu;
//Madgwick filter;

// Variables para almacenar los valores medidos y mostrados
float lecturaAnterior[3]{0,0,0};
float mostrar[3]{0,0,0};

float roll = 0, pitch = 0, yaw= 0;
float ang_x_prev, ang_y_prev, ang_z_prev;

float dt;
float deltaTime = 0;

long tiempo_prev;
unsigned long lastTime = 0;

bool continuarLoop = true;
bool buzzerEnc = false;
bool boton1 = false;
bool luzRoja = false;
bool luces = true;
bool lucesApagadas = false;
bool motor = false;

int brilloR = 0; // Variable para el brillo del LED
int brilloG = 0; // Variable para el brillo del LED
int brilloB = 0; // Variable para el brillo del LED


unsigned int button_status = 0;

unsigned long tiempoInicio = 0;

float ypr[3];
Quaternion q;
VectorFloat gravity;
uint8_t fifoBuffer[64];
uint8_t devStatus; 
bool dmpReady = false;
uint8_t mpuIntStatus;
uint16_t packetSize;


int melody[] = {
  NOTE_G5,  
  NOTE_C5,
  
  NOTE_FS5,
  0,
  NOTE_G5, 
  NOTE_A5,
  
  NOTE_G5,
  NOTE_E5,
  NOTE_C5,
  NOTE_FS5,
  NOTE_G5, 
  NOTE_A5,

  NOTE_G5,
  NOTE_D5,
  NOTE_G5,
  NOTE_D6,

  NOTE_D6,
  NOTE_CS6,
  NOTE_B5,
  NOTE_CS6,

  NOTE_D6
};

int noteDurations[] = {
  2,2,
  3,8,14,14,
  5,5,5,6,14,14,
  4,4,4,4,
  2,6,14,14,
  1
};

int notaActual =0;
int pauseBetweenNotes =0;
int countNote =0;
bool music = false;

void setup() {
  Serial.begin(9600);

  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    calibrarGiro();

    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  //filter.begin(26);

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
    while (true); // Detener el programa si la conexión falla
  }

  //mpu.CalibrateAccel(10);
  //mpu.CalibrateGyro(10);

  Serial.println("");

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(ledPinR, OUTPUT);
  pinMode(ledPinG, OUTPUT);
  pinMode(ledPinB, OUTPUT);
}

void loop() {

  if(music){
    if(countNote >= pauseBetweenNotes){
      countNote=0;
      int noteDuration = 1000 / noteDurations[notaActual];
      tone(buzzerPin, melody[notaActual], noteDuration);
      pauseBetweenNotes = noteDuration *0.02;

      if(notaActual<21){
        notaActual++;
      }
      else{
        notaActual =0;
      }
    }
    else{
    countNote++;
    }
  }

  // Leer datos del MPU6050
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
  

  ypr[0]*= 180/M_PI;
  ypr[1]*= 180/M_PI;
  ypr[2]*= 180/M_PI;

  
  dt = (millis()-tiempo_prev)/1000.0;
  tiempo_prev=millis();

  //ypr[2] = 0.98*((gx*8.181818/131)*dt) + lecturaAnterior[2] +0.02;
  //ypr[1] = 0.98*((gy*8.181818/131)*dt) + lecturaAnterior[1] +0.02;
  //ypr[0] = 0.98*((gz*8.181818/131)*dt) + lecturaAnterior[0] +0.02;

  //Calcular los ángulos con acelerometro
  //float accel_ang_x=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/PI);
  //float accel_ang_y=atan(-ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/PI);
  //float accel_ang_z=atan(sqrt(pow(ax,2) + pow(ay,2)) /az)*(180.0/PI);
  //float accel_yaw = atan2(-ay, az) * 180 / PI;
  
  //Calcular angulo de rotación con giroscopio y filtro complemento  
  //roll = 0.98*(ang_x_prev+(gx/131)*dt) + 0.02*accel_ang_x;
  //pitch = 0.98*(ang_y_prev+(gy/131)*dt) + 0.02*accel_ang_y;
  //yaw += ((gz/131)*dt)+ 0.02;
  //yaw += ((gz/131)*dt);
  
  //ang_x_prev=roll;
  //ang_y_prev=pitch;
  //ang_z_prev=yaw;

  //float promX = (roll + lecturaAnterior[0]) / 2.0;
  //float promY = (pitch + lecturaAnterior[1]) / 2.0;
  //float promZ = (yaw + lecturaAnterior[2]) / 2.0;

  //filter.updateIMU(gx / 131.0, gy / 131.0, gz / 131.0, ax -100, ay-100, az-100);
  //roll = filter.getRoll();
  //pitch = filter.getPitch();
  //yaw = filter.getYaw() -180;


  //ang_x_prev=roll;

  //ang_y_prev=pitch;

  //ang_z_prev=yaw;
  int aux2 = abs(lecturaAnterior[1] - ypr[1]);
  if(ypr[2] >90 || ypr[2] < -90 && aux2 >40){
    //ypr[1] =lecturaAnterior[1];
  }
  else{
    aux2 = abs(ypr[2] - lecturaAnterior[2]);
    if(ypr[1] >80 || ypr[1] < -80 ){
      //ypr[2] =lecturaAnterior[2];
      //ypr[0] =lecturaAnterior[0];
      if(ypr[2] > lecturaAnterior[2]){
        //ypr[2] -=aux2;
      }
      else{
        if(ypr[2] < lecturaAnterior[2]){
          //ypr[2] +=aux2;
        }
      }
      //ypr[0] +=140;
    }
  }

  

  for(int i = 0; i < 3; i++){
    if((ypr[i] - lecturaAnterior[i] > giroOffset || ypr[i] - lecturaAnterior[i] < -giroOffset)) {
      //mostrar[i] = (ypr[i] + lecturaAnterior[i])/2;
      if(ypr[i] - lecturaAnterior[i] >120){
        //ypr[i]-=ypr[i] - lecturaAnterior[i];
      }
      else{
        if(ypr[i] - lecturaAnterior[i] < -120){
          //ypr[i]+=ypr[i] - lecturaAnterior[i];
        }
      }

      

      mostrar[i] = ypr[i];
      lecturaAnterior[i] = mostrar[i];
    }
  }

  
  unsigned long currentTime = millis();
  deltaTime = (float)(currentTime - lastTime) / 1000.0; // Convertir a segundos
  lastTime = currentTime;

  if(luces){
    if(!luzRoja){
      if (brilloR < (128 - cambioLuz)) {
        brilloR += cambioLuz;
      }
      else{
        luzRoja= true;
      }
    }
    else{
      //Rojo a Verde
      if (brilloR > 0 && brilloG < (128 - cambioLuz)  && brilloB == 0) {
        brilloR -= cambioLuz;
        brilloG += cambioLuz;
      }
      //Verde a Azul
      if (brilloR == 0 && brilloG > 0 && brilloB < (128 - cambioLuz)) {
        brilloG -= cambioLuz;
        brilloB += cambioLuz;
      }
      //Azul a Rojo
      if (brilloR < (128 - cambioLuz) && brilloG == 0 && brilloB > 0) {
        brilloB -= cambioLuz;
        brilloR += cambioLuz;
      }
    }

    analogWrite(ledPinR, brilloR); // Establecer el brillo del LED
    analogWrite(ledPinG, brilloG); // Establecer el brillo del LED
    analogWrite(ledPinB, brilloB); // Establecer el brillo del LED
  }
  else{
    if(!lucesApagadas){
      if(brilloR <= 2 && brilloG <= 2 && brilloB <=2){
        lucesApagadas = true;
        brilloR=0, brilloG=0, brilloB=0;
      }
      else{
        unsigned long tiempoActual = millis();
        brilloR -= ((brilloR * (tiempoActual - tiempoInicio)) / 1500);
        brilloG -= ((brilloG * (tiempoActual - tiempoInicio)) / 1500);
        brilloB -= ((brilloB * (tiempoActual - tiempoInicio)) / 1500);   
      }
      analogWrite(ledPinR, brilloR); // Establecer el brillo del LED
      analogWrite(ledPinG, brilloG); // Establecer el brillo del LED
      analogWrite(ledPinB, brilloB); // Establecer el brillo del LED
    }
  }
  

  button_status = !digitalRead(BUTTON_PIN);
 
  if(boton1 != button_status){
    if (button_status == HIGH) {
      tone(buzzerPin, 1000);
      Serial.println("Boton aplastado");
      digitalWrite(motorPin, 1);
      boton1 = true;
    } else {
      noTone(buzzerPin);
      Serial.println("Boton levantado");
      digitalWrite(motorPin, LOW);
      boton1 = false;
    }
  }
  

  Serial.print("P=");
  Serial.print(mostrar[2]);
  Serial.print(" R=");
  Serial.print(-mostrar[1]);
  Serial.print(" Y=");
  Serial.println(mostrar[0]);
  

  // Verificar si hay datos disponibles en el puerto serie
  if (Serial.available() > 0) {
    // Leer el primer byte recibido
    char entrada = Serial.read();
    
    // Verificar si la entrada recibida coincide con la entrada deseada
    switch(entrada){
      case '0':
        continuarLoop = false;
        motor=false;
        digitalWrite(motorPin, LOW);
        break;

      case '4':
        if(!music){
          notaActual = 0;
          //tone(buzzerPin, 3000); // Activar el buzzer con un tono de 1000Hz
        }
        else{
          noTone(buzzerPin); // Apagar el buzzer
        }
        music=!music;
        break;

      case '5':
        if(luces){
          luces = false;
          luzRoja = false;
          tiempoInicio = millis();
        }
        else{
          luces = true;
          brilloR = 0, brilloG = 0, brilloB = 0;
          lucesApagadas= false;
        }
        break;

      case '6':
        if(!motor){
          digitalWrite(motorPin, 1);
        }
        else{
          digitalWrite(motorPin, LOW);
        }
        motor=!motor;
        break;
    }
  }


  if (!continuarLoop) {
    // Salir del bucle loop()
    //Serial.end();
    while (true) {
      // No hacer nada, el bucle se detendrá después de esta iteración
      if (Serial.available() > 0) {
        
        // Si hay datos disponibles, salimos del bucle de espera y volvemos al loop()
        char entrada = Serial.read();
    
        if (entrada == '1') {
          continuarLoop = true;
          calibrarGiro();
        }
        delay(100);
        //Serial.println("P=");
        break;
      }
    }
  }

  delay(25);
  
}


void calibrarGiro(){
  // Calibration Time: generate offsets and calibrate our MPU6050
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  Serial.println("");
  mpu.PrintActiveOffsets();

  for(int i=0;i<3;i++){
    lecturaAnterior[i] = 0;
    mostrar[i] = 0;
  }
}