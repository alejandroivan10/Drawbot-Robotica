// version 4 11/06/20 04:33 pm

#include <math.h>
#include <Servo.h>

//CONEXIONES HARDWARE
#define switch_q1 2
#define switch_q2 3
#define dirPin_q2 4
#define stepPin_q2 5
#define dirPin_q1 6
#define stepPin_q1 7
#define pinWork 8
Servo q3;

//VARIABLES HARDWARE
const short Phi[121]   =  { -23, -27, -31, -31, -31, -30, -26, -22, -17, -13, -9, -6, -6, -5, -5, -8, -12, -11, -9, -6, -4, -4, -3, -3, -5, -7, -10, -12, -15, -18, -18, -19, -19, -16, -13, -11, -17, -13, -11, -8, -6, -3, -0, -2, -5, -7, -9, -11, -15, -23, -19, -13, -9, -7, -4, -2,  1,  3, -7, -8, -9, -7, -5, -2,  0,  3,  6,  7,  8,  6,  3,  1, -1, -4, -7, -10, -14, -18, -23,  1,  3,  5, -11, -6, -3,  0,  3,  6,  8, 11, 13, 15, 17, 17, 19, 21, 22, 24, 25, 24, 22, 20, 18, 16, 14, 12,  9,  7,  3,  1, -2, -4, -1,  2,  5,  8, 11, 12, 14, 16, 30};
const short Alpha[121] =  { 57, 52, 47, 49, 51, 53, 59, 64, 69, 75, 81, 86, 85, 83, 82, 77, 72, 68, 71, 75, 79, 78, 77, 76, 72, 68, 65, 62, 58, 54, 55, 56, 58, 62, 65, 68, 52, 56, 59, 63, 66, 70, 73, 69, 65, 62, 58, 54, 49, 35, 40, 48, 53, 56, 60, 63, 67, 71, 47, 47, 48, 52, 55, 59, 63, 67, 70, 70, 70, 66, 62, 59, 55, 51, 47, 42, 38, 33, 26, 55, 55, 56, 30, 36, 40, 45, 49, 53, 57, 61, 64, 68, 71, 65, 69, 73, 74, 74, 75, 72, 68, 65, 61, 58, 54, 50, 46, 42, 38, 36, 35, 33, 38, 42, 47, 51, 55, 56, 57, 58, 30 };
const short q2_Vel_Mat[121] = { 19, 23, 23, 12, 12, 12, 23, 23, 23, 23, 23, 23, 12, 12, 12, 23, 23, 12, 23, 23, 23, 12, 12, 12, 23, 23, 23, 23, 23, 23, 12, 12, 12, 23, 23, 23, 22, 21, 19, 19, 20, 20, 16, 25, 25, 25, 25, 25, 25, 25, 23, 23, 23, 23, 23, 23, 23, 23, 25, 12, 12, 25, 23, 23, 23, 23, 10, 12, 12, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 18, 12, 12, 25, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 12, 12, 12, 24, 23, 23, 23, 23, 23, 23, 23, 23, 23, 12, 12, 12, 23, 23, 23, 23, 23, 12, 12, 12, 16};
const short q1_Vel_Mat[121] = { 20, 44, 44, 12, 12, 12, 44, 44, 44, 44, 44, 44, 12, 12, 12, 44, 44, 12, 44, 44, 44, 12, 12, 12, 44, 44, 44, 44, 44, 44, 12, 12, 12, 44, 44, 44, 13, 45, 46, 46, 46, 46, 45, 37, 39, 39, 41, 40, 41, 28, 44, 44, 44, 44, 44, 44, 44, 44, 24, 12, 12, 28, 44, 44, 44, 44, 42, 12, 12, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 46, 12, 12, 37, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 17, 44, 44, 12, 12, 12, 41, 44, 44, 44, 44, 44, 44, 44, 44, 44, 12, 12, 12, 44, 44, 44, 44, 44, 12, 12, 12, 16};
float q1_Vel = 10, q2_Vel = 10;  // Velocidades °Grados/seg
bool ServoDown[121] = { 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0};
const unsigned int Num_Step_q1=880, Num_Step_q2=880; // Numero de pasos cada 360°
float t_Act = 0, t_Ant_q1 = 0, t_Ant_q2 = 0;  // t_Actual y t_Anterior
float q1_Act = 30.0, q2_Act = 0.0;
float q1_Meta, q2_Meta;
byte puntero = 0;


//VARIABLES AUXILIARES
byte Num_Ele = sizeof(Phi)/sizeof(short); // Numero de Elementos
bool Work_New, Work_Old, STOP; 
float q1_VelStep, q2_VelStep, t_q1stp, t_q2stp; // Velocidades convertidas de rad a #pasos
bool Pin_q1, Pin_q2; //  Pin_q1 = stepPin_q1  y  Pin_q2 = stepPin_q2
float q1_Actstp, q2_Actstp, q1_Metastp, q2_Metastp; 
   // q1_Actstp ==> Valor Actual q1      q1_Metastp ==> Valor Meta (deseado en q1) en Pasos
bool q1_Asc = 0, q2_Asc;
short phi, alpha;
bool ServoDown_Ant;

//FUNCIONES DE INTERRUPCIÓN
void eventoQ1(){
  Serial.println("eventoQ1");
  q1_Vel = 0.0001;
  STOP = 1;
}
void eventoQ2(){
  Serial.println("eventoQ2");
  q2_Vel = 0.0001;
  STOP = 1;
}

void setup() {
  Serial.begin(9600);

  pinMode(dirPin_q1, OUTPUT);
  pinMode(stepPin_q1, OUTPUT);
  pinMode(dirPin_q2, OUTPUT); 
  pinMode(stepPin_q2, OUTPUT);
  
  q3.attach(10);
  q3.write(100); //Posicion de NO Rayar
  ServoDown_Ant = 0; // Posicion Servo Anterior, o bien [n-1]
  delay(500);
  pinMode(switch_q1, INPUT_PULLUP); 
  pinMode(switch_q2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(switch_q1),eventoQ1,RISING);
  attachInterrupt(digitalPinToInterrupt(switch_q2),eventoQ2,RISING);

  q1_Actstp = q1_Act * float(Num_Step_q1) / 360.0 ;
  q2_Actstp = q2_Act * float(Num_Step_q2) / 360.0 ;
  q1_Metastp = q1_Actstp; 
  q2_Metastp = q2_Actstp;
    if(q1_Metastp - q1_Actstp >= 0) {q1_Asc = 1;}    
    else{q1_Asc = 0;}
    if(q2_Metastp - q2_Actstp >= 0) {q2_Asc = 1;}
    else{q2_Asc = 0;}
  STOP = 0;
  Serial.println("EMPIEZA");
}

  
void loop() {
  t_Act = float(micros())/1000000.0;
  
  Work_New = digitalRead(pinWork); 
  if(Work_New == 1 && Work_Old == 0){
    //STOP = 0

    // BUSCAR HOME
  }
  Work_Old = Work_New;
   
  if(STOP == 0){
    if( digitalRead(pinWork) ){
      STOP = 1;
      Serial.println("STOP");
    }    
    //Obtenemos la mitad del tiempo necesario para girar un PASO en el motor ==> t_q1stp
    q1_VelStep = q1_Vel*float(Num_Step_q1)/360.0; 
    t_q1stp = 1/(2*q1_VelStep);
    q2_VelStep = q2_Vel*float(Num_Step_q2)/360.0;
    t_q2stp = 1/(2*q2_VelStep);

    bool q1_ready = (q1_Actstp >= q1_Metastp && q1_Asc) || (q1_Actstp <= q1_Metastp && q1_Asc == 0); 
    bool q2_ready = (q2_Actstp >= q2_Metastp && q2_Asc) || (q2_Actstp <= q2_Metastp && q2_Asc == 0);
    
    if(t_Act-t_Ant_q1 > t_q1stp && q1_ready == 0){
      t_Ant_q1 = t_Act;
      Pin_q1 = !Pin_q1;
      if(q1_Asc == 1){  // Si Asc == 1, entonces el q1[n] sera más grande que q1[n-1]
        q1_Actstp += q1_VelStep*t_q1stp;   //// ==> Podría haber errores
        digitalWrite(dirPin_q1, HIGH);         //// o imprecisiones debido al acarreo
        digitalWrite(stepPin_q1, Pin_q1);       /// del error tras cada iteración
      }else{
        q1_Actstp -= q1_VelStep*t_q1stp;
        digitalWrite(dirPin_q1, LOW);
        digitalWrite(stepPin_q1, Pin_q1);
      }      
    }  
    if (t_Act-t_Ant_q2 > t_q2stp && q2_ready == 0){
      t_Ant_q2 = t_Act;
      Pin_q2 = !Pin_q2;
      if(q2_Asc == 1){
        q2_Actstp += q2_VelStep*t_q2stp;
        digitalWrite(dirPin_q2, LOW);
        digitalWrite(stepPin_q2, Pin_q2);
      }else{
        q2_Actstp -= q2_VelStep*t_q2stp;
        digitalWrite(dirPin_q2, HIGH);
        digitalWrite(stepPin_q2, Pin_q2);
      }
    }
    
    if( q1_ready && q2_ready ){
      if(puntero >= Num_Ele){
        STOP = 1;
        Serial.println("STOP");
      }else{        
        q1_Vel = float(q1_Vel_Mat[puntero])/2.0;
        q2_Vel = float(q2_Vel_Mat[puntero])/2.0;
        
        if( ServoDown[puntero] == 1 ) {q3.write(112);}
        else  {q3.write(100);}
        if( ServoDown[puntero] != ServoDown_Ant) {delay(1000);}
        ServoDown_Ant = ServoDown[puntero];
        
        phi = Phi[puntero];
        alpha = Alpha[puntero];
        q1_Meta = float(phi);
        q2_Meta = float(phi - alpha);
        Serial.print(" Phi "); Serial.print(phi); Serial.print("   Alpha "); Serial.print(alpha); Serial.print("   puntero:"); Serial.println(puntero);
        Serial.print(" q1_Vel "); Serial.print(q1_Vel); Serial.print("   q2_Vel "); Serial.print(q2_Vel); Serial.println("");
        q1_Metastp = q1_Meta * float(Num_Step_q1) / 360.0;
        q2_Metastp = q2_Meta * float(Num_Step_q2) / 360.0;
        if(q1_Metastp - q1_Actstp >= 0) {q1_Asc = 1;}
        else{q1_Asc = 0;}
        if(q2_Metastp - q2_Actstp >= 0) {q2_Asc = 1;}
        else{q2_Asc = 0;}
        Serial.print("Tiempo_transcurrido:  "); Serial.println(t_Act);Serial.println();
      }
      puntero += 1;
    }
  }
  
  // CODIGO PARA PRUEBAS DE FUNCIONAMIENTO
      if(Serial.available()){
      int c,x;
      c=Serial.parseInt();
      if( c > 1){
        x=c;
        q3.write(x);
        Serial.println(x);
      }    
    }
      //  unsigned int i;
      //  int val_act = millis();
      //  int val_ant;
      //  
      //  i++;
      //  val_act = millis();
      //  if(val_act - val_ant > 500)  {
      //    Serial.println(i);
      //    val_ant = val_act;
      //  }
}
