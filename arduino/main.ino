/******************************************************************
 * PID Simple Example (Augmented with Processing.org Communication)
 * Version 0.3
 * by Brett Beauregard
 * License: Creative-Commons Attribution Share-Alike
 * April 2011
 ******************************************************************/

#include <PID_v1.h>


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// INICIO GLOBAL

//Define Variables we'll be connecting to
double Setpoint_A=0, Input_A=0, Output_A;


//int Setpoint_A_Processing=0; 
double Setpoint_A_Processing=1.0; 


//Specify the links and initial tuning parameters
PID myPID_A(&Input_A, &Output_A, &Setpoint_A,1,5,0, DIRECT);


int Out_Mot_A=0;

const int channelPinA =2 ;
int ENA = 3;    // ENA conectada al pin 3 de Arduino
int IN1 = 4;
int IN2 = 5;
int IN3 = 6;
int IN4 = 7;


long time_reg=0;


//timeCounterA_act
//timeCounterA_last 

const int inicio=128;
const int timeThresholdA = 10;
long timeCounterA_act = 0;
long timeCounterA_last = 0;
long maxStepsA = 100000;
long ISRCounterA = 0;
bool flag_upd_cA = LOW;
int counterA = 0;


const int rpm_max=100;
long rpm_timeCounter = 0;
const int rpm_timeThreshold = 1000;
float millisxmin = 60000;
float pulsos_rev =50;// 1200;
int dir_A=1;
float rpm_A=0;
float rpm_A_int=0;
int muestra_A=0;

long timer_ros=0, timer_rpm_act=0, timer_rpm_last=0;

//int TFA[]={0,80,81,81,81,82,82,82,83,83,83,84,84,85,85,86,86,87,87,88,89,89,90,90,91,91,92,93,94,95,96,97,98,99,100,102,104,106,107,108,109,110,111,112,113,113,114,116,118,120,122,123,124,125,126,127,128,129,130,131,131,132,132,133,133,134,135,136,137,138,139,140,142,143,144,146,150,154,158,160,162,164,166,168,170,173,176,179,182,186,190,192,193,194,195,196,197,201,206,211,216,222,226,230,236,240,252,253,254,255};
int TFA[]={0,39,55,68,79,88,97,105,112,120,126,132,138,146,154,162,168,174,177,179,182,188,193,199};



double Auxiliar_PWM= 0;

// array 110 posiciones 0 a 109

int vel=0;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// FIN GLOBAL

unsigned long serialTime; //this will help us know when to talk with processing

void setup()
{
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// INICIO SETUP

  // initialize serial communication at 9600 bits per second:
 Serial.begin(9600); 
 pinMode (ENA, OUTPUT); 
 pinMode (IN1, OUTPUT);
 pinMode (IN2, OUTPUT);
 pinMode (IN3, OUTPUT);
 pinMode (IN4, OUTPUT);
 pinMode(channelPinA, INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(channelPinA), doEncodeA, RISING);

  //initialize the variables we're linked to
//  Input = analogRead(0);
//  Setpoint = 100;

  //turn the PID on
  myPID_A.SetMode(AUTOMATIC);
  myPID_A.SetOutputLimits(0,23);

  //Preparamos la salida para que el motor gire en un sentido

//   velocidad atras
//  digitalWrite (IN1, HIGH);
//  digitalWrite (IN2, LOW);
//  digitalWrite (IN3, HIGH);
//  digitalWrite (IN4, LOW);
//velocidad adelante
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, HIGH);
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, HIGH);
  
  vel=50;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// FIN SETUP


}

void loop()
{
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// INICIO LOOP
   
 //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
   
//verificamos sentido de giro segun setpoint ROS
if (Setpoint_A_Processing < 0) {
    Setpoint_A= -Setpoint_A_Processing;
    digitalWrite (IN2, LOW);
    digitalWrite (IN1, HIGH);
    dir_A=-1;
   } else {
   Setpoint_A= Setpoint_A_Processing; 
    digitalWrite (IN1, HIGH);
    digitalWrite (IN2, LOW);
    dir_A=1;
    }
  
  Input_A=rpm_A ;
  myPID_A.Compute();
  Out_Mot_A=(int)Output_A ;
  Auxiliar_PWM= TFA[Out_Mot_A];
  analogWrite(ENA, TFA[Out_Mot_A]);

 if (millis() > timer_rpm_last+100) {
    timer_rpm_act=millis();
    if (counterA > 0) {
      rpm_A= counterA*(millisxmin/(timer_rpm_act-timer_rpm_last))/pulsos_rev;
      timer_rpm_last = timer_rpm_act;
      counterA=0;
    } 
  } 
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// FIN LOOP

void doEncodeA()
{
  if (millis() > timeCounterA_last + timeThresholdA)
    {
      timeCounterA_last=millis(); 
       counterA++;
    }     
}


/********************************************
 * Serial Communication functions / helpers
 ********************************************/


union {                // This Data structure lets
  byte asBytes[24];    // us take the byte array
  float asFloat[6];    // sent from processing and
}                      // easily convert it to a
frontendInput;                   // float array



// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1: 0=Direct, 1=Reverse, else = ? error ?
//  2-5: float setpoint
//  6-9: float input
//  10-13: float output  
//  14-17: float P_Param
//  18-21: float I_Param
//  22-245: float D_Param
void SerialReceive()
{

  // read the bytes sent from Processing
  int index=0;
  byte Auto_Man = -1;
  byte Direct_Reverse = -1;
  while(Serial.available() && index<26)
  {
    if(index==0) Auto_Man = Serial.read();
    else if(index==1) Direct_Reverse = Serial.read();
    else frontendInput.asBytes[index-2] = Serial.read();
    index++;
  } 
  
  // if the information we got was in the correct format, 
  // read it into the system
  if(index==26  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1)) {
    Setpoint_A_Processing=double(frontendInput.asFloat[0]);

     // * only change the output if we are in manual mode.  otherwise we'll get an
     //   output blip, then the controller will overwrite.
     
    if(Auto_Man==0)  {                                      
      Output_A=double(frontendInput.asFloat[2]);      
    }
    
    double p, i, d;                       
    // * read in and set the controller tunings
    p = double(frontendInput.asFloat[3]);           
    i = double(frontendInput.asFloat[4]);  
    d = double(frontendInput.asFloat[5]);
    myPID_A.SetTunings(p, i, d);
    // * set the controller mode
    if(Auto_Man==0) myPID_A.SetMode(MANUAL);
    else myPID_A.SetMode(AUTOMATIC);
    // * set the controller Direction
    if(Direct_Reverse==0) {
      myPID_A.SetControllerDirection(DIRECT);
    } else myPID_A.SetControllerDirection(REVERSE); 
  }
  // * clear any random data from the serial buffer
  Serial.flush();
}

// unlike our tiny microprocessor, the processing app
// has no problem converting strings into floats, so
// we can just send strings.  
void SerialSend()
{
  Serial.print("PID ");
  Serial.print(Setpoint_A);   
  Serial.print(" ");
  Serial.print(Input_A);   
  Serial.print(" ");
  Serial.print(Output_A);   
  Serial.print(" ");
  Serial.print(myPID_A.GetKp());   
  Serial.print(" ");
  Serial.print(myPID_A.GetKi());   
  Serial.print(" ");
  Serial.print(myPID_A.GetKd());   
  Serial.print(" ");
  if(myPID_A.GetMode()==AUTOMATIC) Serial.print("Automatic");
  else Serial.print("Manual");  
  Serial.print(" ");
  if(myPID_A.GetDirection()==DIRECT) Serial.println("Direct");
  else Serial.println("Reverse");
}
