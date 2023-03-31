#include <Wire.h>
#include <Arduino.h>




#define stepAng           10      // step angle
#define numStep           18        // = 180/stepAng
#define averagedivider    2
 
#define MIN_MICROS        1000  
#define MAX_MICROS        2000

#define minreverse         500    
#define fullreverse        150  

#define turnrange         2000

#define SERVOLIDAR           7
#define MOTOR               6     
#define RUDDER              2
#define MODE                3
#define PUMP                4


#include <VL53L1X.h>
VL53L1X sensor;


#define ISR_SERVO_DEBUG             0
#define USE_ESP32_TIMER_NO          2
#include "ESP32_S2_ISR_Servo.h"


#include <IBusBM.h>
IBusBM IBus;    // IBus object


#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 4800;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
unsigned long last = 0UL;



#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
Adafruit_MPU6050 mpu;


#include <PID_v1.h>


//Lidar PID LOOP
double LIDARSETPOINT, LIDARPIDINPUT, LIDARPIDOUTPUT;
double Lp=2, Li=5, Ld=1;
PID LIDARPID(&LIDARPIDINPUT, &LIDARPIDOUTPUT, &LIDARSETPOINT, Lp, Li, Ld, DIRECT);



//GYRO PID LOOP
double GYROSETPOINT, GYROPIDINPUT, GYROPIDOUTPUT;
double Gp=2, Gi=5, Gd=1;
PID GYROPID(&GYROPIDINPUT, &GYROPIDOUTPUT, &GYROSETPOINT, Gp, Gi, Gd, DIRECT);






int oldval;
int newval;
int intval;
int oltval;

int avoidmode = 0;
int scanhold = 0;

int yawsmoothen = 0;
int escsmoothen = 0;
int pointyawen = 0;
int pointescen = 0;
int wallsteeren = 0;
int followturnen = 0;

int yawfollow;
int yawsmooth = 1500;
int escsmooth = 1500;


//RC inputs   

int ch1 = 1500;  //throttle
int ch2 = 1500;  //rudder
int ch3 = 1500;  //camera yaw
int ch4 = 1500;  //lights
int ch5 = 1500;  //anchor winch
int ch6 = 1500;  //tow winch
int ch7 = 1500;  //bilge pump / fire monitor
int ch8 = 1500;  //flight mode
int ch9 = 1500;  //avoid gain
int ch10 = 1350; //avoid mode
int ch11 = 1350; //avoid mode
int ch12 = 1350; //avoid mode
int ch13 = 1350; //avoid mode
int ch14 = 1350; //avoid mode



int outch1 = 1500;  //throttle
int outch2 = 1500;  //rudder
int outch3 = 1500;  //camera yaw
int outch4 = 1500;  //lights
int outch5 = 1500;  //anchor winch
int outch6 = 1500;  //tow winch
int outch7 = 1500;  //bilge pump / fire monitor
int outch8 = 1500;  //flight mode
int outch9 = 1500;  //avoid gain
int outch10 = 1350; //avoid mode
int outch11 = 1350; //avoid mode
int outch12 = 1350; //avoid mode
int outch13 = 1350; //avoid mode
int outch14 = 1350; //avoid mode

int gyroz;
uint32_t scanner  = -1;
uint32_t MOTout  = -1;
uint32_t RUDout  = -1;
uint32_t MODout  = -1;
uint32_t pumpout  = -1;


int MOT;
int RUD;
int MOD;
int PUM;


//GPIO

int bilge;

//servo values

int RCRud = 1500;
int RCThr = 1500;
int AVOIDMODE = 1500;
int MULTI = 1500; 
int flightmode;

int esc;
int pointesc;
int escs;
int yaw;
int pointyaw;
int out = 1500;
int rudout = 1500;

int throttlesmooth = 1500;

int turnmulti;
int lmix;
int rmix;
int rudmix;
const int numReadings = 10;
int readings[numReadings];  // the readings from the analog input
int readIndex = 0;          // the index of the current reading
int total = 0;              // the running total
int average = 0;            // the average

int averagescale;
int closest;
int avoiddirection;
int avoidturn;

int followturn;

int leftwallaverage;
int rightwallaverage;
int wallsteer;


int pos = 0;          // servo position
int dir = 1;          // servo moving direction: +1/-1
int val;              // LiDAR measured value
int range;
float distances[numStep+1]; // array for the distance at each angle, including at 0 and 180
float leftSum, rightSum, frontleftSum, frontrightSum, frontSum, leftsumscaled, rightsumscaled;
volatile unsigned long next;

const int ledPin =  LED_BUILTIN;
int ledState = LOW;   
unsigned long previousMillis = 0; 
const long interval = 1000;   





//=======================================================================

void setup() {

Serial.begin(115200);
Wire.begin(33, 35);
IBus.begin(Serial1, IBUSBM_NOTIMER);
pinMode(LED_BUILTIN, OUTPUT);


ESP32_ISR_Servos.useTimer(USE_ESP32_TIMER_NO);
 scanner = ESP32_ISR_Servos.setupServo(SERVOLIDAR, 500, 2500);
  MOTout = ESP32_ISR_Servos.setupServo(MOTOR, MIN_MICROS, MAX_MICROS);
  RUDout = ESP32_ISR_Servos.setupServo(RUDDER, MIN_MICROS, MAX_MICROS);
  MODout = ESP32_ISR_Servos.setupServo(MODE, MIN_MICROS, MAX_MICROS);
  pumpout = ESP32_ISR_Servos.setupServo(PUMP, MIN_MICROS, MAX_MICROS);

  
  sensor.setTimeout(100);
  if (!sensor.init()){val = 1;}
  if (sensor.init()){


  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(35000); //35ms
  sensor.startContinuous(35);
 }


  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

     LIDARPIDINPUT = yaw;
     LIDARSETPOINT = 0;
     LIDARPID.SetMode(AUTOMATIC);

     GYROPIDINPUT = (gyroz);
     GYROSETPOINT = 0;
     GYROPID.SetMode(AUTOMATIC);

}


void loop() { 

ReadLidar();
GetNAV();
ReadIMU();
readrc();
modeselect();
avoidmodes();
controlmodes();
Mixer();
servooutput();
gpio();
serialprint();

}

  


  
void ReadIMU(){

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  gyroz = ((g.gyro.z) * 0.9) + ((g.gyro.z) * 0.1); 
  GYROPID.Compute();
}  


void Mixer (){


outch1 = map(out, 1000, 2000, 0, 180);
outch2 = map(rudout, 1000, 2000, 0, 180);
outch3 = map(flightmode, 1000, 2000, 0, 180);
outch4 = ch4;
outch5 = ch5;
outch6 = ch6;
outch7 = ch7;
outch8 = ch8;
outch9 = ch9;
outch10 = ch10;
outch11 = ch11;
outch12 = ch12;
outch13 = ch13;
outch14 = ch14;

}




void servooutput(){

ESP32_ISR_Servos.setPosition(MOTout,outch1);
ESP32_ISR_Servos.setPosition(RUDout,outch2);
ESP32_ISR_Servos.setPosition(MODout,outch3);
}



void gpio(){
  
//bilge = analogRead(A0);
//if (bilge <500){
//  ESP32_ISR_Servos.setPulseWidth(pumpout,(2000));
//}
//else{
// ESP32_ISR_Servos.setPulseWidth(pumpout,iBus.readChannel(6));
// }

//PUM = map(MOTORout, 1000, 2000, 50, 1);


}
