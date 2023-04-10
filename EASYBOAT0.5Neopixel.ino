#include <Wire.h>
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> 



#define stepAng           10      // step angle
#define numStep           18        // = 180/stepAng
#define averagedivider    2
 
#define MIN_MICROS        1000  
#define MAX_MICROS        2000

#define minreverse         500    
#define fullreverse        150  

#define turnrange         2000

#define SERVORADAR           7
#define MOTOR               6     
#define RUDDER              2
#define MODE                3
#define PUMP                4
//---------------------------------------------------------------------------------


#define MONITOR_SERIAL Serial
#define RADAR_SERIAL Serial1
#define RADAR_RX_PIN 9
#define RADAR_TX_PIN 8

#include <ld2410.h>
ld2410 radar;

uint32_t lastReading = 0;
bool radarConnected = false;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define ISR_SERVO_DEBUG             0
#define USE_ESP32_TIMER_NO          2
#include "ESP32_S2_ISR_Servo.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Define variables to store BME280 readings to be sent
float temperature = 0;
float humidity = 0 ;
float pressure = 0;

// Define variables to store incoming RC
int ech1 = 1500;  //throttle
int ech2 = 1500;  //rudder
int ech3 = 1500;  //camera yaw
int ech4 = 1500;  //lights
int ech5 = 1500;  //anchor winch
int ech6 = 1500;  //tow winch
int ech7 = 1500;  //bilge pump / fire monitor
int ech8 = 1500;  //flight mode
int ech9 = 1500;  //avoid gain
int ech10 = 1350; //avoid mode
int ech11 = 1350; //avoid mode
int ech12 = 1350; //avoid mode
int ech13 = 1350; //avoid mode
int ech14 = 1350; //avoid mode

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float eh1;
    float eh2;
    float eh3;
    float eh4;
    float eh5;
    float eh6;
    float eh7;   
    float eh8;
    float eh9;
    float eh10;
    float eh11;
    float eh12;
    float eh13;
    float eh14;

    
} struct_message;

// Create a struct_message called TELEMETRY_DATA to hold sensor readings
struct_message TELEMETRY_DATA;

// Create a struct_message to hold incoming sensor readings
struct_message INCOMING_RC;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&INCOMING_RC, incomingData, sizeof(INCOMING_RC));
  Serial.print("Bytes received: ");
  Serial.println(len);
  ech1 = INCOMING_RC.eh1;
  ech2 = INCOMING_RC.eh2;
  ech3 = INCOMING_RC.eh3;
  ech4 = INCOMING_RC.eh4;
  ech5 = INCOMING_RC.eh5;
  ech6 = INCOMING_RC.eh6;
  ech7 = INCOMING_RC.eh7;
  ech8 = INCOMING_RC.eh8;
  ech9 = INCOMING_RC.eh9;
  ech10 = INCOMING_RC.eh10;
  ech11 = INCOMING_RC.eh11;
  ech12 = INCOMING_RC.eh12;
  ech13 = INCOMING_RC.eh13;
  ech14 = INCOMING_RC.eh14;
}
 

#include <Adafruit_NeoPixel.h>
#define PIN            6
#define NUMPIXELS      16
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
int delayval = 500;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 4800;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
unsigned long last = 0UL;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//variables for PID control
float target = 0;
float error = 0;
float integral = 0;
float derivative = 0;
float last_error = 0;

//the 'k' values are the ones you need to fine tune before your program will work. Note that these are arbitrary values that you just need to experiment with one at a time.
float Kp = 11;
float Ki = 0.09;
float Kd = 10;

int mtrSpd = 127.5;

float angle = 0;

//Variables for Gyroscope
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;

long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;

// Setup timers and temp variables
long loop_timer;
int temp;
int state = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////


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
int stabyawen = 0;

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

int stabyaw;
int esc;
int pointesc;
int escs;
int yaw;
int pointyaw;
int out = 1500;
int rudout = 1500;
int rudservoout = 1500;

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
int val;              // RADAR measured value
int range;
float distances[numStep+1]; // array for the distance at each angle, including at 0 and 180
float leftSum, rightSum, frontleftSum, frontrightSum, frontSum, leftsumscaled, rightsumscaled;
volatile unsigned long next;

const int ledPin =  LED_BUILTIN;
int ledState = LOW;   
unsigned long previousMillis = 0; 
const long interval = 1000;   


  void setup_mpu_6050_registers(){

  //Activate the MPU-6050
  
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68); 
  //Send the requested starting register                                       
  Wire.write(0x6B);  
  //Set the requested starting register                                                  
  Wire.write(0x00);
  //End the transmission                                                    
  Wire.endTransmission(); 
                                              
  //Configure the accelerometer (+/-8g)
  
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68); 
  //Send the requested starting register                                       
  Wire.write(0x1C);   
  //Set the requested starting register                                                 
  Wire.write(0x10); 
  //End the transmission                                                   
  Wire.endTransmission(); 
                                              
  //Configure the gyro (500dps full scale)
  
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  //Send the requested starting register                                        
  Wire.write(0x1B);
  //Set the requested starting register                                                    
  Wire.write(0x08); 
  //End the transmission                                                  
  Wire.endTransmission(); 
                                              
}


void read_mpu_6050_data(){ 

  //Read the raw gyro and accelerometer data

  //Start communicating with the MPU-6050                                          
  Wire.beginTransmission(0x68);  
  //Send the requested starting register                                      
  Wire.write(0x3B);
  //End the transmission                                                    
  Wire.endTransmission(); 
  //Request 14 bytes from the MPU-6050                                  
  Wire.requestFrom(0x68,14);    
  //Wait until all the bytes are received                                       
  while(Wire.available() < 14);
  
  //Following statements left shift 8 bits, then bitwise OR.  
  //Turns two 8-bit values into one 16-bit value                                       
  acc_x = Wire.read()<<8|Wire.read();                                  
  acc_y = Wire.read()<<8|Wire.read();                                  
  acc_z = Wire.read()<<8|Wire.read();                                  
  temp = Wire.read()<<8|Wire.read();                                   
  gyro_x = Wire.read()<<8|Wire.read();                                 
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read(); 
                                 
}



//=======================================================================

void setup() {

Serial.begin(115200);
Wire.begin(33, 35);
Serial1.begin(256000); //UART for monitoring the radar
pinMode(LED_BUILTIN, OUTPUT);


ESP32_ISR_Servos.useTimer(USE_ESP32_TIMER_NO);
 scanner = ESP32_ISR_Servos.setupServo(SERVORADAR, 500, 2500);
  MOTout = ESP32_ISR_Servos.setupServo(MOTOR, MIN_MICROS, MAX_MICROS);
  RUDout = ESP32_ISR_Servos.setupServo(RUDDER, MIN_MICROS, MAX_MICROS);
  MODout = ESP32_ISR_Servos.setupServo(MODE, MIN_MICROS, MAX_MICROS);
  pumpout = ESP32_ISR_Servos.setupServo(PUMP, MIN_MICROS, MAX_MICROS);

  

     
   //Setup the registers of the MPU-6050                                                       
  setup_mpu_6050_registers(); 
  
  //Read the raw acc and gyro data from the MPU-6050 1000 times                                          
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++){                  
    read_mpu_6050_data(); 
    //Add the gyro x offset to the gyro_x_cal variable                                            
    gyro_x_cal += gyro_x;
    //Add the gyro y offset to the gyro_y_cal variable                                              
    gyro_y_cal += gyro_y; 
    //Add the gyro z offset to the gyro_z_cal variable                                             
    gyro_z_cal += gyro_z; 
    //Delay 3us to have 250Hz for-loop                                             
    delay(3);                                            
}
 // Divide all results by 1000 to get average offset
  gyro_x_cal /= 1000;                                                 
  gyro_y_cal /= 1000;                                                 
  gyro_z_cal /= 1000;
    
  // Init Timer 
  loop_timer = micros();   


  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  pixels.begin();
                                              

}


void loop() { 

ReadRADAR();
GetNAV();
ESPNOW();
modeselect();
avoidmodes();
controlmodes();
gyro();
Mixer();
servooutput();
gpio();
neopixel();
serialprint();

}
