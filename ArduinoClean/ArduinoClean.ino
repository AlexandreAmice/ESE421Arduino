#include <Servo.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_GPS.h>

#define PI 3.1415926535897932384626433832795

#define SLAVE_ADDRESS 0x04 //Address for the pi to talk to arduino DO NOT CHANGE

#define servoPin 7 // pin for servo signalj

#define motorPin 8 // PWM for motor

//ping sensor define
#define pingTrigPin 23 // ping sensor trigger pin (output from Arduino)
#define pingEchoPin 25 // ping sensor echo pin (input to Arduino)
#define pingGrndPin 27 // ping sensor ground pin (use digital pin as ground)

//IMU sensor define
#define LSM9DS1_SCK 52  //BDK-mega
#define LSM9DS1_MISO 50 //BDK-mega
#define LSM9DS1_MOSI 51 //BDK-mega
#define LSM9DS1_XGCS 49 //BDK-mega
#define LSM9DS1_MCS 47 //BDK-mega

//
// tell sensor library which pins for accel & gyro data
//
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);



//
// global variables
// (yucky but needed to make i2c interrupts work later)
//
float SERVOANGLENEUT = 100; //set this to the neutral point of the servo
float pingDistanceCM = 0.0; //ping sensor var
float gyroZbias = 0.8; //degrees per second


//COMMAND LIST
byte piCommand = 0;
const byte psiDCom = 0; //DO NOT CHANGE THIS NEEDS TO BE THE SAME BETWEEN ARDUINO AND PI
const byte  dSpeedCom = 1; //DO NOT CHANGE THIS NEEDS TO BE THE SAME BETWEEN ARDUINO AND 
const byte  gpsLatCom = 2;
const byte  gpsLonCom = 3;
const byte  gpsVCom = 4;
const byte  gpsPsiCom = 5;

//gps vars. Default all to 0
float DEF = 0;
float gpsLat = 25.789;
float gpsLon = DEF;
float gpsV = DEF;
float gpsPsi = DEF;
int gpsNSat = DEF;
//
// connect GPS to Hardware Serial1 (Serial0 is for USB)
// Serial1 is pins 18 & 19 on Mega
//
HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&Serial1);


//
// Global variables deciding what our robot should be doing
byte motorPWM=150; //needs to be between 0-255. this is motor speed
float psiD = 0;
float dSpeed = 180;




Servo steeringServo; //servo name variable


void setup() {
   
//  make the motors not spin
//
    pinMode(motorPin,OUTPUT);
    analogWrite(motorPin,0);

    //servo setup
    steeringServo.attach(servoPin); //initalize servo to pin
    steeringServo.write(SERVOANGLENEUT);//set servo to straight line

    //gyro setup
    setupGyro();
    
//
//  set up the ping sensor pins
//
    pinMode(pingGrndPin,OUTPUT); digitalWrite(pingGrndPin,LOW);
    pinMode(pingTrigPin,OUTPUT);
    pinMode(pingEchoPin,INPUT);

    //Prepare Raspberry Pi Communication
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveData2);
    Wire.onRequest(sendData2);

    //set up GPS
    setupGPS();
    Serial.begin(115200);
    
}

void loop() {
  unsigned long time = millis(); //keep track of how long its been since the loop started. want to loop every 20 milliseconds

  static float estHeading = psiD;
  static int count = 0;
  static float curSpeed = 0;
  float* estHeadAddr = &estHeading; //maintain an address to the heading angle so we don't need to make a global variable
  
  curSpeed = setSpeed(curSpeed); //set the wheel speed
  //updateGPSData(); //updates our GPS data for
  //correctHeading(psiD, estHeadAddr); //keep track of the estimated heading angle and correct it as needed.
  correctHeadingRel(psiD);

  if(count > 100){
   //every few seconds print to Serial monitor some debugging information 
   //Serial.print("Estimated Heading: "); Serial.println(*estHeadAddr);
   Serial.print("Desired Heading: "); Serial.println(psiD);
   //Serial.print("GPS Heading: "); Serial.println(gpsPsi);
   //Serial.print("Desired Speed: "); Serial.println(dSpeed);  
    count = 0;
   }

  count++;
//
//  pause waiting for 20 milliseconds
//
   int DELAY = 20; //amount we wish to delay in miliseconds
   while(millis() <= time + DELAY){
      
   }
   
   
}


////////////////////////////////////////////////////////////
// Adjust wheels to set self to correct heading
////////////////////////////////////////////////////////////
float correctHeading(float psiD, float* estHeadingAddr) {
  //Simple P controller for adjusting heading based on psiD

  //gyro readings
  lsm.read();  /* ask it to read in the data */
  sensors_event_t a, m, g, temp; //a = linear acceleration , m = magnetometer reading, g = gyro acceleration
  lsm.getEvent(&a, &m, &g, &temp);
  static int count = 0;

  
  float estHeading = *estHeadingAddr; //for computation
  float deltaTms = 20*0.001; //dt
  float headingK = 10;

  float deltaAng = (deltaTms)*(g.gyro.z-gyroZbias);

  
  *estHeadingAddr += deltaAng;  

  float servoAngleDeg = SERVOANGLENEUT - headingK * (psiD - *estHeadingAddr);
   steeringServo.write(constrain(servoAngleDeg, SERVOANGLENEUT-25, SERVOANGLENEUT+25));

  return estHeading;
}

////////////////////////////////////////////////////////////
// Adjust wheels to set self to correct heading for Relative Heading
////////////////////////////////////////////////////////////
float correctHeadingRel(float psiD) {
  //Simple P controller for adjusting heading based on psiD

  float deltaTms = 20*0.001; //dt
  float headingK = 10;

  float servoAngleDeg = SERVOANGLENEUT - headingK * (psiD);
  steeringServo.write(constrain(servoAngleDeg, SERVOANGLENEUT-25, SERVOANGLENEUT+25));

  //return estHeading;
}

////////////////////////////////////////////////////////////
// Update Value of speed
////////////////////////////////////////////////////////////
float setSpeed(float prev) {
  //
  //Return the the value of the speed that we wish to drive at
  //
  getPingDistanceCM(); //get distance to furthest object
  int Kspeed = 5;
  int Kint = 0.5;
  float deltaTms = 20*0.001; //dt
  
  float error = pingDistanceCM - 30;
  float intError = error*deltaTms;
  byte val = Kspeed*constrain(error+intError,0,255);
  Serial.println(val);
  motorPWM = constrain(val,0,250);
  analogWrite(motorPin, motorPWM);
  return 0.002069*val+.2824 //motor parameters
}

////////////////////////////////////////////////////////////
// Ping Sensor -- update value of pingDistanceCM
////////////////////////////////////////////////////////////
void getPingDistanceCM()
{
  //
  // 3000 us timeout implies maximum distance is 51cm
  // but in practice, actual max larger?
  //
  const long timeout_us = 3000;
  //
  // pingTrigPin = trigger pin
  // pingEchoPin = echo pin
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  //
  digitalWrite(pingTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingTrigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingTrigPin, LOW);
  //
  // The echo pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  //
  unsigned long echo_time;
  echo_time = pulseIn(pingEchoPin, HIGH, timeout_us);
  if (echo_time == 0)
 {
    echo_time = timeout_us;
 }
  //
  // return the distance in centimeters
  // distance = (10^-6) * (echo_time_us) * (speed of sound m/s) * (100 cm/m) / 2
  // divide by 2 because we measure "round trip" time for echo
  // (0.000001 * echo_time_us * 340.0 * 100.0 / 2.0)
  // = 0.017*echo_time
  //
  pingDistanceCM = constrain(0.017*echo_time,5.0,50.0);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CONTROL HELPERS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////
// Read data from I2C communication from Raspberry Pi
////////////////////////////////////////////////////////////

//Assumes data is sent as a series of chars 'command number' 'number of chars' 'numbers'
void receiveData2(int byteCount){
  //Serial.println("I am here");
  piCommand = Wire.read(); //this line is essential. do not change it
  //Serial.println(piCommand);
  int comSize = byteCount-1;

  if(piCommand == psiDCom || piCommand == dSpeedCom){
    char comNum[comSize];
    int idx = 0;
    while(idx < byteCount && Wire.available()) {
     char nextChar = Wire.read();
     comNum[idx] = nextChar;
     idx ++; 
    }
    
    while(Wire.available()) {
     Wire.read(); //clear the wire buffer 
    }
     
    float newVal = string2float(comNum, comSize-1);
    mapToData(piCommand, newVal);
      
  }
  while (Wire.available()) {Wire.read();}
}

//Horrendous hack to convert a character array into a floating point decimal
float string2float(char theStr[], int byteCount){
  float preDec = 0;
  float postDec = 0;
  float retVal = 0;
  bool decPointFlag = false;
  int decCount = 0;
  bool negative = false;
  for(int i = 0; i <= byteCount; i++){
    char curChar = theStr[i];
    //Serial.println(curChar);
    if((char)curChar == '.'){
      decPointFlag = true;
    }
    else if(curChar == '-'){
      negative = true;
    }
    else {
      if(!decPointFlag){
        preDec = preDec * 10 + (curChar-48);
      }
      if(decPointFlag){
        postDec = postDec*10+(curChar-48);
        decCount += 1;
      }
      
    }
  }
  
  retVal = preDec + postDec / pow(10, decCount);

  if(negative) {
    retVal = -1* retVal;
  }
  return retVal;
}


//map new value to data
void mapToData(byte command, float newVal){
  //Serial.println(command);
  switch (command) {
    case psiDCom:
      psiD = newVal;
      //Serial.print("new psiD is "); 
      //Serial.println(psiD,7);
      break;
    
    case dSpeedCom:
      //Serial.print("new dSpeed is "); 
      //Serial.println(dSpeed,7);
      dSpeed = newVal;
      break;
  
    default:    
      break;   
  }
}

////////////////////////////////////////////////////////////
// Send Data back to Raspberry Pi
////////////////////////////////////////////////////////////

void sendData2() {
  //Sends back exactly one float of data. If you change this take care to make adjustment on the raspberry pi side
  float dataBuffer[1];
  switch (piCommand) {
    case gpsLatCom:
      dataBuffer[0] = gpsLat;
    break;

    case gpsLonCom:
      dataBuffer[0] = gpsLon;
    break;

    case gpsVCom:
      dataBuffer[0] = gpsV;
    break;

    case gpsPsiCom:
      dataBuffer[0] = gpsPsi;
    break;

   default:
   break;
  }

  Wire.write((byte*) &dataBuffer[0], sizeof(float));
  
}


////////////////////////////////////////////////////////////
// Updates global variables of GPS signal
////////////////////////////////////////////////////////////
void updateGPSData(){
  
   if (GPS.newNMEAreceived())
    {
       if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
       {
          Serial.println("GPS Parse Fail");
       }
       else
       {
          gpsLat = (GPS.latitudeDegrees - 40.0);
          gpsLon = (GPS.longitudeDegrees + 75.0);
          gpsV = GPS.speed;
          //make sure the GPS heading angle isn't way off
          gpsPsi = GPS.angle;
          gpsNSat = GPS.satellites;
       }
    }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP HELP
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////
// Perform Gyro Setup
////////////////////////////////////////////////////////////
void setupGyro() {
  //
// initialize gyro / mag / accel
//
    if (!lsm.begin())
    {
      Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
      while (1);
    }
    Serial.println("Found LSM9DS1 9DOF");
    //
    // set ranges for sensor
    //
    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

    //get a gzbias
    float gyroAvg = 0;
    for(int i = 0; i < 100; i++) {
      lsm.read();  /* ask it to read in the data */
      sensors_event_t a, m, g, temp; //a = linear acceleration , m = magnetometer reading, g = gyro acceleration
      lsm.getEvent(&a, &m, &g, &temp);
      gyroAvg += g.gyro.z;
      setSpeed();
      delay(20);
    }
    gyroAvg /= 100;
    gyroZbias = gyroAvg;
}


////////////////////////////////////////////////////////////
// Performs GPS Setup
////////////////////////////////////////////////////////////
void setupGPS() {
  //
//  Activate Interrupt for GPS
//
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
//
//  initialize comm with GPS at 9600 baud
//
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY); // minimum information only
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
}

//  This Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
   char c = GPS.read();
} 
