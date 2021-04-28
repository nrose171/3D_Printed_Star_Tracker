/*
 This is the source code for the astro_tracker.
 Required Libraries:
 https://virtuabotix-virtuabotixllc.netdna-ssl.com/core/wp-content/uploads/2014/01/virtuabotixRTC.zip
 https://drive.google.com/folderview?id=0B1p6T9dV6tnqSjU2WnBXREZPbms&usp=sharing 
 
 Original code by Görkem Bozkurt
 Refactored code by Nathaniel Rose
*/

// LIBRARIES //
#include <AccelStepper.h>
#include <virtuabotixRTC.h>
#include <Wire.h>
#include <MPU6050.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// TinyGPS++ VARIABLES //
static const uint32_t GPSBaud = 9600; //Default is 9600 baud rate
static const int RXPin = 13, TXPin = 12; //Tx on gps goes into Rx on arduino and vice versa.

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin); // The serial connection to the GPS device

// MPU VARIABLES //
MPU6050 mpu;
float timeStep = 0.01;
unsigned long mpuTimer = 0;

// Pitch and Yaw values //
double pitch = 90; //Trying to orientate to DEC
double yaw = 0; //Trying to orientate to RA

double val = 90; //variable to store the user input DEC
double val2 = 0; //variable to store the user input RA

double temp = val2;//temporary value to store val2
double final_yaw_measurement = 0; //Store yaw measurement for when you are 

// RTC VARIABLES //
#define HALFSTEP 8
virtuabotixRTC myRTC(A0, A1, A2);

double Months,Years,Days,Minutes,Hours,Seconds;
double timeSet = false;//Checks if the gps has set the UTC time
double days_in_year,days_in_month;

double latitude;//your latitude.
double longitude;//your longtitude.

double LST_degrees;//variable to store local side real time(LST) in degrees.
double LST_hours;//variable to store local side real time(LST) in decimal hours.


// STEPPER MOTOR VARIABLES //

// motor speed calculation variables //
float rot_per_step = 5.625/64; // 1 step = 5.625/64 degrees (28BYJ-48)
float gear_ratio = 70/21; //Depends on your gears
#define ROT_OF_EARTH 0.0000729211509 //(Rad/sec)
float SPEED = 40.0; // Speed of motors (steps / second)

// Motor pin definitions //
#define motorPin1  2     // IN1 on the ULN2003 driver 1
#define motorPin2  3     // IN2 on the ULN2003 driver 1
#define motorPin3  4     // IN3 on the ULN2003 driver 1
#define motorPin4  5     // IN4 on the ULN2003 driver 1

#define motorPin5  8     // IN1 on the ULN2003 driver 1
#define motorPin6  9     // IN2 on the ULN2003 driver 1
#define motorPin7  10     // IN3 on the ULN2003 driver 1
#define motorPin8  11     // IN4 on the ULN2003 driver 1

boolean stopped1 = false;
boolean stopped2 = false;

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper with 28BYJ-48 //
AccelStepper yawStepper(HALFSTEP, motorPin1, motorPin3, motorPin2, motorPin4); //YAW Stepper Motor
AccelStepper pitchStepper(HALFSTEP, motorPin5, motorPin7, motorPin6, motorPin8); //PITCH Stepper Motor

// GLOBAL FLAG VARIABLES USED FOR FUNCTIONS //
boolean positionSet = false; //Checks if the gps has set the lat/long position
boolean pointing = false; //Checks if a desired yaw and pitch angle has been entered
boolean yaw_finished = false; //Checks if the desired yaw angle has been reached
boolean pitch_finished = false; //Checks if the desired pitch angle has been reached
boolean ra_origin_is_set = false; //Checks if the RA angle has been set to LST
boolean ra_tracking_is_set = false; //Checks if the star is being tracked

void setup() 
{
  //Setup serial ports//
  Serial.begin(115200);
  ss.begin(GPSBaud);

  //Set max speeds for stepper motors//
  yawStepper.setMaxSpeed(100.0);
  pitchStepper.setMaxSpeed(100.0);

  //Initialize mpu and gps modules//
  delay(5000);//wait before starting
  setupMPU();

  delay(1000);
  setupGPS();
    
}//--(end setup )---

void loop()
{
  motor_pitch( 0 ); //Stop pitch before moving again
  motor_yaw( 0 ); //Stop yaw before moving again
  
  myRTC.updateTime();
  LST_time();
  recvdata();
  
  mpuTimer = millis();
  Vector norm = mpu.readNormalizeGyro();
  
  //I've put the sensor with a 90 degree angle on the setup due to
  //cable connection problems. Because of that the data values from the mpu6050 chip are
  //different in this case:
  //roll data(X-axis) is pitch.
  //pitch data(Y-axis) is yaw.
  yaw = yaw + norm.YAxis * timeStep;
  pitch = pitch + norm.XAxis * timeStep;
  Serial.print(" Yaw = ");
  Serial.print(yaw);
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" LST_d = ");
  Serial.print(LST_degrees);//local sidereal time in decimal degrees
  Serial.print(" LST_h = ");
  Serial.println(LST_hours);//local sidereal time in decimal hours.

  Serial.print("target yaw = ");
  Serial.println(val2);

  changeRA(); //Change RA with time based on our latitude

  point_at_star(); //Point at the star

  trackRA(); //

  delay((timeStep*1000) - (millis() - mpuTimer));//timer for the gyro.
}

void setupGPS()
{
  //Wait to get a GPS fix
  while( (!positionSet || !timeSet) && ss.available() > 0  )
  {
    if(gps.encode(ss.read()))
      getGPSInput();
  }

  if( millis() > 5000 && gps.charsProcessed() < 10 )
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}

void getGPSInput()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    positionSet = true;
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if(gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
    Years = gps.date.year();
    Months = gps.date.month();
    Days = gps.date.day();
    
    if(gps.time.isValid())
    {
      if (gps.time.hour() < 10) Serial.print(F("0"));
      Serial.print(gps.time.hour());
      Serial.print(F(":"));
      if (gps.time.minute() < 10) Serial.print(F("0"));
      Serial.print(gps.time.minute());
      Serial.print(F(":"));
      if (gps.time.second() < 10) Serial.print(F("0"));
      Serial.print(gps.time.second());
      Serial.print(F("."));
      if (gps.time.centisecond() < 10) Serial.print(F("0"));
      Serial.print(gps.time.centisecond());
      Hours = gps.time.hour();
      Minutes = gps.time.minute();
      Seconds = gps.time.second();

      //set date-time according to (seconds, minutes, hours, day of the week, day of the month, month, year) 
      myRTC.setDS1302Time(00, 25, 16, 5, 8, 4, 2021);
      timeSet = true;
    }
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();

  Serial.print(F("Satellites Used: "));
  if(gps.satellites.isValid())
  {
    Serial.println(gps.satellites.value()); // Number of satellites in use (u32)
  }
  else{
    Serial.println(F("Invalid"));
  }

  Serial.println();
}

//Waits while the mpu is being connected and calibrated
void setupMPU() {
    while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) //wait for MPU to be ready
    {
      delay(500);
      Serial.println("Waiting for MPU...");
    }
    mpu.calibrateGyro();
    mpu.setThreshold(0); //Set the sensitivity threshold of the mpu (0 is most sensitive, 10 is least sensitive)
}

//Calculates local sidereal time based on this calculation,
//http://www.stargazing.net/kepler/altaz.html 
void LST_time()
{
    Months = (double) myRTC.month;
    Years = (double) myRTC.year;
    Days = (double) myRTC.dayofmonth;
    Minutes = (double) myRTC.minutes;
    Hours = (double) myRTC.hours;
    Seconds = (double) myRTC.seconds;
    days_in_year = (double)(Years-2000)*365.242199; //Days in the current year since J2000
    days_in_month = (double)(Months-1)*30.4368499; //Days in the current month since J2000
    double JDN2000= days_in_year + days_in_month + (Days-1) + myRTC.hours/24; //Days since Jan 1st, 2000AD
    double universal_time = Hours + (Minutes/60) + (Seconds/3600);
    double LST = 100.46 + 0.985647 * JDN2000 + longitude + 15 * universal_time;
    LST_degrees = (LST-(floor(LST/360)*360));
    LST_hours = LST_degrees/15;
}

//this will update the RA degrees with sidereal time 1 degree at a time
//this way the object or star on the sky is tracked.
void changeRA() {
  if( ra_origin_is_set == false )
  {
    Serial.println("changing RA");
    if(latitude>0) //For the northern hemisphere
    {
      Serial.print("LST_degrees: ");
      Serial.println(LST_degrees);
        if (LST_degrees>180)
        {
          val2 = temp+(360-LST_degrees);
        }
        else
        {
          val2 = temp-LST_degrees; //use val2 = temp+LST_degrees; if you are located in the southern hemisphere.
        }
        Serial.print("target yaw = ");
        Serial.println(val2);
    }
    else //For the southern hemisphere.
    {
      if (LST_degrees>180)
      {
        val2 = temp-(360-LST_degrees);
      }
      else
      {
        val2 = temp+LST_degrees; //use val2 = temp+LST_degrees; if you are located in the southern hemisphere.
      }
    }
    yaw_check();
  }
}

//This function receives data from serial as (0.00,0.00)
//splits it to strings by the comma ","
//than converts them to doubles 
void recvdata()
{
    if (Serial.available() > 0)
    {
        String a= Serial.readString();
        String value1, value2;
        // For loop which will separate the String in parts
        // and assign them the the variables we declare
        for (int i = 0; i < a.length(); i++) 
        {
            if (a.substring(i, i+1) == ",") 
            {
                value2 = a.substring(0, i);
                value1= a.substring(i+1, a.length()-1);
                break;
            }
        }
        //val= 90 - value1.toFloat();
        val=value1.toFloat();
        val2=value2.toFloat();
        temp = val2;
        pointing = true;
    }
}

void point_at_star()
{
  if(pointing == true)
  {
    if(yaw_finished == false)
    {
      yaw_check();
    }
    else if(pitch_finished == false)
    {
      pitch_check();
    }
  }
}

//check if pitch is high, low or equal to the user input
//send commands to slave-module to start and stop motors
void pitch_check()
{
    if( (floor(pitch*100))==(floor(val*100)) )
    {
        motor_pitch( 0 );
        pitch_finished = true;
        yaw = final_yaw_measurement;
        ra_tracking_is_set = true;
    }
    
    else if(floor(pitch*100)<floor(val*100))
    {
        motor_pitch( 1 );
    }
    
    else if(floor(pitch*100)>floor(val*100))
    {
        motor_pitch( -1 );
    }
}

//check if yaw is high, low or equal to the user input
//send commands to slave-module to start and stop motors
void yaw_check()
{
    if( (floor(yaw*100))==(floor(val2*100)) )
    {
        motor_yaw( 0 );
        if(ra_origin_is_set == false)
        {
          ra_origin_is_set = true;
          yaw = 0;
        }
        else
        {
          yaw_finished = true;
          pitch = 90; //reset original pitch measurement
          final_yaw_measurement = yaw; //save final yaw measurement
        }
    }
    
    else if(floor(yaw*100)<floor(val2*100))
    {
        motor_yaw( -1 );
    }
    
    else if(floor(yaw*100)>floor(val2*100))
    {
        motor_yaw( 1 );
    }
}

//If the pitch motor is not stopped, run the pitch motor either clockwise(SPEED) or counter-clockwise(-SPEED)
void motor_pitch(int stepperMove)
{
    if(stepperMove == 0)
    {
        stopped2 = true;
    }
    else
    { 
      if(stepperMove == 1)
      {
            pitchStepper.setSpeed(SPEED);
            stopped2 = false;
      }
      else if(stepperMove == -1)
      {
            pitchStepper.setSpeed(-SPEED);
            stopped2 = false;
      }
      
      pitchStepper.run(); //run the motor
    }
}

//If the yaw motor is not stopped, run the yaw motor either clockwise(SPEED) or counter-clockwise(-SPEED)
void motor_yaw(int stepperMove)
{
    if(stepperMove == 0)
    {
        stopped1 = true;
    }
    else
    { 
      if(stepperMove == 1)
      {
            yawStepper.setSpeed(SPEED);
            stopped1 = false;
      }
      else if(stepperMove == -1)
      {
            yawStepper.setSpeed(-SPEED);
            stopped1 = false;
      }
      yawStepper.run();
    }
}

//this will update the RA degrees with sidereal time 1 degree at a time
//this way the object or star on the sky is tracked.
void trackRA() {
  if( ra_tracking_is_set == true )
  {
    float rot_of_earth_degrees = ROT_OF_EARTH * (180/3.14); //rotation of earth (degrees/sec)
    float earth_rot_steps_per_sec = rot_of_earth_degrees / rot_per_step; //rotation of earth (steps/sec) 
    float stepper_tracking_speed = earth_rot_steps_per_sec * gear_ratio; //rotation speed of YAW stepper motor (steps/sec)
  
    SPEED = stepper_tracking_speed;
    
    motor_yaw(1);
  }
}
