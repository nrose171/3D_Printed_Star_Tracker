/*
 This is the source code for the master-module of Star Track.
 Required Libraries:
 https://virtuabotix-virtuabotixllc.netdna-ssl.com/core/wp-content/uploads/2014/01/virtuabotixRTC.zip
 https://drive.google.com/folderview?id=0B1p6T9dV6tnqSjU2WnBXREZPbms&usp=sharing 
 
 Created 20 July 2016 by Görkem Bozkurt
 */
#include <AccelStepper.h>
#include <virtuabotixRTC.h>
#include <MPU6050.h>
#include <Wire.h>
MPU6050 mpu;
//define RTC.
#define HALFSTEP 8
virtuabotixRTC myRTC(A0, A1, A2);
double Months,Years,Days,Minutes,Hours,Seconds;
double days_in_year,days_in_month;

double latitude = 39.5453;//your latitude.
double longitude = -119.8162;//your longtitude.

double LST_degrees;//variable to store local side real time(LST) in degrees.
double LST_hours;//variable to store local side real time(LST) in decimal hours.

unsigned long timer = 0;
float timeStep = 0.01;

// Global flags used for functions
boolean yaw_finished = false;
boolean pitch_finished = false;
boolean pointing = false;
boolean ra_origin_is_set = false;
boolean ra_tracking_is_set = false;

// Pitch and Yaw values
double pitch = 90; //Equivalent to DEC
double yaw = 0; //Equivalent to RA
double val = 90; //Pitch and DEC
double val2 = 0; //Yaw and RA

double final_yaw_measurement = 0; //Store yaw measurement for when you are 

double temp = val2;//temporary value to store val2

// MOTOR SPEED CALCULATION CONSTANTS //

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

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper with 28BYJ-48
AccelStepper yawStepper(HALFSTEP, motorPin1, motorPin3, motorPin2, motorPin4); //YAW Stepper Motor
AccelStepper pitchStepper(HALFSTEP, motorPin5, motorPin7, motorPin6, motorPin8); //PITCH Stepper Motor

boolean stopped1 = false;
boolean stopped2 = false;

void setup() 
{
    //set date-time according to (seconds, minutes, hours, day of the week, day of the month, month, year) 
    myRTC.setDS1302Time(00, 25, 18, 3, 20, 4, 2021);
    Serial.begin(115200);
    
    yawStepper.setMaxSpeed(100.0);
    pitchStepper.setMaxSpeed(100.0);
    delay(5000);//wait before starting

    setupMPU();
    
    Serial.println("MPU calibrated");
}//--(end setup )---

void loop()
{
  motor_pitch( 0 ); //Stop pitch before moving again
  motor_yaw( 0 ); //Stop yaw before moving again
  
  myRTC.updateTime();
  LST_time();
  recvdata();
  
  timer = millis();
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

  delay((timeStep*1000) - (millis() - timer));//timer for the gyro.
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
      //if( floor(LST_degrees)==LST_degrees )
  //    { 
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
  //    }
    }
    else //For the southern hemisphere.
    {
      Serial.println("southern hemisphere");
     if( floor(LST_degrees)==LST_degrees )
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
    }
    yaw_check();
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
    //LST_degrees = 144;
    LST_hours = LST_degrees/15;
    //LST_hours = 9.60;
    
//    Serial.println(Seconds);
//    Serial.println(days_in_year);
//    Serial.println(days_in_month);
//    Serial.println(JDN2000);
//    Serial.println(universal_time);
//    Serial.println(LST);
//    Serial.println(LST_hours);
//    Serial.println(LST_degrees);
//    Serial.print("\n");
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
