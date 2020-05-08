#include <PID_v1.h>
#include <Arduino.h>
#include <Wire.h>
#include <HMC5883L_Simple.h>
#include <TinyGPS.h>


// RF Module //

/*-----( Import needed libraries as headers )-----*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/*-----( Declare Constants and Pin Numbers )-----*/
#define CE_PIN   9
#define CSN_PIN 53

// NOTE: the "LL" at the end of the constant is "LongLong" type
const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe

/*-----( Declare objects )-----*/
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio

char  radio_string[100];
int   radio_count[1] = {0};


int dummy[2] = {10,20};

//RF
#define propellerKp 0.4
#define propellerKi 0
#define propellerKd 0

//other constants
#define timeDelay 100 
#define pi 3.141592
#define RAD 111000
//motorOne (Right)
#define motor1_pwm 5//right
//motorTwo(Left)
#define motor2_pwm 6//left

#define GPSBAUD 9600
#define address 0x1E
//////////////////////////////////////////Change

#define DEBUG_MODE 0 // change 0 to stop serial printing.
#define USE_RF 0

#define OVERRIDE_PID_WITH_ON_OFF 0 // change 0 to stop overiding.
#define NO_OF_WAYPOINTS 2


//PID constants
#define motorKp 1
#define motorKi 0
#define motorKd 0

double baseSpeedLeft = 30;
double baseSpeedRight = 31;

double minSpeedLeft = 0;
double minSpeedRight = 0;

double maxSpeedLeft = 50;
double maxSpeedRight = 51;

////////////////////////////////////////////////
#define NO_OF_STATES 10
                  //id, lat, long, theta, alpha, x, y , z
double states[NO_OF_STATES] = {0.01, 0.05, 0.25,0.25,0,0,0,0,0,0};
double pwmL = 10, pwmR = 0;

//Global variables
double X,Y,theta,beta,alpha,distance,motorSetpoint,motorInput,motorOutput,propellerSetpoint,propellerInput,propellerOutput,betaSum,lastBeta;
double waypointLat, waypointLon;
int propellerSpeed;

int cmp_x,cmp_y,cmp_z;

float latLong[] [NO_OF_WAYPOINTS] = {
  //as many vals as dim1
 {14.078877, 100.614978}//first way point
 {14.078877, 100.614978}//change second waypoint
 
  };

  
int m=sizeof(latLong)/(sizeof(float)*2);
int j;

//Time Delay
unsigned long lastTime=0;
unsigned long lastDispTime=0;

// ESC variables
//Servo esc;
//Servo servo;

//PID variables

PID motorPID(&motorInput,&motorOutput,&motorSetpoint,motorKp,0,0, DIRECT);

PID propellerPID(&propellerInput,&propellerOutput,&propellerSetpoint,0.01,0,0,DIRECT);

//Compass variables
HMC5883L_Simple Compass;

//GPS variables
TinyGPS gps;
void getgps(TinyGPS &gps);


void setup() {
  Serial.begin(9600);
  
  initGPS();
  initCompass();
  initmotorPID();
  initpropellerPID();
  j=0;
  motorPID.SetOutputLimits(-90,90);

  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor2_pwm, OUTPUT);

  // RF

  radio.begin();
  radio.openWritingPipe(pipe);

  ////////// test for RF. 
  // remove this code after testing.
  analogWrite (motor1_pwm,0);
  analogWrite(motor2_pwm,0);

  //wait until gps is locking
  X = 0;
  do{
    currentPos();
    delay(200);
  }while(X < 0.001);
  ///
  
}

void clear_buf(){

  int i = 0;
  for(i = 0; i < 100; i++){
    radio_string[i] = '\0';
  }
}

double rf_tag_id = 0;
void sendStates(){
 
  rf_tag_id  = rf_tag_id + 1;
  states[0] = rf_tag_id;
  states[1] = X;
  states[2] = Y;
  states[3] = theta;
  states[4] = alpha;
  states[5] = pwmL;
  states[6] = pwmR;
  states[7] = cmp_z;
  states[8] = pwmL;
  states[9] = pwmR;

  if(DEBUG_MODE){
    Serial.println("\nstates");
    Serial.print(states[8]); Serial.print("-"); Serial.print(states[9]);
  }

  radio.write(states,sizeof(states));
}

void sendRF(){
 // radio_count[0] = strlen(radio_string);

 // Serial.print("\n count: "); Serial.println(radio_count[0]);

  //radio.write(radio_count, sizeof(radio_count));
  radio.write(radio_string, 100);

}


void goToWayPoint(int index){
  
  waypointLat=latLong[index][0];
  waypointLon= latLong[index][1];

  do{
 
    waypointLat=latLong[index][0];
    waypointLon= latLong[index][1];
 
  
    currentPos();
    getDistanceWaypoint();

    if(distance < 0.5){ // if boat is near, stop boat. and halt.
       /*while(1){
          pwmL = -100;
          pwmR = -100;

        
        sendStates();
  
        Serial.print("waypoint reached. stopeed \n");
     }*/
        analogWrite(motor2_pwm, 0);
        analogWrite(motor1_pwm, 0);
        return;
   }

    currentHead();
    getAlpha();
    getBeta();

    //changed : set PID variables from theta and alpha
  
    //modify theta and alpha, so that error is minimum

    if(abs(alpha - theta) > 180){

      if(alpha < theta){
        motorInput = theta;
        motorSetpoint = alpha + 360; 
      }else{
        motorInput = theta + 360;
        motorSetpoint = alpha;
      }

    }else{
      motorInput=theta;
      motorSetpoint=alpha; // 
    }

  
  //compute pid output
    motorPID.Compute();

    if(DEBUG_MODE){
      Serial.print("\n Motors : ");
      Serial.print(motorInput); Serial.print(":"); Serial.print(motorSetpoint); // comment serial printing
      Serial.print(":"); Serial.print(motorOutput);Serial.print("--> speeds L:");// when boat running.
    }
    int baseSpeed = 30; // change later.

    int pwmLeft  = baseSpeedLeft + motorOutput;
    int pwmRight = baseSpeedRight - motorOutput;

    pwmLeft = pwmLeft > minSpeedLeft ? pwmLeft : minSpeedLeft;
    pwmLeft = pwmLeft < maxSpeedLeft ? pwmLeft : maxSpeedLeft;

    pwmRight =  pwmRight > minSpeedRight ? pwmRight : minSpeedRight;
    pwmRight = pwmRight < maxSpeedRight ? pwmRight : maxSpeedRight;

 

  
    if(OVERRIDE_PID_WITH_ON_OFF){

      if(motorOutput > 0){
        pwmLeft = baseSpeedLeft;
        pwmRight = 25; //set other motoro speed
      }else if(motorOutput == 0){
        pwmLeft = baseSpeedLeft;
        pwmRight = baseSpeedRight;

      }else if(motorOutput < 0){
        pwmLeft = 25; // set other motor speed.
        pwmRight = baseSpeedRight;
      }


    }

    if(DEBUG_MODE){
      Serial.print(pwmLeft); Serial.print(" R:"); Serial.println(pwmRight);
    }

    pwmL = pwmLeft;
    pwmR = pwmRight;

    analogWrite(motor2_pwm, pwmLeft);
    analogWrite(motor1_pwm, pwmRight);

  

    if(USE_RF){
      sendStates();
    }

  //delay(10); // reduce 500 to a low value when testing with boat.
 
  }while(distance < 0.5 /* change 10 */); 
  analogWrite(motor2_pwm, 0);
  analogWrite(motor1_pwm, 0);

}
int idx_1;
void loop() {
  
  for(idx_1 = 0; idx_1 < NO_OF_WAYPOINTS; idx_1++ ){
    goToWayPoint(idx_1);

    delay(200);
  }

  while(1){
    Serial.println("reached waypoints.");
    delay(100);
  }

  
}


void initGPS(){
  Serial1.begin(GPSBAUD);
  Serial.println("waiting for signal");
}

void getgps(TinyGPS &gps)
{
  float gpsX, gpsY;
  gps.f_get_position(&gpsX, &gpsY);

  //Serial.println("gps_x, gps_Y "); Serial.print(gpsX); Serial.print("- "); Serial.print(gpsY);
  X =(double)gpsX;
  Y =(double)gpsY;

  //Serial.print("gps_Lat/Long: "); 
 // Serial.print(X,5); 
  //Serial.print(", "); 
 // Serial.println(Y,5);


}

void initCompass(){
 
  Wire.begin();

  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
  
  
}

void initmotorPID(){
   motorInput=theta;
   motorSetpoint=alpha;
   motorPID.SetMode(AUTOMATIC);

}

void initpropellerPID(){
  propellerInput=sqrt(pow(X,2)+pow(Y,2));
  propellerSetpoint=sqrt(pow(waypointLat,2)+pow(waypointLon,2));
  //propellerInput=distance;
  //propellerSetpoint=1;
  
  propellerPID.SetMode(AUTOMATIC); 
}



void currentPos(){
  //Read the serial port to see if GPS data is available
  while(Serial1.available()){
    char c = Serial1.read();
    if(gps.encode(c)){
      getgps(gps);
    }
  }
}

double distanceBetween(double lat1, double long1, double lat2, double long2)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(long1-long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}

void getDistanceWaypoint(){
 //currentPos();
  if(DEBUG_MODE){
    Serial.println("Lat/Long: "); 
    Serial.print(X,5); 
    Serial.print(", "); 
    Serial.println(Y,5);
    Serial.println("Waypoint Lat/Lon");
    Serial.print(waypointLat,5); 
    Serial.print(", "); 
    Serial.println(waypointLon,5);
  }
  
  distance=sqrt(pow((X-waypointLat),2)+pow((Y-waypointLon),2))*RAD;

  if(DEBUG_MODE){
    Serial.print("\ndistance to waypoint:");
    Serial.print(distance,6);Serial.print(",: ");
 }

  double dist2 = distanceBetween(X, Y, waypointLat, waypointLon);

  if(DEBUG_MODE){
    Serial.println(dist2,6);
  }
}

void currentHead(){

  int x,y,z; //triple axis data

  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();


  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }

    theta =(atan2(-y,x)/3.141*180)+90;
    if(theta<0){
      theta=360+theta;
    }
  //Print out values of each axis

   if(DEBUG_MODE){ 
    Serial.println("\ntheta ");
    Serial.print(theta);
    Serial.print("x: ");

  Serial.print(x);
  Serial.print("  y: ");
  Serial.print(y);
  Serial.print("  z: ");
  Serial.println(z);

  cmp_x = x;
  cmp_y = y;
  cmp_z = z;
 }
  //delay(250);
 
}

void getAlpha(){
  
  alpha=atan2((X-waypointLat),(waypointLon-Y));
  //14.078931, 100.614949 Swimming pool  //14.081073, 100.609873 ISE
  //alpha=atan2((14.081073-14.078931),(100.609873-100.614949));
  
  alpha= (360/(2*pi))*alpha;
 

  alpha = alpha + 90; // measure from north.

  if(alpha < 0) alpha = alpha + 360; // make to 0 - 360 range.

}

void getBeta(){

  beta=alpha-theta;
}

void pidMotor(){
  motorPID.Compute();
  //turn(motorOutput,propellerSpeed); 
  
}

void pidPropeller(){
  
 
  propellerPID.Compute();
  propellerSpeed=map(propellerOutput,0,255,50,150);
  //Serial.print("Propeller Output");

  
  
 if (distance<=1){
  propellerSpeed=0;
 }
  
  

}

void display1(){
  unsigned long now=millis();
  unsigned long timeChange= now-lastDispTime;  

  if(timeChange > 1000){

   // Serial.print("distance to waypoint:");
   /// Serial.println(distance);
    
   // Serial.print("Alpha:");
  //  Serial.println(alpha,5);
    
   /// Serial.print("Beta:");
   // Serial.println(beta,5);
    
    lastDispTime=millis();
  }
}
