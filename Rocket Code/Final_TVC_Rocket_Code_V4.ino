#include <Arduino_LSM9DS1.h> //nano ble IMU sensor
#include <Servo.h>  
#include <SPI.h> //micro sd comms

//BMP390 (altimeter) stuff
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#define SEALEVELPRESSURE_HPA (1019.98) //set current sea level pressure of day in hPa for launch location
Adafruit_BMP3XX bmp;
float INIT_HEIGHT; //initial height
float INIT_BMP_ERROR = 1.5;
bool BMP_initialized = false;
float DELTA_HEIGHT = 0; //change in height from last loop iteration that's constantly calculaated
float H_0 = 0;
float H;
int fallcount = 0;

//LED pins on Arduino Nano BLE
int RED = 22;
int BLUE = 24;
int GREEN = 23;
int LEDPWR = 25;

//Gimbal Servo setup and variables for PID
Servo servoX; //X AXIS SERVO   
Servo servoY; //Y AXIS SERVO  
int servo_pinX = 9; //servo 1
int servo_pinY = 8; //servo 2
int angleX = 0; //ServoY | values for desired angle output
int angleY = 0; //servoX 
int angleXoffset = 145; //servo 1
int angleYoffset = 70; //servo 2
int servoStartX = angleXoffset; //values for 'dead center' pre-launch (This is where gimbal is at 0,0 position.)
int servoStartY = angleYoffset;
//int linkage_offset = 5; //offset for ratio between linkage and servo movement
float pidX_p = 0; //
float pidX_i = 0;
float pidX_d = 0;
float pidY_p = 0;
float pidY_i = 0;
float pidY_d = 0;
int pos;
double kp = 10; //constants for PID equation
double ki = .1;
double kd = .5;
float ax, ay, az ; //accelerometer values
float gx, gy, gz ; //gyroscope values
double PIDX, PIDY, current_errorX, current_errorY, previous_errorX, previous_errorY, outputX, outputY, pitch, roll;
double gyroAngleX;
double gyroAngleY;
double accAngleX;
double accAngleY;
double xAdj = 0;
double yAdj = 0;
int range = 40;
double prevgyroX = 0;
double prevgyroY= 0;

//loop iteration counter
int i = 0;
int curr_i=0; //to record for bmp averaging

//timing variables
double delta_t, currentTime, initialTime;
double thrust_start_time, thrust_timer;
double burn_time = 2.4;
int apogee_delay = 200; //ms after apogee detected or rocket has stopped buring to release chute
double kill_delay = 1.5; //seconds after theoretical burn time to kill rocket and deploy the chute

//parachute depolyment setup
Servo chuteServo;
bool chuteDeployed = false;
double apogee_height;
bool apogee_detected = false;
bool liftoff = false;

void setup ( ) {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Taking Values from the sensor");
  //setup all pins on board 
  servoX.attach(servo_pinX);
  servoY.attach(servo_pinY);
  chuteServo.attach(7); //pin that the parachute deployment servo is attached to
  servoX.write(servoStartX);
  servoY.write(servoStartY);
  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(LEDPWR, OUTPUT);
 
  // Set up oversampling and filter initialization for BMP390
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);
  Wire.begin();
  if (!bmp.begin_I2C() ||  !IMU.begin()) {
    digitalWrite(RED, LOW); //if the altimiter or IMU have trouble establishing communication flash red led, green if all good
    digitalWrite(GREEN, HIGH);
    digitalWrite(BLUE, HIGH);
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while(1);
  }
  else {
    digitalWrite(GREEN, LOW);
    digitalWrite(RED, HIGH);
    digitalWrite(BLUE, HIGH);
    Serial.println("Sensors are good to go");
    
  }
  if (! bmp.performReading()) {
    digitalWrite(RED, LOW);
    digitalWrite(BLUE, HIGH);
    digitalWrite(GREEN, HIGH);
    Serial.println("Failed to perform reading :");
    while(1);
  }
  initialize(); //startup sweep function for servos
  initialTime = millis();
}
 
void loop ( ) { 
  currentTime = millis();
  delta_t = (currentTime - initialTime) / 1000;
//  Serial.print("Delta Time:");
//  Serial.println(delta_t);
  
//  PID();
  //Serial.print(outputY);
  //Serial.print(",");
  //Serial.print(outputX);
  servoX.write(outputX); 
  servoY.write(outputY);
  
  if ((delta_t > 5.0) && (delta_t < 5.03)) {
//    Serial.println("Curr_i is set");
    curr_i = i;
  }
  
  if (delta_t > 5.0) {
//    if ((i >= curr_i) && (i < curr_i+50)) {
//      bmp.readAltitude(SEALEVELPRESSURE_HPA);
////      Serial.println("Rocket is settled");
//    }
    if ((i >= curr_i+50) && (i < curr_i+100)) {
    INIT_HEIGHT += bmp.readAltitude(SEALEVELPRESSURE_HPA);
//    Serial.println("BMP is initializing");
//    Serial.print("Unaveraged initial height: ");
//    Serial.print(INIT_HEIGHT);
//    delay(1000);
    }
    if (curr_i+100 == i) {
      INIT_HEIGHT = INIT_HEIGHT/50;
      BMP_initialized = true;
     Serial.print("INITIAL HEIGHT:                  ");
     Serial.println(INIT_HEIGHT);
     delay(1000);
    }
  }
  H = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.print("Current height H: ");
  Serial.println(H);
  Serial.print("Previous height H0: ");
  Serial.println(H_0);
  DELTA_HEIGHT = H - H_0;
  H_0 = H;
  if (BMP_initialized) {
    Serial.println("Sensor initialized");
    if (bmp.readAltitude(SEALEVELPRESSURE_HPA) > (INIT_HEIGHT + INIT_BMP_ERROR)){
      Serial.println("Sensor above initial height");
      if (!liftoff){
        thrust_start_time = millis();
        liftoff = true;
      }
      thrust_timer = (millis() - thrust_start_time)/1000;
      Serial.print("Time since liftoff: ");
      Serial.println(thrust_timer);
      if (!(apogee_detected) && (DELTA_HEIGHT < -.5)) {
        Serial.println("Sensor apogee");
         apogee_height = bmp.readAltitude(SEALEVELPRESSURE_HPA) ;
         apogee_detected = true; 
        }
      if (DELTA_HEIGHT < -.7) {
        Serial.println("Sensor is falling");
        fallcount++;
      }
      else {
        Serial.println("Sensor not falling");
        fallcount = 0;
      }
      if ((fallcount > 10) || (thrust_timer > (burn_time + kill_delay)));
        Serial.println("deploy");
        DEPLOY_CHUTE();
    }
  }
  i++;
}

//needs to be tuned for the correct range and speed
void DEPLOY_CHUTE() {
  for (pos = 110; pos <= 180; pos += 1) { 
    // in steps of 1 degree
    chuteServo.write(pos);           
    delay(5);                      
  }
  for (pos = 180; pos >= 110; pos -= 1) { 
    chuteServo.write(pos);             
    delay(5);                       
  }
}

void PID() {
  
  IMU.readGyroscope(gz,gy,gx);
  IMU.readAcceleration(az,ay,ax);

  accAngleX = (atan(ay / sqrt(pow(az, 2) + pow(ax, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) S
  accAngleY = (atan(-1 * ax / sqrt(pow(ay, 2) + pow(az, 2))) * 180 / PI) + 1.58; // 
  
  //converting angular acceleration to degrees
  gyroAngleX += ((((gx + .286) + (prevgyroX+.286)) / 2)* delta_t); // deg/s * s = deg, //gyro values are cumulative.
  gyroAngleY += ((((gy + prevgyroY)-1.362)/ 2)* delta_t);
 
  double OrientationX = 0 * gyroAngleX + 1*accAngleX - xAdj;
  double OrientationY = 0 * gyroAngleY + 1*accAngleY - yAdj; //complementary filters to calculate orientation (combines gyro + accel)

  Serial.print("GyroX: \t");
  Serial.print(gyroAngleX);
  Serial.print("\tGyroY: \t");
  Serial.print(gyroAngleY);
  Serial.print("\t");
  pitch = OrientationX;
  roll = OrientationY;
  
  previous_errorX = current_errorX;
  previous_errorY = current_errorY;
 
  current_errorX = pitch - angleX;
  current_errorY = roll - angleY;
 
  pidX_p = kp*current_errorX;
  pidY_p = kp*current_errorY;
  
  pidX_d = kd*((current_errorX - previous_errorX)/delta_t);
  pidY_d = kd*((current_errorY - previous_errorY)/delta_t);
 
  pidX_i = ki * (pidX_i + current_errorX * delta_t);
  pidY_i = ki * (pidY_i + current_errorY * delta_t);
 
  PIDX = pidX_p + pidX_i + pidX_d; //pid loop for each x and y
  PIDY = pidY_p + pidY_i + pidY_d;

  if (angleXoffset-range > PIDX + angleXoffset){
    outputX = angleXoffset-range;
  }
  else if (angleXoffset+range < PIDX + angleXoffset){
    outputX = angleXoffset + range;
  
  }
  else{
    outputX = ((PIDX + angleXoffset));
  }

  if (angleYoffset-range > PIDY + angleYoffset){
    outputY = angleYoffset-range;
  }
  else if (angleYoffset+range < PIDY + angleYoffset){
    outputY = angleYoffset + range;
  }
  else{
    outputY = ((PIDY + angleYoffset));
  }
  Serial.print("OutputX: \t");
  Serial.print(outputX);
  Serial.print("OutputY: \t");
  Serial.print(outputY);
  Serial.print("\n");
//  xAdj = xAdj + .000004;
//  yAdj = yAdj + .002;

  prevgyroX = gx;
  prevgyroY = gy;
}
 
void initialize() {

  servoX.write(servoStartX);
  servoY.write(servoStartY);
  
  delay(200);
  servoX.write(servoStartX + 40);
  delay(200);
  servoY.write(servoStartY + 40);
  delay(200);

  servoX.write(servoStartX);
  delay(200);
  servoY.write(servoStartY);
  delay(200);

  servoX.write(servoStartX - 40);
  delay(200);
  servoY.write(servoStartY - 40);
  delay(200);
  servoX.write(servoStartX);
  delay(200);
  servoY.write(servoStartY);
  delay(500);
}
