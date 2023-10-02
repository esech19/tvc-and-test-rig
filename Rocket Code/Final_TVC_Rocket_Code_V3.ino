#include <Arduino_LSM9DS1.h> //nano ble IMU sensor
#include <Servo.h>  
#include <SPI.h> //micro sd comms

//BMP390 (altimeter) stuff
#include <I2Cdev.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#define SEALEVELPRESSURE_HPA (1019.98) //set current sea level pressure of day in hPa for launch location
Adafruit_BMP3XX bmp;
float INIT_BMP;
float INIT_BMP_ERROR = 1.0;
float DESC_BMP_ERROR = 2.0;
bool BMP_initialized = false;

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
int curr_i; //to record for bmp averaging

//timing variables
double delta_t, currentTime, previousTime;
double prev_thrust_timer, curr_thrust_timer, thrust_timer;
double burn_time = 2.4;
int apogee_delay = 500; //ms after apogee detected or rocket has stopped buring to release chute
double kill_delay = 1.5; //seconds after theoretical burn time to kill rocket and deploy the chute

//parachute depolyment setup
Servo chuteServo;
bool chuteDeployed = false;
double apogee_height;
bool apogee_detected = true;

void setup ( ) {
//   Serial.begin(115200);
//   Serial.println ( "Taking Values from the sensor" );
   
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
 
  if (!bmp.begin_I2C() || Wire.begin () ||  IMU.begin()) {
    digitalWrite(RED, HIGH); //if the altimiter or IMU have trouble establishing communication flash red led, green if all good
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while(1);
  }
  else {
    digitalWrite(GREEN, HIGH);
    Serial.println("Sensors are good to go");
  }
  if (! bmp.performReading()) {
    digitalWrite(RED, HIGH);
    Serial.println("Failed to perform reading :(");
    while(1);
  }
  initialize(); //startup sweep function for servos
}
 
void loop ( ) { 
 
  previousTime = currentTime;
  currentTime = millis();
  delta_t = (currentTime - previousTime) / 1000;
  
  PID();
  //Serial.print(outputY);
  //Serial.print(",");
  //Serial.print(outputX);
  servoX.write(outputX); 
  servoY.write(outputY);
  
  if delata_t = 5.0{
    curr_i = i;
  }
  
  if delta_t > 5.0{
    if i < curr_i+50 {
      bmp.readAltitude(SEALEVELPRESSURE_HPA);
    }
    else{
      if i < curr_i+100 {
      INIT_BMP += bmp.readAltitude(SEALEVELPRESSURE_HPA);
      }
    }
    if i = 100 {
      INIT_BMP = INIT_BMP/50;
      BMP_initialized = true;
    }
  }
  if BMP_initialized {
    if bmp.readAltitude(SEALEVELPRESSURE_HPA) > (INIT_BMP + INIT_BMP_ERROR){
      prev_thrust_timer = curr_thrust_timer;
      curr_thrust_timer = millis();
      thrust_timer = curr_thrust_timer - prev_thrust_timer;
      if thrust_timer > burn_time {
        if apogee_detected {
           bmp.readAltitude(SEALEVELPRESSURE_HPA) = apogee_height;
           apogee_detected = false;
        }
          if bmp.readAltitude(SEALEVELPRESSURE_HPA) < apogee_height - DESC_BMP_ERROR {
            delay(apogee_delay); 
            DEPLOY_CHUTE();
          }
      }
      if thrust_timer > (burn_time + kill_delay){
        DEPLOY_CHUTE();
      }
    }
  }
  
  i++;
}

//needs to be tuned for the correct range and speed
void DEPLOY_CHUTE() {
  for (pos = 70; pos <= 180; pos += 10) { 
    // in steps of 1 degree
    myservo.write(pos);           
    delay(10);                      
  }
  for (pos = 180; pos >= 70; pos -= 10) { 
    myservo.write(pos);             
    delay(10);                       
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
