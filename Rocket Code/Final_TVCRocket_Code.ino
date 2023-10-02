/*
 * Thrust Vectoring Control (TVC) Rocket             ^
 * Carnegie Mellon University                       / \
 * Build 18 2022 Track 1                           | T |
 * Ian Turner and Elijah Sech                      | V |
 *                                                 | C |
 *                                                 |   |
 *                                                 |   |
 *                                                 -- --
 *                                                  /*\
 *                                                 *****
 *                                                *******
 */
#include <MPU6050.h>
#include <I2Cdev.h>
#include <Wire.h>
#include <Servo.h>
#include <SPI.h>

//creating two servo instances
Servo servoX; //X AXIS SERVO, top black w/ yellow link 
Servo servoY; //Y AXIS SERVO, bottom blue w/ black link

//create sensor instance
MPU6050 sensor ; //sensor

//assigning pins
int servo_pinX = 2; 
int servo_pinY = 3;
int blue = 6; //LED PINS
int green = 5;
int red = 4;

//Servo variables
int angleX = 0; //ServoY | values for desired angle output
int angleY = 0; //servoX
int angleXoffset = 54;
int angleYoffset = 164;
int servoStartX = 54; //values for 'dead center' pre-launch
int servoStartY = 164;
int pos; //current position of servo
double X_linkage_offset = 7; 
double Y_linkage_offset = 6; //offset for ratio between linkage and servo movement
                                                                              
//PID varibales
double delta_t, currentTime, previousTime; 
float pidX_p = 0; //PID variables
float pidX_i = 0;
float pidX_d = 0;
float pidY_p = 0;
float pidY_i = 0;
float pidY_d = 0;
double kp = 0.11; //constants for PID equation
double ki = 0.04;
double kd = 0.025;
int16_t ax, ay, az ; //accelerometer values
int16_t gx, gy, gz ; //gyroscope values
double PIDX, PIDY, current_errorX, current_errorY, previous_errorX, previous_errorY, outputX, outputY, pitch, roll;
double gyroAngleX;
double gyroAngleY;
double accAngleX;
double accAngleY;

bool isPID_first = false;
double PIDX_first = 0;
double PIDY_first = 0;

void setup ( )
{
  Wire.begin ( );
  Serial.begin  (9600);
  Serial.println  ( "Initializing the sensor" );
  sensor.initialize ( );
  servoX.attach (servo_pinX );
  servoY.attach (servo_pinY);
  servoX.write(servoStartX);
  servoY.write(servoStartY);
  pinMode(blue, OUTPUT);
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  Serial.println (sensor.testConnection ( ) ? "Successfully Connected" : "Connection failed");
  Serial.println ( "Taking Values from the sensor" );
  initialize();
  loop();
  isPID_first = false;
  Serial.println(PIDX_first);
  Serial.println(PIDY_first);
}



void loop ( )
{
  previousTime = currentTime;
  currentTime = millis();
  delta_t = (currentTime - previousTime) / 1000;
  //Serial.println("TIME:");
  //Serial.println(delta_t);
  sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  accAngleX = (atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * ax / sqrt(pow(ay, 2) + pow(az, 2))) * 180 / PI) + 1.58; //

  // pitch = map (gx, -17000, 17000, 15, 96) ;
  // roll = map (gy, -17000, 17000, 20, 150) ;
  //Serial.println ("gyro values");
  //Serial.println (gx);
  //Serial.println (gy);
  //Serial.println (gz);
  //Serial.println ("acceleration values");
  //Serial.println (ax);
  //Serial.println (ay);
  //Serial.println (az);

  PID();
  //Serial.println(gx);
  //Serial.println(outputY);
  //Serial.print(",");
  //Serial.println(outputX);
  
  if ((angleXoffset-50 < outputX) && (outputX < angleXoffset+50)){
      servoX.write(outputX);
  }
  if ((angleYoffset-26 < outputY) && (outputY < angleYoffset+26)){
      servoY.write(outputY);
  }
}



void PID() {

  double prevgyroX = gx;
  double prevgyroY = gy;
  
  //converting angular acceleration to degrees
  gyroAngleX = (((gx + prevgyroX) / 2) * delta_t); // deg/s * s = deg
  gyroAngleY = (((gy + prevgyroY) / 2) * delta_t);

  double OrientationX = 0.94 * gyroAngleX + 0.06 * accAngleX;
  double OrientationY = 0.94 * gyroAngleY + 0.06 * accAngleY;

  //divided by 32.8 as recommended by the datasheet
  pitch = OrientationX / 32.8;
  roll = OrientationY / 32.8;
  previous_errorX = current_errorX;
  previous_errorY = current_errorY;
  current_errorX = pitch - angleX;
  current_errorY = roll - angleY;
  pidX_p = kp * current_errorX;
  pidY_p = kp * current_errorY;
  pidX_d = kd * ((current_errorX - previous_errorX) / delta_t);
  pidY_d = kd * ((current_errorY - previous_errorY) / delta_t);
  pidX_i = ki * (pidX_i + current_errorX * delta_t);
  pidY_i = ki * (pidY_i + current_errorY * delta_t);
  PIDX = pidX_p + pidX_i + pidX_d; //pid loop for each x and y
  PIDY = pidY_p + pidY_i + pidY_d;
  Serial.println('x');
  Serial.println(PIDX);
  Serial.println('y');
  Serial.println(PIDY);
  outputX = (((PIDX-PIDX_first) * X_linkage_offset) + angleXoffset);
  outputY = (((PIDY-PIDY_first) * Y_linkage_offset) + angleYoffset);
  Serial.println(outputX);
  Serial.println(outputY);

  if (isPID_first == true){
    Serial.println("FIRST VALUE SET");
    PIDX_first = PIDX;
    PIDY_first = PIDY;
  }
}



void initialize() {

  servoX.write(servoStartX);
  digitalWrite(green, HIGH);
  servoX.write(servoStartX);
  servoY.write(servoStartY);
  delay(1000);
  digitalWrite(green, LOW);
  digitalWrite(red, HIGH);
  servoX.write(servoStartX + 20);
  delay(200);
  servoY.write(servoStartY + 20);
  delay(200);
  digitalWrite(red, LOW);
  digitalWrite(blue, HIGH);
  servoX.write(servoStartX);
  delay(200);
  servoY.write(servoStartY);
  delay(200);
  digitalWrite(blue, LOW);
  digitalWrite(green, HIGH);
  servoX.write(servoStartX - 20);
  delay(200);
  servoY.write(servoStartY - 20);
  delay(200);
  servoX.write(servoStartX);
  delay(200);
  servoY.write(servoStartY);
  delay(500);

}
