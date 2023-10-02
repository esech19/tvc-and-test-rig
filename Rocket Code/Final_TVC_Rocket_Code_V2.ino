#include <Arduino_LSM9DS1.h>
#include <I2Cdev.h>
#include <Wire.h>
#include <Servo.h>  
#include <SPI.h> 

 
Servo servoX; //X AXIS SERVO   
Servo servoY; //Y AXIS SERVO  
 
int servo_pinX = 9; //servo 1
int servo_pinY = 8; //servo 2

int REDPIN = 22;
int BLUEPIN = 24;
int GREEN = 23;
int LEDPWR = 25;
 
int angleX = 0; //ServoY | values for desired angle output
int angleY = 0; //servoX 
 
int angleXoffset = 145; //servo 1
int angleYoffset = 70; //servo 2
 
int servoStartX = angleXoffset; //values for 'dead center' pre-launch (This is where gimbal is at 0,0 position.)
int servoStartY = angleYoffset;
 
//int linkage_offset = 5; //offset for ratio between linkage and servo movement
 
double delta_t, currentTime, previousTime;
 
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
 
void setup ( )
 
{ 
 
Wire.begin ();
Serial.begin  (9600); 
IMU.begin();
 
 
servoX.attach(servo_pinX);
servoY.attach(servo_pinY);
servoX.write(servoStartX);
servoY.write(servoStartY);
 
Serial.println ( "Taking Values from the sensor" );

initialize();
 
}
 
void loop ( ) 
 
{ 
 
previousTime = currentTime;
currentTime = millis();
delta_t = (currentTime - previousTime) / 1000;


 
PID();

//Serial.print(outputY);
//Serial.print(",");
//Serial.print(outputX);
 
servoX.write(outputX); 
servoY.write(outputY);
 
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
