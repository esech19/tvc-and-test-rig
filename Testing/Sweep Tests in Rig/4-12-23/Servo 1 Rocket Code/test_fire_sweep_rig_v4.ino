#include <Arduino_LSM9DS1.h>
#include <I2Cdev.h>
#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>

float servoIncrement = 12.5; //time in ms between position commands sent to servo in increments of 1 to run at 80 Hz
int runs = 50; //number of loops to allocate space for 
int dataEntries = 84; //number of lines of servo positions
float times[4201]; //84*50=4200 + 0th index = 4201
float servoPositions[4201]; //84*50=4200 + 0th index = 4201
int arraypos = 1;
 int lines = 4201; //total number of lines to dump to SD in every test file after dynamic mem runs out
 
Servo servoX; //X AXIS SERVO
Servo servoY; //Y AXIS SERVO
File myFile;
int servo_pinX = 9; //servo 1 is pin 9
int servo_pinY = 8; //servo 2 is pin 8
int servoStartX = 143; //servo 1 is 143 for 0 pos
int servoStartY = 73; //servo 2 is 73 for 0 pos
const int chipSelect = 4;
int dataLog = 0;

int pos = 0;
String c = ",";
float timeStamp = 0;
float tstart = 0;
float tinit = 0; //time at power on
int fileNum = 0;


 
void setup() {
  servoX.attach(9);
  servoY.attach(8);
  servoX.write(servoStartX);
  servoY.write(servoStartY);
  SD.begin(chipSelect);
 
}


void loop ( ) {
  times[0] = 0;
  tinit = micros();
  servoX.write(servoStartX);
  servoY.write(servoStartY);
  for (int i=0; i <= runs; i++){
    sweepTest2_X();
  }
  while (SD.exists("test_" + String(fileNum) + ".txt")) {
    fileNum = fileNum + 1;
  }
  myFile = SD.open("test_" + String(fileNum) + ".txt", FILE_WRITE);
  myFile.println("Start Time: " + String(times[0]));

  for (int j = 1; j <=lines; j++){
      myFile.println(times[j] + c + servoPositions[j]);
  }
  myFile.close();
  arraypos = 1;
}

void sweepTest2_X() { 
  for (pos = servoStartX; pos <= servoStartX+20; pos += 1) { 
    servoY.write(servoStartY);
    servoX.write(pos); 
    servoPositions[arraypos] = pos;
    times[arraypos] = (micros()-tinit+servoIncrement*1000)/1000;   
    delay(servoIncrement);   
    arraypos++;                
  }
  for (pos = servoStartX+20; pos >= servoStartX; pos -= 1) { 
    servoY.write(servoStartY);
    servoX.write(pos); 
    servoPositions[arraypos] = pos;
    times[arraypos] = (micros()-tinit+servoIncrement*1000)/1000;
    delay(servoIncrement);  
    arraypos++;                                 
  }
  for (pos = servoStartX; pos >= servoStartX-20; pos -= 1) { 
    servoY.write(servoStartY);
    servoX.write(pos);
    servoPositions[arraypos] = pos;
    times[arraypos] = (micros()-tinit+servoIncrement*1000)/1000;
    delay(servoIncrement);  
    arraypos++;                          
  }
  for (pos = servoStartX-20; pos <= servoStartX-1; pos += 1) { 
    servoY.write(servoStartY);
    servoX.write(pos); 
    servoPositions[arraypos] = pos;
    times[arraypos] = (micros()-tinit+servoIncrement*1000)/1000;
    delay(servoIncrement);   
    arraypos++;                                  
  }
}
