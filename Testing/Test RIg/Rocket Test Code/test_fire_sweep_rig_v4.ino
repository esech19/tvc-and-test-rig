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
int servo_pinX = 9; //servo 1
int servo_pinY = 8; //servo 2
int servoStartX = 143;
int servoStartY = 73;
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

//void sweepTest2_Y() {
//  if (i==0) {
//    while (SD.exists("test_" + String(fileNum) + ".txt")) {
//        fileNum = fileNum + 1;
//      }
//  }
//  myFile = SD.open("test_" + String(fileNum) + ".txt", FILE_WRITE);
//  tstart = micros();
//  servoX.write(servoStartX);
//  myFile.println("Servo Y Run: " + String(i));
//  servoY.write(servoStartY);
//  timeStamp = timeStamp + servoIncrement;
//  myFile.println(timeStamp + c + servoStartY);
//  delay(servoIncrement -(micros() - tstart - (tstart-tstart2)));
//  
//  for (pos = servoStartY; pos <= servoStartY+20; pos += 1) { 
//    tstart = micros();
//    servoX.write(servoStartX);
//    servoY.write(pos); 
//    timeStamp = timeStamp + servoIncrement;
//    myFile.println(timeStamp + c + pos);      
//    delay(servoIncrement -(micros() - tstart));                     
//  }
//  for (pos = servoStartY+20; pos >= servoStartY; pos -= 1) { 
//    tstart = micros();
//    servoX.write(servoStartX);
//    servoY.write(pos); 
//    timeStamp = timeStamp + servoIncrement;
//    myFile.println(timeStamp + c + pos);                 
//    delay(servoIncrement -(micros() - tstart)); 
//                          
//  }
//   for (pos = servoStartY; pos >= servoStartY-20; pos -= 1) { 
//    tstart = micros();
//    servoX.write(servoStartX);
//    servoY.write(pos);  
//    timeStamp = timeStamp + servoIncrement;
//    myFile.println(timeStamp + c + pos);             
//    delay(servoIncrement -(micros() - tstart));                    
//  }
//  for (pos = servoStartY-20; pos <= servoStartY-1; pos += 1) { 
//    tstart = micros();
//    servoX.write(servoStartX);
//    servoY.write(pos); 
//    timeStamp = timeStamp + servoIncrement;
//    myFile.println(timeStamp + c + pos);                 
//    delay(servoIncrement -(micros() - tstart));                       
//  }
//  tstart2 = micros();
//  myFile.close();
//}
//
//void sweepTest1() {
//  servoX.write(servoStartX);
//  delay(50);
//  servoY.write(servoStartY);
//  delay(500);
//  servoX.write(servoStartX + 20);
//  delay(500);
//  servoX.write(servoStartX);
//  delay(500);
//  servoX.write(servoStartX - 20);
//  delay(500);
//  servoX.write(servoStartX);
//  delay(500);
//}
//void sweepSquare() {
//  servoX.write(servoStartX);
//  delay(200);
//  servoY.write(servoStartY);
//  delay(200);
//  servoX.write(servoStartX - 20);
//  servoY.write(servoStartY + 20);
//  delay(200);
//  servoX.write(servoStartX + 20);
//  delay(200);
//  servoY.write(servoStartY + 20);
//  delay(200);
//  servoX.write(servoStartX + 20);
//  delay(200);
//  servoY.write(servoStartY - 20);
//  delay(200);
//  servoX.write(servoStartX - 20);
//  delay(200);
//  servoY.write(servoStartY - 20);
//  delay(200);
//  servoX.write(servoStartX - 20);
//  delay(200);
//  servoY.write(servoStartY + 20);
//  delay(200);
//  servoX.write(servoStartX);
//  delay(200);
//  servoY.write(servoStartY);
//}
