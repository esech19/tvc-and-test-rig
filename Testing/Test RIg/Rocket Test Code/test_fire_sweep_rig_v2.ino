#include <Arduino_LSM9DS1.h>
#include <I2Cdev.h>
#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>

Servo servoX; //X AXIS SERVO
Servo servoY; //Y AXIS SERVO
File myFile;
int servo_pinX = 9; //servo 1
int servo_pinY = 8; //servo 2
int servoStartX = 143;
int servoStartY = 73;
const int chipSelect = 4;
int dataLog = 0;
int servoIncrement = 10; //ms
int pos = 0;
String c = ",";
int timeStamp = 0;
int fileNum = 0;
int i = 0;
double tstart = 0;
double tstart2 = 0;

void setup() {
  servoX.attach(9);
  servoY.attach(8);
  servoX.write(servoStartX);
  servoY.write(servoStartY);
  SD.begin(chipSelect);
}

void loop ( ) {
  sweepTest2_X();
  i++;
}

void sweepTest2_X() {
  if (i==0) {
    while (SD.exists("test_" + String(fileNum) + ".txt")) {
        fileNum = fileNum + 1;
      }
  }
  myFile = SD.open("test_" + String(fileNum) + ".txt", FILE_WRITE);
  tstart = micros();
  servoX.write(servoStartX);
  myFile.println("Servo X Run: " + String(i));
  servoY.write(servoStartY);
  timeStamp = timeStamp + servoIncrement;
  myFile.println(timeStamp + c + servoStartX);
  delay(servoIncrement -(micros() - tstart - (tstart-tstart2)));
  
  for (pos = servoStartX; pos <= servoStartX+20; pos += 1) { 
    tstart = micros();
    servoY.write(servoStartY);
    servoX.write(pos); 
    timeStamp = timeStamp + servoIncrement;
    myFile.println(timeStamp + c + pos);      
    delay(servoIncrement -(micros() - tstart));                     
  }
  for (pos = servoStartX+20; pos >= servoStartX; pos -= 1) { 
    tstart = micros();
    servoY.write(servoStartY);
    servoX.write(pos); 
    timeStamp = timeStamp + servoIncrement;
    myFile.println(timeStamp + c + pos);                 
    delay(servoIncrement -(micros() - tstart)); 
                          
  }
   for (pos = servoStartX; pos >= servoStartX-20; pos -= 1) { 
    tstart = micros();
    servoY.write(servoStartY);
    servoX.write(pos);  
    timeStamp = timeStamp + servoIncrement;
    myFile.println(timeStamp + c + pos);             
    delay(servoIncrement -(micros() - tstart));                    
  }
  for (pos = servoStartX-20; pos <= servoStartX-1; pos += 1) { 
    tstart = micros();
    servoY.write(servoStartY);
    servoX.write(pos); 
    timeStamp = timeStamp + servoIncrement;
    myFile.println(timeStamp + c + pos);                 
    delay(servoIncrement -(micros() - tstart));                       
  }
  tstart2 = micros();
  myFile.close();
}

void sweepTest2_Y() {
  if (i==0) {
    while (SD.exists("test_" + String(fileNum) + ".txt")) {
        fileNum = fileNum + 1;
      }
  }
  myFile = SD.open("test_" + String(fileNum) + ".txt", FILE_WRITE);
  tstart = micros();
  servoX.write(servoStartX);
  myFile.println("Servo Y Run: " + String(i));
  servoY.write(servoStartY);
  timeStamp = timeStamp + servoIncrement;
  myFile.println(timeStamp + c + servoStartY);
  delay(servoIncrement -(micros() - tstart - (tstart-tstart2)));
  
  for (pos = servoStartY; pos <= servoStartY+20; pos += 1) { 
    tstart = micros();
    servoX.write(servoStartX);
    servoY.write(pos); 
    timeStamp = timeStamp + servoIncrement;
    myFile.println(timeStamp + c + pos);      
    delay(servoIncrement -(micros() - tstart));                     
  }
  for (pos = servoStartY+20; pos >= servoStartY; pos -= 1) { 
    tstart = micros();
    servoX.write(servoStartX);
    servoY.write(pos); 
    timeStamp = timeStamp + servoIncrement;
    myFile.println(timeStamp + c + pos);                 
    delay(servoIncrement -(micros() - tstart)); 
                          
  }
   for (pos = servoStartY; pos >= servoStartY-20; pos -= 1) { 
    tstart = micros();
    servoX.write(servoStartX);
    servoY.write(pos);  
    timeStamp = timeStamp + servoIncrement;
    myFile.println(timeStamp + c + pos);             
    delay(servoIncrement -(micros() - tstart));                    
  }
  for (pos = servoStartY-20; pos <= servoStartY-1; pos += 1) { 
    tstart = micros();
    servoX.write(servoStartX);
    servoY.write(pos); 
    timeStamp = timeStamp + servoIncrement;
    myFile.println(timeStamp + c + pos);                 
    delay(servoIncrement -(micros() - tstart));                       
  }
  tstart2 = micros();
  myFile.close();
}

void sweepTest1() {
  servoX.write(servoStartX);
  delay(50);
  servoY.write(servoStartY);
  delay(500);
  servoX.write(servoStartX + 20);
  delay(500);
  servoX.write(servoStartX);
  delay(500);
  servoX.write(servoStartX - 20);
  delay(500);
  servoX.write(servoStartX);
  delay(500);
}
void sweepSquare() {
  servoX.write(servoStartX);
  delay(200);
  servoY.write(servoStartY);
  delay(200);
  servoX.write(servoStartX - 20);
  servoY.write(servoStartY + 20);
  delay(200);
  servoX.write(servoStartX + 20);
  delay(200);
  servoY.write(servoStartY + 20);
  delay(200);
  servoX.write(servoStartX + 20);
  delay(200);
  servoY.write(servoStartY - 20);
  delay(200);
  servoX.write(servoStartX - 20);
  delay(200);
  servoY.write(servoStartY - 20);
  delay(200);
  servoX.write(servoStartX - 20);
  delay(200);
  servoY.write(servoStartY + 20);
  delay(200);
  servoX.write(servoStartX);
  delay(200);
  servoY.write(servoStartY);
}
