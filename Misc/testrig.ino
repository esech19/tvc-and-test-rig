#include "HX711.h"
#include <SPI.h>
#include <SD.h>

// HX711 circuit wiring


File file; // file object that is used to read and write data
HX711 loadcell1; //scale object for making load cell read/write easier
HX711 loadcell2; //scale object for making load cell read/write easier

//SD PINS

const int CS = 4; //chip select pin for the MicroSD Card Adapter, This is the CS Pin

//ButtoN PIN

const int buttonPin = 2;  // the number of the pushbutton pin
int buttonState = 0;  // variable for reading the pushbutton status
bool buttonPressed = false;

//ENCODER PINS

int A1channelpin = 10; //A and B signals from 1st encoder, and index (original position)
int B1channelpin = 9;
int indexpin1 = 8;

int A2channelpin = 13; //A and B signals from 2nd encoder, and index (original position)
int B2channelpin = 12;
int indexpin2 = 11;

int a1State;
int a1LastState;

int b1State;
int b1LastState;

int a2State;
int a2LastState;

int b2State;
int b2LastState;

int counter1 = 0;
int counter2 = 0;

//SCALE PINS

const int LOADCELL1_DOUT_PIN = 2;
const int LOADCELL1_SCK_PIN = 3;

const int LOADCELL2_DOUT_PIN = 4;
const int LOADCELL2_SCK_PIN = 5;


void setup() {

    Serial.begin(57600); //setup for HX711 SCALE
    loadcell1.begin(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN); // begin loadcell 1
    loadcell2.begin(LOADCELL2_DOUT_PIN, LOADCELL2_SCK_PIN); // begin loadcell 2

    pinMode(CS, OUTPUT); // chip select pin is set as OUTPUT

    pinMode(A1channelpin, INPUT); //pinmode for encoders
    pinMode(B1channelpin, INPUT);
    pinMode(indexpin1, INPUT);

    pinMode(A2channelpin, INPUT); //pinmode for encoders
    pinMode(B2channelpin, INPUT);
    pinMode(indexpin2, INPUT);

    pinMode(buttonPin, INPUT);

    a1LastState = digitalRead(A1channelpin); 
    a2LastState = digitalRead(A2channelpin); 
 
    if (!SD.begin(CS)) { // Initialize SD card
    Serial.println("Could not initialize SD card."); // if return value is false, something went wrong.
    
    }

}

void writeFile(String first,int data, String last) //writing something to the SD card
          {
          file = SD.open("file.txt", FILE_WRITE); // 
          if (file) {  
            file.println(first);
            file.println(data, 2); // write number to file; in this case, the data with 2 decimals precision
            file.println(last);
            file.close(); // close file
           
          } else {
            Serial.println("Could not open file (writing).");
          }
  }

void loop() {
  
    buttonState = digitalRead(buttonPin); //read button to see if it was pressed, start recording data when pressed, when pressed again stop.
    if (buttonState == HIGH && buttonPressed == false){
        buttonPressed = true; 
    }
    else if (buttonState == HIGH && buttonPressed == true){
        buttonPressed = false; 
    }

    if (buttonPressed == true){ //record data
        if (loadcell1.is_ready() && loadcell2.is_ready()) { //reading values from first loadcell

            long reading1 = loadcell1.read() * 266;
            long reading2 = loadcell2.read() * 266;

            float grams1 = (float)reading1 / 100000.0; //the equation can be found here: https://www.teachmemicro.com/how-to-use-load-cell-hx711-arduino/ might need to change.
            grams1 = grams1 - 996.0;
            float kilograms1 = grams1/1000.0;  // to SI units
            writeFile("Loadcell Reads", kilograms1, " kg");

            float grams2 = (float)reading1 / 100000.0; //the equation can be found here: https://www.teachmemicro.com/how-to-use-load-cell-hx711-arduino/ might need to change.
            grams2 = grams2 - 996.0;
            float kilograms2 = grams2/1000.0;  // to SI units
            writeFile("Loadcell Reads", kilograms2, " kg");

            a1State = digitalRead(A1channelpin); // Reads the "current" state of the outputA
            // If the previous and the current state of the outputA are different, that means a Pulse has occured
            if (a1State != a1LastState){     
            // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
                if (digitalRead(B1channelpin) != a1State) { 
                    counter1 ++;
                } 
                else {
                    counter1 --;
                }
            }
            a1LastState = a1State; // Updates the previous state of the outputA with the current state

            a2State = digitalRead(A2channelpin); // Reads the "current" state of the outputA
            // If the previous and the current state of the outputA are different, that means a Pulse has occured
            if (a2State != a2LastState){     
            // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
                if (digitalRead(B2channelpin) != a2State) { 
                    counter2 ++;
                } 
                else {
                    counter2 --;
                }
            }
            a2LastState = a2State; // Updates the previous state of the outputA with the current state
            }   
        }
        writeFile("",counter1,"");
        writeFile("",counter2,"");
}
