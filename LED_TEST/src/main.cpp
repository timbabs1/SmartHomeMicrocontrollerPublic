#include <Arduino.h>
#include <chrono>
//Initializing I/O pins
const int pirPin = 22;
const int ledPin = 23;
const int ledHeat = 2;
const int ledCool = 4;
const int ldrPin = 36;
//Initializing ldr and pir values
int ldrValue = 0;
int pirState = LOW;
//Initializing time variables
unsigned long now = millis();
unsigned long lastTrigger = 0;
//Initializing flag to indicate motion detection
bool stopFlag = false;
//LED Heat set to TRUE
bool ledHeatFlag = false;
bool ledOffFlag  = false;
//Interrupt routine, executed when motion is detected by pir
void IRAM_ATTR MovementDetected() {
    //Read ldr value, record trigger time and set flag to "true"
    int ldrValue = analogRead(ldrPin);
    //Change flag state to true, indicating motion detected
    stopFlag = true;
    //Record current time as "last trigger"
    lastTrigger = millis();
    //Output "Motion Detected!"
    Serial.println("Movement Detected!");
    //If ldr reading is below 500 then turn on led
    if (ldrValue <=500){
            digitalWrite(ledPin, HIGH);
    }
}

void temperature(){
    //If target temperature > current temperature, turn cooling led on
    if(target_Temperature > current_Temp && !ledCoolFlag && ledHeatFlag){
        digitalWrite(ledHeat, LOW);
        digitalWrite(ledCool, HIGH);
        ledCoolFlag = true;
        ledHeatFlag = false;

    } //If target temperature < current temperature, turn heating led on
    else if(target_temperature < current_Temp && ledCoolFlag && !ledHeatFlag) {
        digitalWrite(ledCool, LOW);
        digitalWrite(ledHeat, HIGH);
        ledCoolFlag = false;
        ledHeatFlag = true;
        
    }
    else{
        digitalWrite(ledCool, LOW);
        digitalWrite(ledHeat, HIGH);
        ledHeatFlag = false;
        ledCoolFlag = false;
    }
}

//Setting baud rate and I/O pins
void setup() {
Serial.begin(9600);
//Pirpin set as input with interrupt attached
pinMode(pirPin, INPUT);
attachInterrupt(digitalPinToInterrupt(pirPin), MovementDetected, RISING);
//ldrPin set as input
pinMode(ldrPin, INPUT);
//ledpin set as output
pinMode(ledPin, OUTPUT);
pinMode(ledHeat, OUTPUT);
pinMode(ledCool, OUTPUT);
}
//Main routine turns led off 1000ms after the last trigger
void loop() {
    //Stores current time
    now = millis();
   //If current time-trigger is greater than 1000ms
    if((now-lastTrigger) > 10000 && stopFlag) {
        //Output "Motion stopped"
        Serial.println("Movement Stopped...");
        //Turn off led
        digitalWrite(ledPin, LOW);
        //Indicate motion is no longer detected
        stopFlag = false;
    }
}

