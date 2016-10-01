/*

  Pulse Patch
  This code targets a Simblee
  I2C Interface with MAX30102 Sp02 Sensor Module

*/

#include <Wire.h>
#include "MAX30102_Definitions.h"


unsigned int LED_timer;
int LED_delayTime = 300;
boolean boardLEDstate = HIGH;
volatile boolean MAX_interrupt = false;
short interruptSetting;
short interruptFlags;
float Celcius;
float Fahrenheit;
char sampleCounter = 0;
int REDvalue;
int IRvalue;
char mode = SPO2_MODE;  // SPO2_MODE or HR_MODE
char readPointer;
char writePointer;
char ovfCounter;
int rAmp = 10;
int irAmp = 10;
char sampleRate;

//  TESTING
unsigned int thisTestTime;
unsigned int thatTestTime;

void setup(){

  Wire.beginOnPins(SCL_PIN,SDA_PIN);
  Serial.begin(230400);
  pinMode(BOARD_LED,OUTPUT); digitalWrite(BOARD_LED, boardLEDstate);
  pinMode(MAX_INT,INPUT);

  attachPinInterrupt(MAX_INT,MAX_ISR,LOW);
  LED_timer = millis();
  Serial.println("\nPulsePatch 01\n");
  MAX_init(SR_100);
  printAllRegisters();
}


void loop(){

  if(MAX_interrupt){
    serviceInterrupts(); // go see what woke us up, and do the work
    if(sampleCounter == 0x00){  // rolls over to 0 at 200
      MAX30102_writeRegister(TEMP_CONFIG,0x01); // take temperature
    }
  }

  blinkBoardLED();


  eventSerial();
}

void eventSerial(){
  while(Serial.available()){
    char inByte = Serial.read();
    uint16_t intSource;
    switch(inByte){
      case 'b':
        Serial.println("start running");
        enableMAX30102(true);
        thatTestTime = micros();
        break;
      case 's':
        Serial.println("stop running");
        enableMAX30102(false);
        break;
      case 't':
        MAX30102_writeRegister(TEMP_CONFIG,0x01);
        break;
      case 'i':
        intSource = MAX30102_readShort(STATUS_1);
        Serial.print("intSource: 0x"); Serial.println(intSource,HEX);
        break;
      case 'v':
        getDeviceInfo();
        break;
      case '?':
        printAllRegisters();

      case '1':
        rAmp++; if(rAmp > 50){rAmp = 50;}
        setLEDamplitude(rAmp, irAmp);
        serialAmps();
        break;
      case '2':
        rAmp--; if(rAmp < 1){rAmp = 0;}
        setLEDamplitude(rAmp, irAmp);
        serialAmps();
        break;
      case '3':
        irAmp++; if(irAmp > 50){irAmp = 50;}
        setLEDamplitude(rAmp, irAmp);
        serialAmps();
        break;
      case '4':
        irAmp--; if(irAmp < 1){irAmp = 0;}
        setLEDamplitude(rAmp, irAmp);
        serialAmps();
      default:
        break;
    }
  }
}

void blinkBoardLED(){
  if(millis()-LED_timer > LED_delayTime){
      LED_timer = millis();
      boardLEDstate = !boardLEDstate;
      digitalWrite(BOARD_LED,boardLEDstate);
    }
}

int MAX_ISR(uint32_t dummyPin) { // gotta have a dummyPin...
  MAX_interrupt = true;
  return 0; // gotta return something, somehow...
}

void serialAmps(){
  Serial.print("PA\t");
  Serial.print(rAmp); printTab(); Serial.println(irAmp);
}

