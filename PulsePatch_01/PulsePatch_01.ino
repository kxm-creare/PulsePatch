/*
  Pulse Patch
  This code targets a Simblee
  I2C Interface with MAX30102 Sp02 Sensor Module
*/

#include <Wire.h>
#include "PulsePatch_Definitions.h"
//#include "PulsePatch.h"
#include <SimbleeBLE.h>

//Added by Chip 2016-09-28 to enable plotting by Arduino Serial Plotter
//Modified by Joel December, 2016
//Set to OUTPUT_NORMAL for normal verbose print() statements
//Set to OUTPUT_PLOTTER for Arduino Serial Plotter formatting
//Set to OUTPUT_BLE to enable BLE
const int OUTPUT_TYPE = OUTPUT_BLE;


unsigned int LED_timer;
int LED_delayTime = 300;
boolean boardLEDstate = HIGH;
int lastSwitchState;
volatile boolean MAX_interrupt = false;
volatile boolean ADS_interrupt = false;
short interruptSetting;
short interruptFlags;
char tempInteger;
char tempFraction;
float Celcius;
float Fahrenheit;
char MAX_sampleCounter = 0xFF;
int packetSampleNumber = 1;
int REDvalue[4];
int IRvalue[4];
char mode = MAX_SPO2_MODE;  // MAX_SPO2_MODE or MAX_HR_MODE
char readPointer;
char writePointer;
char ovfCounter;
int rAmp = 10;
int irAmp = 10;


//  TESTING
unsigned int thisTestTime;
unsigned int thatTestTime;

// FILTER STUFF
char sampleRate;
boolean useFilter = false;
int gain = 10;
float HPfilterInputRED[NUM_SAMPLES];
float HPfilterOutputRED[NUM_SAMPLES];
float LPfilterInputRED[NUM_SAMPLES];
float LPfilterOutputRED[NUM_SAMPLES];
float HPfilterInputIR[NUM_SAMPLES];
float HPfilterOutputIR[NUM_SAMPLES];
float LPfilterInputIR[NUM_SAMPLES];
float LPfilterOutputIR[NUM_SAMPLES];

// BLE STUFF
boolean BLEconnected = false;
char MAX_radioBuffer[20];
char ADS_radioBuffer[20];

void setup(){

  Wire.beginOnPins(SCL_PIN,SDA_PIN);
  boardLEDstate = true;
  pinMode(RED_LED,OUTPUT); digitalWrite(RED_LED, boardLEDstate);
  pinMode(GRN_LED,OUTPUT); digitalWrite(GRN_LED, !boardLEDstate);
  pinMode(TACT_SWITCH,INPUT);
  lastSwitchState = digitalRead(TACT_SWITCH);
  pinMode(MAX_INT,INPUT_PULLUP);
  attachPinInterrupt(MAX_INT,MAX_ISR,LOW);

  switch (OUTPUT_TYPE){
    case OUTPUT_NORMAL:
      Serial.println("\nPulsePatch 010\n");
      Serial.begin(230400);
      break;
    case OUTPUT_PLOTTER:
      Serial.begin(230400);
      break;
    case OUTPUT_BLE:
      Serial.begin(9600);
      SimbleeBLE.advertisementData = "PulsePatch 0.1.0";
//      Serial.println("BLE advertizing Pulse Patch");
      SimbleeBLE.begin();
      break;
    default: break;
  }

  LED_timer = millis();
  MAX_init(MAX_SR_200); // initialize MAX30102, specify sampleRate
  if (useFilter){ initFilter(); }
  if (OUTPUT_TYPE != OUTPUT_PLOTTER) {
    MAX_printAllRegisters();
    Serial.println();
    printHelpToSerial();
    Serial.println();
  } else {
    //when configured for the Arduino Serial Plotter, start the system running right away
    enableMAX30102(true);
    thatTestTime = micros();
  }
}


void loop(){

  if(MAX_interrupt){
    MAX_serviceInterrupts(); // go see what MAX event woke us up, and do the work
  }
  if(ADS_interrupt){
    ADS_serviceInterrupts(); // go see what ADS event woke us up, and do the work
  }

  blinkBoardLEDs();
  readSwitch();

  eventSerial(); // see if there's anything on the serial port; if so, process it
}



// RED_LED blinks when not connected to BLE
// GRN_LED steady on when BLE connected
void blinkBoardLEDs(){
  if((millis()-LED_timer > LED_delayTime) && !BLEconnected){
      LED_timer = millis();
      boardLEDstate = !boardLEDstate;
      digitalWrite(RED_LED,boardLEDstate);
    }
}

void readSwitch(){
  int switchState = digitalRead(TACT_SWITCH);
  if(switchState != lastSwitchState){
    delay(10);
    switchState = digitalRead(TACT_SWITCH);
    if(switchState != lastSwitchState){
      lastSwitchState = switchState;
      Serial.print("switch = "); Serial.println(switchState);
    }
  }
}


//Print out all of the commands so that the user can see what to do
//Added: Chip 2016-09-28


int MAX_ISR(uint32_t dummyPin) { // gotta have a dummyPin...
  MAX_interrupt = true;
  return 0; // gotta return something, somehow...
}

void serialAmps(){
  Serial.print("PA\t");
  Serial.print(rAmp); printTab(); Serial.println(irAmp);
}
