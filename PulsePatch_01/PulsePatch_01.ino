/*
  Pulse Patch v02
  This code targets a Simblee
  I2C Interface with MAX30102 Sp02 Sensor Module
*/

#include <Wire.h>
#include <SPI.h>
#include "PulsePatch_Definitions.h"
//#include "PulsePatch.h"
#include <SimbleeBLE.h>
#include "algorithm.h"

//Added by Chip 2016-09-28 to enable plotting by Arduino Serial Plotter
//Modified by Joel December, 2016
//Set to OUTPUT_NORMAL for normal verbose print() statements
//Set to OUTPUT_PLOTTER for Arduino Serial Plotter formatting
//Set to OUTPUT_BLE to enable BLE
const int OUTPUT_TYPE = OUTPUT_NORMAL;

// LED/Board Functions
unsigned int LED_timer;
int LED_delayTime = 300;
boolean boardLEDstate = HIGH;
int lastSwitchState;

// MAX VARIABLES:
volatile boolean MAX_interrupt = false;
short interruptSetting;
short interruptFlags;
char tempInteger;
char tempFraction;
float Celcius;
float Fahrenheit;
int MAX_sampleRate;
long MAX_packetNumber; // Counter telling us how many MAX waveform packets we've sent.  Also allows us to calculate sample number.
int MAX_packetSampleNumber = 0; // counter of samples within a packet
long REDvalue[4];
long IRvalue[4];
char mode = MAX_SPO2_MODE;  // MAX_SPO2_MODE or MAX_HR_MODE
char readPointer;
char writePointer;
char ovfCounter;
int rAmp = 10;
int irAmp = 10;
uint32_t RED_buffer[IR_BUFFER_LENGTH];
uint32_t IR_buffer[IR_BUFFER_LENGTH];
int IR_buffer_counter = 0; // how many points in the IR buffer are filled.  If IR_buffer_counter=IR_BUFFER_LENGTH, data must be shifted.
int32_t IR_valley_locs[15]; // the locations within the buffer where a valley is within the IR buffer data
int32_t num_IR_valleys; // How many valleys have been located within the IR buffer data
uint32_t IRvalleyQueue_packetNumber[15]; // the locations of valleys that are queued up and ready to send
uint32_t IRvalleyQueue_packetSampleNumber[15]; // the locations of valleys that are queued up and ready to send
uint32_t IRvalleyQueue_instHR[15]; // the locations of valleys that are queued up and ready to send
int IRvalleyQueue_length = 0; // How many valleys are in the queue?
int32_t MAX_avg_SpO2; // avg SpO2 across the duration of the buffer
int8_t MAX_SpO2_valid; // Does the algorithm think the calculated SpO2 is valid?
int32_t MAX_avg_HR; // avg HR across the duration of the buffer
int8_t MAX_HR_valid; // Does the algorithm think the calculated HR is valid?

// ADS VARIABLES:
byte ADS_channelDataRaw[6];
int ADS_channelDataInt[2];
int ECGvalue[6];
volatile boolean ADS_interrupt = false;
int ADS_internalCounter = 0;
long ADS_packetNumber; // Counting every waveform packet we send.  Starts with 0.
int ADS_packetSampleNumber = 0; // Sample # within a waveform packet
unsigned long runTimer;
byte ADS_sampleCounter = 0;
boolean isRunning = false;
long ADSstatus;   // first 3 bytes of each sample is the ADSstatus
byte regData[12];           // array is used to mirror register data
boolean verbosity = false;

//  TESTING
unsigned int thisTestTime;
unsigned int thatTestTime;
unsigned int timeStartAll;
unsigned int timeEndAll;
unsigned int dtAll;
unsigned int timeStart1;
unsigned int timeEnd1;
unsigned int dt1;


// FAKING THE ADS INTERRUPT LOOP
// unsigned int ADS_timer = 0;
// int ADS_delayTime = 2; // 500 SPS

// MAX FILTER STUFF
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
  SPI.begin();
  SPI.setDataMode(SPI_MODE1);
  SPI.setFrequency(4000);
  SPI.setDataMode(MSBFIRST);
  pinMode(ADS_SS,OUTPUT); digitalWrite(ADS_SS,HIGH);
  pinMode(ADS_DRDY,INPUT);
  attachPinInterrupt(ADS_DRDY,ADS_ISR,LOW);
  pinMode(MAX_INT,INPUT_PULLUP);
  attachPinInterrupt(MAX_INT,MAX_ISR,LOW);


  boardLEDstate = true;
  pinMode(RED_LED,OUTPUT); digitalWrite(RED_LED,boardLEDstate);
  pinMode(GRN_LED,OUTPUT); digitalWrite(GRN_LED,!boardLEDstate);
  pinMode(TACT_SWITCH,INPUT);
  lastSwitchState = digitalRead(TACT_SWITCH);


  switch (OUTPUT_TYPE){
    case OUTPUT_NORMAL:
      Serial.begin(230400);
      delay(10);
      Serial.println("\nPulsePatch v0.2.0\n");
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
  Serial.println("init MAX");
  MAX_init(MAX_SR_200); // initialize MAX30102, specify sampleRate
  Serial.println("init ADS");
  ADS_init(); // initialize the ADS ECG
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
    updateChannelData(); // go see what ADS event woke us up, and do the work
    if(millis() - runTimer > 10000){
      stopADS();
      Serial.println("test complete");
    }
  }

//  blinkBoardLEDs();
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
//      digitalWrite(GRN_LED,!boardLEDstate);
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

int ADS_ISR(uint32_t dummyPin) { // gotta have a dummyPin...
  ADS_interrupt = true;
  return 0; // gotta return something, somehow...
}

int MAX_ISR(uint32_t dummyPin) { // gotta have a dummyPin...
  MAX_interrupt = true;
  return 0; // gotta return something, somehow...
}

void serialAmps(){
  Serial.print("PA\t");
  Serial.print(rAmp); printTab(); Serial.println(irAmp);
}
