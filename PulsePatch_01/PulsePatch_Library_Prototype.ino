
/***************************************************
 * 
 *  HIGH LEVEL FUNCTIONS
 * 
 ***************************************************/


// read interrupt flags and do the work to service them
void serviceInterrupts(){
    MAX_interrupt = false;  // reset this software flag
    interruptFlags = MAX_readInterrupts();  // read interrupt registers
//    Serial.println(interruptFlags,HEX);
    if((interruptFlags & (A_FULL<<8)) > 0){ // FIFO Almost Full
//      Serial.println("A_FULL");
    }
    if((interruptFlags & (PPG_RDY<<8)) > 0){ // PPG data ready
//      Serial.println("PPG_RDY");
//      readPointers();
      readPPG();  // read the light sensor data that is available
      if(OUTPUT_TYPE < 2){
        serialPPG(); // send the RED and/or IR data over a wire
      }
//      else {
//        buildBLEpacket();
//      }
    }
    if((interruptFlags & (ALC_OVF<<8)) > 0){ // Ambient Light Cancellation Overflow
//      Serial.println("ALC_OVF");
    }
    if((interruptFlags & TEMP_RDY) > 0){  // Temperature Conversion Available
//      Serial.println("TEMP_RDY");
      readTemp();
      if(OUTPUT_TYPE < 2){ printTemp(); }
    }
}


// send PPG value(s) via Serial port
void serialPPG(){
  if (OUTPUT_TYPE != OUTPUT_PLOTTER) {
    Serial.println();  // formatting...
    Serial.print(sampleCounter,DEC); printTab();
    Serial.print(REDvalue[packetSampleNumber]); printTab();
    Serial.print(IRvalue[packetSampleNumber]);
  } else {
    if(useFilter){
      Serial.print(HPfilterOutputRED[NUM_SAMPLES-1]); printSpace();
      Serial.print(HPfilterOutputIR[NUM_SAMPLES-1]);
    } else {
      Serial.print(REDvalue[packetSampleNumber]); printSpace();
      Serial.print(IRvalue[packetSampleNumber]);
    }
    Serial.println();
  }
}

void printSpace(){
   Serial.print(" ");
}


void readPPG(){
  sampleCounter++;
  if(sampleCounter > 200){ 
    sampleCounter = 1; 
    MAX30102_writeRegister(TEMP_CONFIG,0x01); // start a temperature conversion
  }
  readFIFOdata();
}

void readFIFOdata(){  // read in the FIFO data three bytes per ADC result
  char dataByte[6];
  int byteCounter = 0;
  packetSampleNumber++;
  if(packetSampleNumber == 4){
    packetSampleNumber = 0;
  }
  Wire.beginTransmission(MAX_ADD);
  Wire.write(FIFO_DATA);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX_ADD,6);
  while(Wire.available()){
    dataByte[byteCounter] = Wire.read();
    byteCounter++;
  }
  REDvalue[packetSampleNumber] = 0; IRvalue[packetSampleNumber] = 0;

  REDvalue[packetSampleNumber] = (dataByte[0] & 0xFF); REDvalue[packetSampleNumber] <<= 8;
  REDvalue[packetSampleNumber] |= dataByte[1]; REDvalue[packetSampleNumber] <<= 8;
  REDvalue[packetSampleNumber] |= dataByte[2];
  IRvalue[packetSampleNumber] = (dataByte[3] & 0xFF); IRvalue[packetSampleNumber] <<= 8;
  IRvalue[packetSampleNumber] |= dataByte[4]; IRvalue[packetSampleNumber] <<= 8;
  IRvalue[packetSampleNumber] |= dataByte[5];
  REDvalue[packetSampleNumber] &= 0x0003FFFF;
  IRvalue[packetSampleNumber] &= 0x0003FFFF;

  if(useFilter){
    filterHP(REDvalue[packetSampleNumber], IRvalue[packetSampleNumber]);
  }

    if(OUTPUT_TYPE == OUTPUT_BLE && packetSampleNumber == 3){
      packSamples();
      sendSamplesBLE();
    }
}

void packSamples(){

        radioBuffer[1] = ((REDvalue[0] &  0x0003FC00) >> 10);
        radioBuffer[2] = ((REDvalue[0] &  0x000003FC) >> 2);
        radioBuffer[3] = ((REDvalue[0] &  0x00000003) << 6);
        radioBuffer[3] |= ((IRvalue[0] & 0x0003F000) >> 12);
        radioBuffer[4] = ((IRvalue[0] &  0x00000FF0) >> 4);
        radioBuffer[5] = ((IRvalue[0] &  0x0000000F) << 4);
        radioBuffer[5] |= ((REDvalue[1] & 0x0003C000) >> 14);
        radioBuffer[6] = ((REDvalue[1] &  0x00003FC0) >> 6);
        radioBuffer[7] = ((REDvalue[1] &  0x0000003F) << 2);
        radioBuffer[7] |= ((IRvalue[1] & 0x00030000) >> 16);
        radioBuffer[8] = ((IRvalue[1] &  0x0000FF00) >> 8);
        radioBuffer[9] = (IRvalue[1] &   0x000000FF);
        radioBuffer[10] = ((REDvalue[2] &   0x0003FC00) >> 10);
        radioBuffer[11] = ((REDvalue[2] &  0x000003FC) >> 2);
        radioBuffer[12] = ((REDvalue[2] &  0x00000003) << 6);
        radioBuffer[12] |= ((IRvalue[2] & 0x0003F000) >> 12);
        radioBuffer[13] = ((IRvalue[2] &  0x00000FF0) >> 4);
        radioBuffer[14] = ((IRvalue[2] &  0x0000000F) << 4);
        radioBuffer[14] |= ((REDvalue[3] & 0x0003C000) >> 14);
        radioBuffer[15] = ((REDvalue[3] &  0x00003FC0) >> 6);
        radioBuffer[16] = ((REDvalue[3] &  0x0000003F) << 2);
        radioBuffer[16] |= ((IRvalue[3] & 0x00030000) >> 16);
        radioBuffer[17] = ((IRvalue[3] &  0x0000FF00) >> 8);
        radioBuffer[18] = (IRvalue[3] &   0x000000FF);
}

void sendSamplesBLE(){
      radioBuffer[0] = sampleCounter;
//      Serial.print(sampleCounter,DEC);  Serial.print('\t');
      radioBuffer[19] = 0;
      if(sampleCounter == 100){ radioBuffer[19] = tempInteger; }  // Serial.println(Celcius); }
      if(sampleCounter == 104){ radioBuffer[19] = tempFraction; }
//      Serial.println();
      if (BLEconnected) {
        SimbleeBLE.send(radioBuffer, 20);
      }
      
}

/***************************************************
 * 
 *  COMMUNICATION FUNCTIONS
 * 
 ***************************************************/

void eventSerial(){
  if(Serial.available()){
    char inChar = Serial.read();
    parseChar(inChar);
  }
}


void parseChar(char command){  
  uint16_t intSource;  
    switch(command){
      case 'h':
        printHelpToSerial();
        break;
      case 'b':
        Serial.println("start running");
        packetSampleNumber = -1;
        sampleCounter = 0;
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
        break;

      case 'f':
        useFilter = false;
        break;
      case 'F':
        useFilter = true;
        break;
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
        break;
      default:
        break;
    }
  }


void printHelpToSerial() {
  Serial.println(F("Commands:"));
  Serial.println(F("   'h'  Print this help information on available commands"));
  Serial.println(F("   'b'  Start the thing running at the sample rate selected"));
  Serial.println(F("   's'  Stop the thing running"));
  Serial.println(F("   't'  Initiate a temperature conversion. This should work if 'b' is pressed or not"));
  Serial.println(F("   'i'  Query the interrupt flags register. Not really useful"));
  Serial.println(F("   'v'  Verify the device by querying the RevID and PartID registers (hex 6 and hex 15 respectively)"));
  Serial.println(F("   '1'  Increase red LED intensity"));
  Serial.println(F("   '2'  Decrease red LED intensity"));
  Serial.println(F("   '3'  Increase IR LED intensity"));
  Serial.println(F("   '4'  Decrease IR LED intensity"));
  Serial.println(F("   '?'  Print all registers"));
  Serial.println(F("   'F'  Turn on filters"));
  Serial.println(F("   'f'  Turn off filters"));
}

/***************************************************
 * 
 *  SIMBLEE FUNCTIONS
 * 
 ***************************************************/

void SimbleeBLE_onConnect()
    {
      Serial.println("Connected");
      BLEconnected = true;
      digitalWrite(GRN_LED,HIGH);
      boardLEDstate = false;
      digitalWrite(RED_LED,boardLEDstate);
    }



    void SimbleeBLE_onDisconnect()
    {
      Serial.println("Connection Lost...");
      BLEconnected = false;
      enableMAX30102(false);
      digitalWrite(GRN_LED,LOW);
      LED_timer = millis();

    }

    void SimbleeBLE_onReceive(char *data, int len)
    {
      byte inChar = data[0];
      parseChar(inChar);
    }


/***************************************************
 * 
 *  MAX3010x FUNCTIONS
 * 
 ***************************************************/

void MAX_init(char sr){
  char setting;
  // reset the MAX30102
  setting = RESET;
  MAX30102_writeRegister(MODE_CONFIG,setting);
  delay(50);
  // set mode configuration put device in shutdown to set registers
  // 0x82 = shutdown, Heart Rate mode
  // 0x83 = shutdown, Sp02 mode
  setting = (SHUTDOWN | mode) & 0xFF;
  MAX30102_writeRegister(MODE_CONFIG,setting);
  // set fifo configuration
  setting = (SMP_AVE_1 | ROLLOVER_EN | 0x0F) & 0xFF;
  MAX30102_writeRegister(FIFO_CONFIG,setting);
  // set Sp02 configuration
  sampleRate = sr;
  setting = (ADC_RGE_4096 | sampleRate | PW_411) & 0xFF;
  MAX30102_writeRegister(SPO2_CONFIG,setting);
  // set LED pulse amplitude (current in mA)
  setLEDamplitude(rAmp,irAmp);
  // enable interrupts
  short interruptSettings = ( (PPG_RDY<<8) | (ALC_OVF<<8) | TEMP_RDY ) & 0xFFFF; // (A_FULL<<8) |
  MAX_setInterrupts(interruptSettings);
}

void enableMAX30102(boolean activate){
  char setting = mode;
  zeroFIFOpointers();
  if(!activate){ setting |= 0x80; }
  MAX30102_writeRegister(MODE_CONFIG,setting);
}

void zeroFIFOpointers(){
  MAX30102_writeRegister(FIFO_WRITE,0x00);
  MAX30102_writeRegister(OVF_COUNTER,0x00);
  MAX30102_writeRegister(FIFO_READ,0x00);
}

void readPointers(){
  readPointer = MAX30102_readRegister(FIFO_READ);
  ovfCounter = MAX30102_readRegister(OVF_COUNTER);
  writePointer = MAX30102_readRegister(FIFO_WRITE);
  Serial.print(readPointer,HEX); printTab();
  Serial.print(ovfCounter,HEX); printTab();
  Serial.println(writePointer,HEX);
}

// report RevID and PartID for verification
void getDeviceInfo(){
  char revID = MAX30102_readRegister(REV_ID);
  char partID = MAX30102_readRegister(PART_ID);
  Serial.print("Rev ID: 0x"); Serial.print(revID,HEX);
  Serial.print("\tPart ID: 0x"); Serial.println(partID,HEX);
}

//  read die temperature to compansate for RED LED
void readTemp(){
//  Serial.println("temp");
  tempInteger = MAX30102_readRegister(TEMP_INT);
  tempFraction = MAX30102_readRegister(TEMP_FRAC);
  Celcius = float(tempInteger);
  Celcius += (float(tempFraction)/16);
  Fahrenheit = Celcius*1.8 + 32;
}



void printTemp(){
  if (OUTPUT_TYPE != OUTPUT_PLOTTER) {
    printTab(); // formatting...
    Serial.print(Celcius,3); Serial.print("*C");
  }
}


// set the current amplitude for the LEDs
// currently uses the same setting for both RED and IR
// should be able to adjust each dynamically...
void setLEDamplitude(int Ir, int Iir){
  Ir *= 1000; Iir *= 1000;
  Ir /= 196; Iir /= 196;
  char currentIR = Iir & 0xFF;
  char currentR = Ir & 0xFF;
  MAX30102_writeRegister(RED_PA,currentR);
  if(mode == SPO2_MODE){
    MAX30102_writeRegister(IR_PA,currentIR);
  }else{
    MAX30102_writeRegister(IR_PA,0x00);
  }
}

// measures time between samples for verificaion purposes
void sampleTimeTest(){
  thisTestTime = micros();
  Serial.print("S\t"); Serial.println(thisTestTime - thatTestTime);
  thatTestTime = thisTestTime;
}

// set the desired interrupt flags
void MAX_setInterrupts(uint16_t setting){
  char highSetting = (setting >> 8) & 0xFF;
  char lowSetting = setting & 0xFF;
  Wire.beginTransmission(MAX_ADD);
  Wire.write(ENABLE_1);
  Wire.write(highSetting);
  Wire.write(lowSetting);
  Wire.endTransmission(true);
}



// reads the interrupt status registers
// returns a 16 bit value
uint16_t MAX_readInterrupts(){
  short inShort = MAX30102_readShort(STATUS_1);
  return inShort;
}

// writes one register to the MAX30102
void MAX30102_writeRegister(char reg, char setting){
  Wire.beginTransmission(MAX_ADD);
  Wire.write(reg);
  Wire.write(setting);
  Wire.endTransmission(true);
}

// reads one register from the MAX30102
char MAX30102_readRegister(char reg){
  char inChar;
  Wire.beginTransmission(MAX_ADD);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX_ADD,1);
  while(Wire.available()){
    inChar = Wire.read();
  }
 return inChar;
}

// reads two successive registers from the MAX30102
short MAX30102_readShort(char startReg){
  char inChar[2];
  short Shorty;
  int byteCounter = 0;
  Wire.beginTransmission(MAX_ADD);
  Wire.write(startReg);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX_ADD,2);
  while(Wire.available()){
    inChar[byteCounter] = Wire.read();
    byteCounter++;
  }
  Shorty = (inChar[0]<<8) | inChar[1];
 return Shorty;
}

// prints out register values
void printAllRegisters(){
  Wire.beginTransmission(MAX_ADD);
  Wire.write(STATUS_1);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX_ADD,7);
  readWireAndPrintHex(STATUS_1);
  Wire.endTransmission(true);
  Wire.beginTransmission(MAX_ADD);
  Wire.write(FIFO_CONFIG);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX_ADD,11);
  readWireAndPrintHex(FIFO_CONFIG);
  Wire.endTransmission(true);
  Wire.beginTransmission(MAX_ADD);
  Wire.write(TEMP_INT);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX_ADD,3);
  readWireAndPrintHex(TEMP_INT);
  Wire.endTransmission(true);

  Wire.beginTransmission(MAX_ADD);
  Wire.write(PROX_INT_THRESH);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX_ADD,1);
  readWireAndPrintHex(PROX_INT_THRESH);
  Wire.endTransmission(true);

  Wire.beginTransmission(MAX_ADD);
  Wire.write(REV_ID);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX_ADD,2);
  readWireAndPrintHex(REV_ID);
  Wire.endTransmission(true);
}

// helps to print out register values
void readWireAndPrintHex(char startReg){
  char inChar;
  while(Wire.available()){
    inChar = Wire.read();
    printRegName(startReg); startReg++;
    Serial.print("0x"); Serial.println(inChar,HEX);
  }
}



// helps with verbose feedback
void printRegName(char regToPrint){

  switch(regToPrint){
    case STATUS_1:
      Serial.print("STATUS_1\t"); break;
    case STATUS_2:
      Serial.print("STATUS_2\t"); break;
    case ENABLE_1:
      Serial.print("ENABLE_1\t"); break;
    case ENABLE_2:
      Serial.print("ENABLE_2\t"); break;
    case FIFO_WRITE:
      Serial.print("FIFO_WRITE\t"); break;
    case OVF_COUNTER:
      Serial.print("OVF_COUNTER\t"); break;
    case FIFO_READ:
      Serial.print("FIFO_READ\t"); break;
    case FIFO_DATA:
      Serial.print("FIFO_DATA\t"); break;
    case FIFO_CONFIG:
      Serial.print("FIFO_CONFIG\t"); break;
    case MODE_CONFIG:
      Serial.print("MODE_CONFIG\t"); break;
    case SPO2_CONFIG:
      Serial.print("SPO2_CONFIG\t"); break;
    case RED_PA:
      Serial.print("RED_PA\t"); break;
    case IR_PA:
      Serial.print("IR_PA\t"); break;
    case PILOT_PA:
      Serial.print("PILOT_PA\t"); break;
    case MODE_CNTRL_1:
      Serial.print("MODE_CNTRL_1\t"); break;
    case MODE_CNTRL_2:
      Serial.print("MODE_CNTRL_2\t"); break;
    case TEMP_INT:
      Serial.print("TEMP_INT\t"); break;
    case TEMP_FRAC:
      Serial.print("TEMP_FRAC\t"); break;
    case TEMP_CONFIG:
      Serial.print("TEMP_CONFIG\t"); break;
    case PROX_INT_THRESH:
      Serial.print("PROX_INT_THRESH\t"); break;
    case REV_ID:
      Serial.print("REV_ID\t"); break;
    case PART_ID:
      Serial.print("PART_ID\t"); break;
    default:
      Serial.print("RESERVED\t"); break;
  }
}

// formatting...
void printTab(){
  Serial.print("\t");
}
