
/***************************************************
 * 
 *  HIGH LEVEL MAX FUNCTIONS
 * 
 ***************************************************/

// read interrupt flags and do the work to service them
void MAX_serviceInterrupts(){
    MAX_interrupt = false;  // reset this software flag
    interruptFlags = MAX_readInterrupts();  // read interrupt registers
//    Serial.println(interruptFlags,HEX);
    if((interruptFlags & (MAX_A_FULL<<8)) > 0){ // FIFO Almost Full
//      Serial.println("MAX_A_FULL");
    }
    if((interruptFlags & (MAX_PPG_RDY<<8)) > 0){ // PPG data ready
//      Serial.println("MAX_PPG_RDY");
//      readPointers();
      readPPG();  // read the light sensor data that is available
      if(OUTPUT_TYPE < 2){
        serialPPG(); // send the RED and/or IR data over a wire
      }
//      else {
//        buildBLEpacket();
//      }
    }
    if((interruptFlags & (MAX_ALC_OVF<<8)) > 0){ // Ambient Light Cancellation Overflow
//      Serial.println("MAX_ALC_OVF");
    }
    if((interruptFlags & MAX_TEMP_RDY) > 0){  // Temperature Conversion Available
//      Serial.println("MAX_TEMP_RDY");
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

//
void readPPG(){
  sampleCounter++;
  if(sampleCounter > 200){ 
    sampleCounter = 1; 
    MAX30102_writeRegister(MAX_TEMP_CONFIG,0x01); // start a temperature conversion
  }
  MAX_readFIFOdata();
}

// read in the FIFO data three bytes per ADC result
void MAX_readFIFOdata(){
  char dataByte[6];
  int byteCounter = 0;
  packetSampleNumber++;
  if(packetSampleNumber == 4){
    packetSampleNumber = 0;
  }
  Wire.beginTransmission(MAX_ADD);
  Wire.write(MAX_FIFO_DATA);
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
      MAX_packsamples();
      MAX_sendSamplesBLE();
    }
}

// Pack samples into a buffer for transmission via BLE
void MAX_packsamples(){

  MAX_radioBuffer[1] = ((REDvalue[0] &  0x0003FC00) >> 10);
  MAX_radioBuffer[2] = ((REDvalue[0] &  0x000003FC) >> 2);
  MAX_radioBuffer[3] = ((REDvalue[0] &  0x00000003) << 6);
  MAX_radioBuffer[3] |= ((IRvalue[0] & 0x0003F000) >> 12);
  MAX_radioBuffer[4] = ((IRvalue[0] &  0x00000FF0) >> 4);
  MAX_radioBuffer[5] = ((IRvalue[0] &  0x0000000F) << 4);
  MAX_radioBuffer[5] |= ((REDvalue[1] & 0x0003C000) >> 14);
  MAX_radioBuffer[6] = ((REDvalue[1] &  0x00003FC0) >> 6);
  MAX_radioBuffer[7] = ((REDvalue[1] &  0x0000003F) << 2);
  MAX_radioBuffer[7] |= ((IRvalue[1] & 0x00030000) >> 16);
  MAX_radioBuffer[8] = ((IRvalue[1] &  0x0000FF00) >> 8);
  MAX_radioBuffer[9] = (IRvalue[1] &   0x000000FF);
  MAX_radioBuffer[10] = ((REDvalue[2] &   0x0003FC00) >> 10);
  MAX_radioBuffer[11] = ((REDvalue[2] &  0x000003FC) >> 2);
  MAX_radioBuffer[12] = ((REDvalue[2] &  0x00000003) << 6);
  MAX_radioBuffer[12] |= ((IRvalue[2] & 0x0003F000) >> 12);
  MAX_radioBuffer[13] = ((IRvalue[2] &  0x00000FF0) >> 4);
  MAX_radioBuffer[14] = ((IRvalue[2] &  0x0000000F) << 4);
  MAX_radioBuffer[14] |= ((REDvalue[3] & 0x0003C000) >> 14);
  MAX_radioBuffer[15] = ((REDvalue[3] &  0x00003FC0) >> 6);
  MAX_radioBuffer[16] = ((REDvalue[3] &  0x0000003F) << 2);
  MAX_radioBuffer[16] |= ((IRvalue[3] & 0x00030000) >> 16);
  MAX_radioBuffer[17] = ((IRvalue[3] &  0x0000FF00) >> 8);
  MAX_radioBuffer[18] = (IRvalue[3] &   0x000000FF);
}

void MAX_sendSamplesBLE(){
  MAX_radioBuffer[0] = sampleCounter;
//      Serial.print(sampleCounter,DEC);  Serial.print('\t');
  MAX_radioBuffer[19] = 0;
  if(sampleCounter == 100){ MAX_radioBuffer[19] = tempInteger; }  // Serial.println(Celcius); }
  if(sampleCounter == 104){ MAX_radioBuffer[19] = tempFraction; }
//      Serial.println();
  if (BLEconnected) {
    SimbleeBLE.send(MAX_radioBuffer, 20);
  }
}

/***************************************************
 * 
 *  HIGH LEVEL ADS FUNCTIONS
 * 
 ***************************************************/

// read interrupt flags and do the work to service them
void ADS_serviceInterrupts(){
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
      MAX30102_writeRegister(MAX_TEMP_CONFIG,0x01);
      break;
    case 'i':
      intSource = MAX30102_readShort(MAX_STATUS_1);
      Serial.print("intSource: 0x"); Serial.println(intSource,HEX);
      break;
    case 'v':
      MAX_getDeviceInfo();
      break;
    case '?':
      MAX_printAllRegisters();
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

// Print tab to serial port
void printTab(){
  Serial.print("\t");
}

// Print space to serial port
void printSpace(){
   Serial.print(" ");
}

/***************************************************
 * 
 *  SIMBLEE FUNCTIONS
 * 
 ***************************************************/

void SimbleeBLE_onConnect() {
  Serial.println("Connected");
  BLEconnected = true;
  digitalWrite(GRN_LED,HIGH);
  boardLEDstate = false;
  digitalWrite(RED_LED,boardLEDstate);
}

void SimbleeBLE_onDisconnect() {
  Serial.println("Connection Lost...");
  BLEconnected = false;
  enableMAX30102(false);
  digitalWrite(GRN_LED,LOW);
  LED_timer = millis();
}

void SimbleeBLE_onReceive(char *data, int len) {
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
  setting = MAX_RESET;
  MAX30102_writeRegister(MAX_MODE_CONFIG,setting);
  delay(50);
  // set mode configuration put device in shutdown to set registers
  // 0x82 = shutdown, Heart Rate mode
  // 0x83 = shutdown, Sp02 mode
  setting = (MAX_SHUTDOWN | mode) & 0xFF;
  MAX30102_writeRegister(MAX_MODE_CONFIG,setting);
  // set fifo configuration
  setting = (MAX_SMP_AVE_1 | MAX_ROLLOVER_EN | 0x0F) & 0xFF;
  MAX30102_writeRegister(MAX_FIFO_CONFIG,setting);
  // set Sp02 configuration
  sampleRate = sr;
  setting = (MAX_ADC_RGE_4096 | sampleRate | MAX_PW_411) & 0xFF;
  MAX30102_writeRegister(MAX_SPO2_CONFIG,setting);
  // set LED pulse amplitude (current in mA)
  setLEDamplitude(rAmp,irAmp);
  // enable interrupts
  short interruptSettings = ( (MAX_PPG_RDY<<8) | (MAX_ALC_OVF<<8) | MAX_TEMP_RDY ) & 0xFFFF; // (MAX_A_FULL<<8) |
  MAX_setInterrupts(interruptSettings);
}

void enableMAX30102(boolean activate){
  char setting = mode;
  MAX_zeroFIFOpointers();
  if(!activate){ setting |= 0x80; }
  MAX30102_writeRegister(MAX_MODE_CONFIG,setting);
}

void MAX_zeroFIFOpointers(){
  MAX30102_writeRegister(MAX_FIFO_WRITE,0x00);
  MAX30102_writeRegister(MAX_OVF_COUNTER,0x00);
  MAX30102_writeRegister(MAX_FIFO_READ,0x00);
}

void readPointers(){
  readPointer = MAX30102_readRegister(MAX_FIFO_READ);
  ovfCounter = MAX30102_readRegister(MAX_OVF_COUNTER);
  writePointer = MAX30102_readRegister(MAX_FIFO_WRITE);
  Serial.print(readPointer,HEX); printTab();
  Serial.print(ovfCounter,HEX); printTab();
  Serial.println(writePointer,HEX);
}

// report RevID and PartID for verification
void MAX_getDeviceInfo(){
  char revID = MAX30102_readRegister(MAX_REV_ID);
  char partID = MAX30102_readRegister(MAX_PART_ID);
  Serial.print("Rev ID: 0x"); Serial.print(revID,HEX);
  Serial.print("\tPart ID: 0x"); Serial.println(partID,HEX);
}

//  read die temperature to compansate for RED LED
void readTemp(){
//  Serial.println("temp");
  tempInteger = MAX30102_readRegister(MAX_TEMP_INT);
  tempFraction = MAX30102_readRegister(MAX_TEMP_FRAC);
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
  MAX30102_writeRegister(MAX_RED_PA,currentR);
  if(mode == MAX_SPO2_MODE){
    MAX30102_writeRegister(MAX_IR_PA,currentIR);
  }else{
    MAX30102_writeRegister(MAX_IR_PA,0x00);
  }
}

// measures time between samples for verificaion purposes
void MAX_sampleTimeTest(){
  thisTestTime = micros();
  Serial.print("S\t"); Serial.println(thisTestTime - thatTestTime);
  thatTestTime = thisTestTime;
}

// set the desired interrupt flags
void MAX_setInterrupts(uint16_t setting){
  char highSetting = (setting >> 8) & 0xFF;
  char lowSetting = setting & 0xFF;
  Wire.beginTransmission(MAX_ADD);
  Wire.write(MAX_ENABLE_1);
  Wire.write(highSetting);
  Wire.write(lowSetting);
  Wire.endTransmission(true);
}

// reads the interrupt status registers
// returns a 16 bit value
uint16_t MAX_readInterrupts(){
  short inShort = MAX30102_readShort(MAX_STATUS_1);
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
void MAX_printAllRegisters(){
  Wire.beginTransmission(MAX_ADD);
  Wire.write(MAX_STATUS_1);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX_ADD,7);
  readWireAndPrintHex(MAX_STATUS_1);
  Wire.endTransmission(true);
  Wire.beginTransmission(MAX_ADD);
  Wire.write(MAX_FIFO_CONFIG);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX_ADD,11);
  readWireAndPrintHex(MAX_FIFO_CONFIG);
  Wire.endTransmission(true);
  Wire.beginTransmission(MAX_ADD);
  Wire.write(MAX_TEMP_INT);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX_ADD,3);
  readWireAndPrintHex(MAX_TEMP_INT);
  Wire.endTransmission(true);

  Wire.beginTransmission(MAX_ADD);
  Wire.write(MAX_PROX_INT_THRESH);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX_ADD,1);
  readWireAndPrintHex(MAX_PROX_INT_THRESH);
  Wire.endTransmission(true);

  Wire.beginTransmission(MAX_ADD);
  Wire.write(MAX_REV_ID);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX_ADD,2);
  readWireAndPrintHex(MAX_REV_ID);
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
    case MAX_STATUS_1:
      Serial.print("MAX_STATUS_1\t"); break;
    case MAX_STATUS_2:
      Serial.print("MAX_STATUS_2\t"); break;
    case MAX_ENABLE_1:
      Serial.print("MAX_ENABLE_1\t"); break;
    case MAX_ENABLE_2:
      Serial.print("MAX_ENABLE_2\t"); break;
    case MAX_FIFO_WRITE:
      Serial.print("MAX_FIFO_WRITE\t"); break;
    case MAX_OVF_COUNTER:
      Serial.print("MAX_OVF_COUNTER\t"); break;
    case MAX_FIFO_READ:
      Serial.print("MAX_FIFO_READ\t"); break;
    case MAX_FIFO_DATA:
      Serial.print("MAX_FIFO_DATA\t"); break;
    case MAX_FIFO_CONFIG:
      Serial.print("MAX_FIFO_CONFIG\t"); break;
    case MAX_MODE_CONFIG:
      Serial.print("MAX_MODE_CONFIG\t"); break;
    case MAX_SPO2_CONFIG:
      Serial.print("MAX_SPO2_CONFIG\t"); break;
    case MAX_RED_PA:
      Serial.print("MAX_RED_PA\t"); break;
    case MAX_IR_PA:
      Serial.print("MAX_IR_PA\t"); break;
    case MAX_PILOT_PA:
      Serial.print("MAX_PILOT_PA\t"); break;
    case MAX_MODE_CNTRL_1:
      Serial.print("MAX_MODE_CNTRL_1\t"); break;
    case MAX_MODE_CNTRL_2:
      Serial.print("MAX_MODE_CNTRL_2\t"); break;
    case MAX_TEMP_INT:
      Serial.print("MAX_TEMP_INT\t"); break;
    case MAX_TEMP_FRAC:
      Serial.print("MAX_TEMP_FRAC\t"); break;
    case MAX_TEMP_CONFIG:
      Serial.print("MAX_TEMP_CONFIG\t"); break;
    case MAX_PROX_INT_THRESH:
      Serial.print("MAX_PROX_INT_THRESH\t"); break;
    case MAX_REV_ID:
      Serial.print("MAX_REV_ID\t"); break;
    case MAX_PART_ID:
      Serial.print("MAX_PART_ID\t"); break;
    default:
      Serial.print("RESERVED\t"); break;
  }
}

/***************************************************
 * 
 *  ADS1292 FUNCTIONS
 * 
 ***************************************************/

