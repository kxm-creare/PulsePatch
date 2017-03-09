
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
    Serial.print(MAX_sampleCounter,DEC); printTab();
    Serial.print(REDvalue[MAX_packetSampleNumber]); printTab();
    Serial.print(IRvalue[MAX_packetSampleNumber]);
  } else {
    if(useFilter){
      Serial.print(HPfilterOutputRED[NUM_SAMPLES-1]); printSpace();
      Serial.print(HPfilterOutputIR[NUM_SAMPLES-1]);
    } else {
      Serial.print(REDvalue[MAX_packetSampleNumber]); printSpace();
      Serial.print(IRvalue[MAX_packetSampleNumber]);
    }
    Serial.println();
  }
}

//
void readPPG(){
  MAX_sampleCounter++;
  if(MAX_sampleCounter > 200){
    MAX_sampleCounter = 1;
    MAX30102_writeRegister(MAX_TEMP_CONFIG,0x01); // start a temperature conversion
  }
  MAX_readFIFOdata();
}

// read in the FIFO data three bytes per ADC result
void MAX_readFIFOdata(){
  char dataByte[6];
  int byteCounter = 0;
  MAX_packetSampleNumber++;
  if(MAX_packetSampleNumber == 4){
    MAX_packetSampleNumber = 0;
  }
  Wire.beginTransmission(MAX_ADD);
  Wire.write(MAX_FIFO_DATA);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX_ADD,6);
  while(Wire.available()){
    dataByte[byteCounter] = Wire.read();
    byteCounter++;
  }
  REDvalue[MAX_packetSampleNumber] = 0; IRvalue[MAX_packetSampleNumber] = 0;

  REDvalue[MAX_packetSampleNumber] = (dataByte[0] & 0xFF); REDvalue[MAX_packetSampleNumber] <<= 8;
  REDvalue[MAX_packetSampleNumber] |= dataByte[1]; REDvalue[MAX_packetSampleNumber] <<= 8;
  REDvalue[MAX_packetSampleNumber] |= dataByte[2];
  IRvalue[MAX_packetSampleNumber] = (dataByte[3] & 0xFF); IRvalue[MAX_packetSampleNumber] <<= 8;
  IRvalue[MAX_packetSampleNumber] |= dataByte[4]; IRvalue[MAX_packetSampleNumber] <<= 8;
  IRvalue[MAX_packetSampleNumber] |= dataByte[5];
  REDvalue[MAX_packetSampleNumber] &= 0x0003FFFF;
  IRvalue[MAX_packetSampleNumber] &= 0x0003FFFF;

  if(useFilter){
    filterHP(REDvalue[MAX_packetSampleNumber], IRvalue[MAX_packetSampleNumber]);
  }

    if(OUTPUT_TYPE == OUTPUT_BLE && MAX_packetSampleNumber == 3){
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
  char MAX_packetNumber = MAX_sampleCounter>>2; // equivalent to dividing by 4.  If we have 6 samples per packet we'd need to divide by 6.
  MAX_radioBuffer[0] = (PKT_TYPE_MAX<<6);
  MAX_radioBuffer[0] |= MAX_packetNumber;
  //Serial.print(MAX_packetNumber,DEC);  Serial.print('\t'); Serial.print(MAX_radioBuffer[0],HEX); Serial.print('\n');
  MAX_radioBuffer[19] = 0;
  if(MAX_packetNumber == 25){ MAX_radioBuffer[19] = tempInteger; }  // Serial.println(Celcius); }
  if(MAX_packetNumber == 26){ MAX_radioBuffer[19] = tempFraction; }
  if (BLEconnected) {
    SimbleeBLE.send(MAX_radioBuffer, 20);
  }
}

/***************************************************
 *
 *  HIGH LEVEL ADS FUNCTIONS
 *
 ***************************************************/

//  Initialize the ADS at 500SPS, Gain of 12, ready to rock and roll
void ADS_init(){
    _ADS_RESET();
    delay(10);
    WREG(ADS_CONFIG1,0b00000000); // 0x02 = sample rate of 500SPs
    WREG(ADS_CONFIG2,0b10100000); // 0xA0, use the internal reference, do test signal as 0b10100011
    WREG(ADS_CH1SET,0b01100000);  // 0x60 enable input normal on channel 1, gain 12, test signal ends 0101
    WREG(ADS_CH2SET,0b01100001);  // 0x60 enable input normal on channel 2, gain 12, shorted is last bit = 1

    printADSregisters();    // report register values for confirmation

}


//  Start the ADS running
 void startADS(void)
 {
     ADS_sampleCounter = 0;
     _ADS_RDATAC(); // enter Read Data Continuous mode
     delay(1);
     _ADS_START();  // start the data acquisition
//     delay(1);
     isRunning = true;
 }



 // CALLED WHEN DRDY PIN IS ASSERTED. NEW ADS DATA AVAILABLE!
 void updateChannelData(){
     ADS_interrupt = false;
     byte inByte;
     int byteCounter = 0; // count the raw ADS bytes. Should be six of them

     digitalWrite(ADS_SS,LOW);       //  open SPI
     for(int i=0; i<3; i++){    //  read status register (1100 + LOFF_STATP + LOFF_STATN + GPIO[7:4])
         inByte = SPI.transfer(0x00);
         ADSstatus = (ADSstatus << 8) | inByte;
     }
     for(int i = 0; i<2; i++){  //  read 2 channels of 24 bits of channel data in 3 byte chunks
       ADS_channelDataInt[i] = 0;
         for(int j=0; j<3; j++){   
             inByte = SPI.transfer(0x00);
             ADS_channelDataRaw[byteCounter] = inByte;  // raw data goes here
             byteCounter++;
             ADS_channelDataInt[i] = (ADS_channelDataInt[i]<<8) | inByte;  // int data goes here
         }
     }
     digitalWrite(ADS_SS,HIGH); // close SPI

     // need to convert 24bit to 32bit if using the filter
     for(int i=0; i<2; i++){ // convert 3 byte 2's compliment to 4 byte 2's compliment
         if(bitRead(ADS_channelDataInt[i],23) == 1){
             ADS_channelDataInt[i] |= 0xFF000000;
         } else{
             ADS_channelDataInt[i] &= 0x00FFFFFF;
         }
     }

     //print data to serial port
     for(int i=0; i<1; i++){  //print 1 channel or both?
       Serial.print(ADS_channelDataInt[i]); Serial.print("\t");
     }
     Serial.println();
 }

 // Stop the continuous data acquisition
 void stopADS()
 {
     _ADS_STOP();     // stop the data acquisition
     delay(1);
     _ADS_SDATAC();   // stop Read Data Continuous mode to communicate with ADS
     delay(1);
     isRunning = false;
 }


 // write as binary each channel's data to the serial port
 void ADS_writeChannelDataSerial()
 {
   for(int i=0; i<6; i++){
     Serial.write(ADS_channelDataRaw[i]);
   }
 }


// read in the FIFO data three bytes per ADC result
void ADS_serviceInterrupts(){
  // char dataByte[] = {0x00, 0x00, 0x00};
  // int byteCounter = 0;
  // ADS_sampleCounter++;
  // if(ADS_packetSampleNumber == 6){
  //   ADS_packetSampleNumber = 0;
  // }
  //
  // // Look at my awesomely fake data for channel 1!
  // switch (ADS_packetSampleNumber) {
  //   case 0:
  //     // zero is fine
  //     break;
  //   case 1:
  //     dataByte[1] = 0xFF; // 0x00FF00 = 65,280
  //     break;
  //   case 2:
  //     dataByte[0] = 0x07; // 0x070000 = 458,752
  //     break;
  //   case 3:
  //     dataByte[0] = 0x70; // 0x700000 = 7,340,032
  //     break;
  //   case 4:
  //     dataByte[0] = 0xF7; // 0xF70000 = 16,187,392
  //     break;
  //   case 5:
  //     dataByte[0] = 0xF0; // 0xF00000 = 15,728,640
  //     break;
  // }
  //
  // ECGvalue[ADS_packetSampleNumber] = 0;
  // ECGvalue[ADS_packetSampleNumber] = (dataByte[0] & 0xFF); ECGvalue[ADS_packetSampleNumber] <<= 8;
  // ECGvalue[ADS_packetSampleNumber] |= dataByte[1]; ECGvalue[ADS_packetSampleNumber] <<= 8;
  // ECGvalue[ADS_packetSampleNumber] |= dataByte[2];
  //
  // ECGvalue[ADS_packetSampleNumber] &= 0x00FFFFFF;
  //
  // if(OUTPUT_TYPE == OUTPUT_BLE && ADS_packetSampleNumber == 5){
//    ADS_packsamples();
//    ADS_sendSamplesBLE();

}

// Pack samples into a buffer for transmission via BLE
void ADS_packsamples(){

  ADS_radioBuffer[1]  = ((ECGvalue[0] &  0x00FF0000) >> 16);
  ADS_radioBuffer[2]  = ((ECGvalue[0] &  0x0000FF00) >> 8);
  ADS_radioBuffer[3]  =  (ECGvalue[0] &  0x000000FF);
  ADS_radioBuffer[4]  = ((ECGvalue[1] &  0x00FF0000) >> 16);
  ADS_radioBuffer[5]  = ((ECGvalue[1] &  0x0000FF00) >> 8);
  ADS_radioBuffer[6]  =  (ECGvalue[1] &  0x000000FF);
  ADS_radioBuffer[7]  = ((ECGvalue[2] &  0x00FF0000) >> 16);
  ADS_radioBuffer[8]  = ((ECGvalue[2] &  0x0000FF00) >> 8);
  ADS_radioBuffer[9]  =  (ECGvalue[2] &  0x000000FF);
  ADS_radioBuffer[10] = ((ECGvalue[3] &  0x00FF0000) >> 16);
  ADS_radioBuffer[11] = ((ECGvalue[3] &  0x0000FF00) >> 8);
  ADS_radioBuffer[12] =  (ECGvalue[3] &  0x000000FF);
  ADS_radioBuffer[13] = ((ECGvalue[4] &  0x00FF0000) >> 16);
  ADS_radioBuffer[14] = ((ECGvalue[4] &  0x0000FF00) >> 8);
  ADS_radioBuffer[15] =  (ECGvalue[4] &  0x000000FF);
  ADS_radioBuffer[16] = ((ECGvalue[5] &  0x00FF0000) >> 16);
  ADS_radioBuffer[17] = ((ECGvalue[5] &  0x0000FF00) >> 8);
  ADS_radioBuffer[18] =  (ECGvalue[5] &  0x000000FF);
}

void ADS_sendSamplesBLE(){
  ADS_packetNumber++;
  if (ADS_packetNumber == 64) {
    ADS_packetNumber = 0;
  }

  ADS_radioBuffer[0] = (PKT_TYPE_ADS<<6);
  ADS_radioBuffer[0] |= ADS_packetNumber;
  //Serial.print(MAX_packetNumber,DEC);  Serial.print('\t'); Serial.print(MAX_radioBuffer[0],HEX); Serial.print('\n');
  ADS_radioBuffer[19] = 0;
  if(ADS_packetNumber == 25){ ADS_radioBuffer[19] = 0x22; }  // arbitrary thing I'm putting in here
  if(ADS_packetNumber == 26){ ADS_radioBuffer[19] = 0x99; }  // another arbitrary thing.
//      Serial.println();
  if (BLEconnected) {
    SimbleeBLE.send(ADS_radioBuffer, 20);
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
      Serial.println("start PPG");
      MAX_packetSampleNumber = -1;
      MAX_sampleCounter = 0;
      enableMAX30102(true);
      thatTestTime = micros();
      break;
    case 'a':
      Serial.println("start ADS");
      _ADS_RDATAC();
      delay(10);
      _ADS_START();
      break;
    case 's':
      Serial.println("stop running");
      enableMAX30102(false);
      _ADS_STOP();
      break;
    case 't':
      MAX30102_writeRegister(MAX_TEMP_CONFIG,0x01);
      break;
    case 'i':
      intSource = MAX30102_readShort(MAX_STATUS_1);
      Serial.print("intSource: 0x"); Serial.println(intSource,HEX);
      break;
    case 'v':
      _ADS_RESET();
      delay(10);
      MAX_getDeviceInfo();
      ADS_getDeviceID();
      break;
    case '?':
      MAX_printAllRegisters();
      printADSregisters();
      Serial.println();
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
  Serial.println(F("   'a'  Start the ECG running"));
  Serial.println(F("   'b'  Start the PPG running"));
  Serial.println(F("   's'  Stop the PPG and stop the ECG"));
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
 *  The regData array holds a mirror of the values
 *  in the 12 ADS control registers
 *
 ***************************************************/



 //System Commands
 void _ADS_WAKEUP() {
     digitalWrite(ADS_SS,LOW);
     SPI.transfer(ADS_WAKEUP);
     digitalWrite(ADS_SS,HIGH);
     delayMicroseconds(3);     //must wait 4 tCLK cycles before sending another command (Datasheet, pg. 35)
 }

 void _ADS_STANDBY() {    // only allowed to send WAKEUP after sending STANDBY
     digitalWrite(ADS_SS,LOW);
     SPI.transfer(ADS_STANDBY);
     digitalWrite(ADS_SS,HIGH);
 }

 void _ADS_RESET() {      // reset all the registers to default settings
     digitalWrite(ADS_SS,LOW);
     SPI.transfer(ADS_RESET);
     digitalWrite(ADS_SS,HIGH);
     delayMicroseconds(12);    //must wait 18 tCLK cycles to execute this command (Datasheet, pg. 35)
 }

 void _ADS_START() {      //start data conversion
     digitalWrite(ADS_SS,LOW);
     SPI.transfer(ADS_START);           // KEEP ON-BOARD AND ON-DAISY IN SYNC
     digitalWrite(ADS_SS,HIGH);
 }

 void _ADS_STOP() {     //stop data conversion
     digitalWrite(ADS_SS,LOW);
     SPI.transfer(ADS_STOP);            // KEEP ON-BOARD AND ON-DAISY IN SYNC
     digitalWrite(ADS_SS,HIGH);
 }

 void _ADS_RDATAC() {
     digitalWrite(ADS_SS,LOW);
     SPI.transfer(ADS_RDATAC);      // read data continuous
     digitalWrite(ADS_SS,HIGH);
     delayMicroseconds(3);
 }
 void _ADS_SDATAC() {
     digitalWrite(ADS_SS,LOW);
     SPI.transfer(ADS_SDATAC);
     digitalWrite(ADS_SS,HIGH);
     delayMicroseconds(10);   //must wait at least 4 tCLK cycles after executing this command (Datasheet, pg. 37)
 }

 //print out the state of all the control registers
 void printADSregisters()
 {
     boolean prevverbosityState = verbosity;
     verbosity = true;			// set up for verbosity output
     RREGS(0x00,0x0B);     	// read out the ADS registers
     verbosity = prevverbosityState;
 }

 byte ADS_getDeviceID() {      // simple hello world com check
     byte ID = RREG(ADS_ID);
//     if(verbosity){            // verbosity otuput
         Serial.print("On Board ADS ID ");
         printHex(ID); Serial.println();
//     }
     return ID;
 }


 // void RDATA() {          //  use in Stop Read Continuous mode when DRDY goes low
 //     byte inByte;            //  to read in one sample of the channels
 //     digitalWrite(ADS_SS,LOW);        //  open SPI
 //     SPI.transfer(ADS_RDATA);         //  send the RDATA command
 //     for(int i=0; i<3; i++){   //  read in the status register and new channel data
 //         inByte = SPI.transfer(0x00);
 //         ADSstatus = (ADSstatus<<8) | inByte; //  read status register (1100 + LOFF_STATP + LOFF_STATN + GPIO[7:4])
 //     }
 //     for(int i = 0; i<2; i++){
 //         for(int j=0; j<3; j++){   //  read in the new channel data
 //             inByte = SPI.transfer(0x00);
 //             ADS_channelDataRaw[i] = inByte;
 //             ADS_channelDataInt[i] = ADS_channelDataInt[i]<<8) | inByte;
 //         }
 //     }
 //         for(int i=0; i<8; i++){
 //             if(bitRead(boardChannelDataInt[i],23) == 1){  // convert 3 byte 2's compliment to 4 byte 2's compliment
 //             boardChannelDataInt[i] |= 0xFF000000;
 //         }else{
 //             boardChannelDataInt[i] &= 0x00FFFFFF;
 //         }
 //     }
 // }
 // digitalWrite(ADS_SS,HIGH);        //  close SPI
 //
 //
 // }


void delay10(){
  delayMicroseconds(10);
}
 

//  Read a single register from the ADS_SS
//  Receives the address to be read, returns the result
//  And stores the value in the mirror array
 byte RREG(byte _address) {    //  reads ONE register at _address
     byte opcode1 = (_address | 0x20);   //  RREG expects 001r rrrr where r rrrr = _address
     digitalWrite(ADS_SS,LOW);        //  open SPI
     delay10();
     SPI.transfer(opcode1);          //  opcode1
     delay10();
     SPI.transfer(0x00);           //  opcode2
     delay10();
     regData[_address] = SPI.transfer(0x00);//  update mirror location with returned byte
     delay10();
     digitalWrite(ADS_SS,HIGH);       //  close SPI
//     if (verbosity){           //  verbosity output
         printRegisterName(_address);
         printHex(_address);
         Serial.print(", ");
         printHex(regData[_address]);
         Serial.print(", ");
         for(byte j = 0; j<8; j++){
             Serial.print(bitRead(regData[_address], 7-j));
             if(j!=7) Serial.print(", ");
         }

         Serial.println();
//     }
     return regData[_address];     // return requested register value
 }


//  Read more than one register on the ADS
//  Receives the address to start reading at and the number to read -1
//  Reads the registers, and stores them in the mirror array
 void RREGS(byte _address, byte _numRegistersMinusOne) {

     byte inByte;
     byte opcode1 = (_address | 0x20);   //  RREG expects 001r rrrr where r rrrr = _address
     digitalWrite(ADS_SS,LOW);        //  open SPI
     delay10();
     SPI.transfer(opcode1);          //  opcode1
     delay10();
     SPI.transfer(_numRegistersMinusOne);  //  opcode2
     delay10();
     for(int i = 0; i <= _numRegistersMinusOne; i++){
         regData[_address + i] = SPI.transfer(0x00);   //  add register byte to mirror array
         delay10();
     }
     digitalWrite(ADS_SS,HIGH);       //  close SPI
//     if(verbosity){            //  verbosity output
         for(int i = 0; i<= _numRegistersMinusOne; i++){
             printRegisterName(_address + i);
             printHex(_address + i);
             Serial.print(", ");
             printHex(regData[_address + i]);
             Serial.println();
         }
//     }
 }

//  write to a single register on the ADS_SS
//  receives the address to write to and the value to write
//  updates the mirror array with the new value
 void WREG(byte _address, byte _value) { //  Write ONE register at _address
     byte opcode1 = (_address | 0x40);   //  WREG expects 010r rrrr where r rrrr = _address
     digitalWrite(ADS_SS,LOW);        //  open SPI
     delay10();
     SPI.transfer(opcode1);          //  Send WREG command & address
     delay10();
     SPI.transfer(0x00);           //  Send number of registers to write -1
     delay10();
     SPI.transfer(_value);         //  Write the value to the register
     delay10();
     digitalWrite(ADS_SS,HIGH);       //  close SPI
     regData[_address] = _value;     //  update the mirror array
     if(verbosity){            //  verbosity output
         Serial.print("Register ");
         printHex(_address);
         Serial.println(" modified.");
     }
 }

//  Write ADS registers
//  accepts the register address to start
//  and the number of registers to write to - 1
//  writes the regData mirror array to the ADS
 void WREGS(byte _address, byte _numRegistersMinusOne) {
     byte opcode1 = (_address | 0x40);   //  WREG expects 010r rrrr where r rrrr = _address
     digitalWrite(ADS_SS,LOW);        //  open SPI
     delay10();
     SPI.transfer(opcode1);          //  Send WREG command & address
     delay10();
     SPI.transfer(_numRegistersMinusOne);  //  Send number of registers to write -1
     delay10();
     for (int i=_address; i <=(_address + _numRegistersMinusOne); i++){
         SPI.transfer(regData[i]);     //  Write to the registers
         delay10();
     }
     digitalWrite(ADS_SS,HIGH);
     if(verbosity){
         Serial.print("Registers ");
         printHex(_address); Serial.print(" to ");
         printHex(_address + _numRegistersMinusOne);
         Serial.println(" modified");
     }
 }


 // Used for printing HEX in verbosity feedback mode
 void printHex(byte _data){
     Serial.print("0x");
     if(_data < 0x10) Serial.print("0");
     Serial.print(_data, HEX);
 }


 void printRegisterName(byte _address) {
     switch(_address){
         case ADS_ID:
         Serial.print("ADS_ID, "); break;
         case ADS_CONFIG1:
         Serial.print("ADS_CONFIG1, "); break;
         case ADS_CONFIG2:
         Serial.print("ADS_CONFIG2, "); break;
         case ADS_LOFF:
         Serial.print("ADS_LOFF, "); break;
         case ADS_CH1SET:
         Serial.print("ADS_CH1SET, "); break;
         case ADS_CH2SET:
         Serial.print("ADS_CH2SET, "); break;
         case ADS_RLD_SENS:
         Serial.print("ADS_RLD_SENS, "); break;
         case ADS_LOFF_SENS:
         Serial.print("ADS_LOFF_SENS, "); break;
         case ADS_LOFF_STAT:
         Serial.print("ADS_LOFF_STAT, "); break;
         case ADS_RESP1:
         Serial.print("ADS_RESP1, "); break;
         case ADS_RESP2:
         Serial.print("ADS_RESP2, "); break;
         case ADS_GPIO:
         Serial.print("ADS_GPIO, "); break;
         default:
         break;
     }
 }

////////////////////////////////////
//  dummy ADS funcions
////////////////////////////////////

 // void dummy_ADS_serviceInterrupts(){
 //   ADS_interrupt = false;
 //   readECG();
 // }


 // void dummy_readECG() {
 //   ADS_readFIFOdata();
 // }


 // read in the FIFO data three bytes per ADC result
 // void dummy_ADS_readFIFOdata(){
 //   char dataByte[] = {0x00, 0x00, 0x00};
 //   int byteCounter = 0;
 //   ADS_packetSampleNumber++;
 //   if(ADS_packetSampleNumber == 6){
 //     ADS_packetSampleNumber = 0;
 //   }
 //
 //   // Look at my awesomely fake data for channel 1!
 //   switch (ADS_packetSampleNumber) {
 //     case 0:
 //       // zero is fine
 //       break;
 //     case 1:
 //       dataByte[1] = 0xFF; // 0x00FF00 = 65,280
 //       break;
 //     case 2:
 //       dataByte[0] = 0x07; // 0x070000 = 458,752
 //       break;
 //     case 3:
 //       dataByte[0] = 0x70; // 0x700000 = 7,340,032
 //       break;
 //     case 4:
 //       dataByte[0] = 0xF7; // 0xF70000 = 16,187,392
 //       break;
 //     case 5:
 //       dataByte[0] = 0xF0; // 0xF00000 = 15,728,640
 //       break;
 //   }
 //
 //   ECGvalue[ADS_packetSampleNumber] = 0;
 //   ECGvalue[ADS_packetSampleNumber] = (dataByte[0] & 0xFF); ECGvalue[ADS_packetSampleNumber] <<= 8;
 //   ECGvalue[ADS_packetSampleNumber] |= dataByte[1]; ECGvalue[ADS_packetSampleNumber] <<= 8;
 //   ECGvalue[ADS_packetSampleNumber] |= dataByte[2];
 //
 //   ECGvalue[ADS_packetSampleNumber] &= 0x00FFFFFF;
 //
 //   if(OUTPUT_TYPE == OUTPUT_BLE && ADS_packetSampleNumber == 5){
//     ADS_packsamples();
//     ADS_sendSamplesBLE();
//   }
// }
