
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
    Serial.print(MAX_packetNumber,DEC); printTab();
    Serial.print(MAX_packetSampleNumber,DEC); printTab();
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
  if( (MAX_packetNumber & 0x3F) == 0){
    if(MAX_packetSampleNumber == 0){
      // At the beginning of packet #0 (mod 64):
      MAX30102_writeRegister(MAX_TEMP_CONFIG,0x01); // start a temperature conversion
    }
  } //  NOT SURE IF THIS BELONGS HERE
  MAX_readFIFOdata();
  // }  // OR IF IT BELONGS HERE
}

// read in the FIFO data three bytes per ADC result
void MAX_readFIFOdata(){
  char dataByte[6];
  int byteCounter = 0;
  long thisREDvalue;
  long thisIRvalue;
  int thisValleyLoc; // just used to prevent repeated calls into an array.
  long buffer_reference_packet_number; // the packet from which we start counting, when sending peak data from the buffer

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

  // interpret six bytes as two sensor values
  thisREDvalue = 0L;
  thisREDvalue = (dataByte[0] & 0xFF); thisREDvalue <<= 8;
  thisREDvalue |= dataByte[1]; thisREDvalue <<= 8;
  thisREDvalue |= dataByte[2];
  thisREDvalue &= 0x0003FFFF;

  thisIRvalue = 0L;
  thisIRvalue = (dataByte[3] & 0xFF); thisIRvalue <<= 8;
  thisIRvalue |= dataByte[4]; thisIRvalue <<= 8;
  thisIRvalue |= dataByte[5];
  thisIRvalue &= 0x0003FFFF;

  // USE THESE DATA IN TWO PLACES: 1) FOR SENDING RAW DATA, and 2) FOR FINDING PEAKS:

  // 1) SEND RAW DATA:
  // Fill the value arrays for packing packets:
  REDvalue[MAX_packetSampleNumber] = thisREDvalue;
  IRvalue[MAX_packetSampleNumber] = thisIRvalue;

  if(useFilter){
    filterHP(REDvalue[MAX_packetSampleNumber], IRvalue[MAX_packetSampleNumber]);
  }

  // Send a waveform packet
  if(OUTPUT_TYPE == OUTPUT_BLE && MAX_packetSampleNumber == 3){
    MAX_packWFMsamples();
    MAX_sendSamplesBLE();
  }

  // 2) FIND PEAKS:
  // Fill the buffer for the peak detection algorithm (but first average down to 100 SPS):
  if ( (MAX_packetSampleNumber == 0) || (MAX_packetSampleNumber == 2) ) {
    IR_buffer[IR_buffer_counter] = thisIRvalue;
    RED_buffer[IR_buffer_counter] = thisREDvalue;
  } else {
    IR_buffer[IR_buffer_counter] = (IR_buffer[IR_buffer_counter] + thisIRvalue)>>1; // average two samples
    RED_buffer[IR_buffer_counter] = (RED_buffer[IR_buffer_counter] + thisREDvalue)>>1; //average two samples
    IR_buffer_counter++;
  }

  // If the buffer is full, run the peak detection/hr/SpO2 algorithm and send a packet
  if (IR_buffer_counter==IR_BUFFER_LENGTH) {
    timeStart1 = micros();
    maxim_heart_rate_and_oxygen_saturation_details(IR_buffer, IR_BUFFER_LENGTH, RED_buffer, &MAX_avg_SpO2, &MAX_SpO2_valid, &MAX_avg_HR, &MAX_HR_valid, IR_valley_locs, &num_IR_valleys);
    // Correct SpO2 for -999 (invalid):
    if (MAX_avg_SpO2 < 0) {
      MAX_avg_SpO2 = 0;
    }
    buffer_reference_packet_number = (MAX_packetNumber & 0xFFFFFFC0)-64;
    Serial.print("Number of IR valleys: ");
    Serial.println(num_IR_valleys);
    //Process these peaks:
    for(int i=0; i<num_IR_valleys; i++) {
      thisValleyLoc = IR_valley_locs[i];
      // Only queue up the peaks that were in the last packet's worth of data (the buffer happens to contain additional info, just for better averaging purposes).
      if(thisValleyLoc < 3*128 && thisValleyLoc>=2*128 ) {
        IRvalleyQueue_packetNumber[IRvalleyQueue_length] = buffer_reference_packet_number + ((thisValleyLoc-2*128) >> 1);
        IRvalleyQueue_packetSampleNumber[IRvalleyQueue_length] = (thisValleyLoc & 0x01<<1); // the locations of valleys that are queued up and ready to send
        if (i==0) {
          IRvalleyQueue_instHR[IRvalleyQueue_length] = 0;
        } else {
          IRvalleyQueue_instHR[IRvalleyQueue_length] = 6000/(thisValleyLoc - IR_valley_locs[i-1]); // assuming 100 SPS
          if (IRvalleyQueue_instHR[IRvalleyQueue_length]>255) {IRvalleyQueue_instHR[IRvalleyQueue_length]=255; }
        }
        IRvalleyQueue_length++;
      }
    }
    // Now shift all the points to the left by 128, and adjust IR_buffer_counter accordingly
    shift_buffer(IR_buffer,IR_BUFFER_LENGTH,128);
    shift_buffer(RED_buffer,IR_BUFFER_LENGTH,128);
    IR_buffer_counter = IR_buffer_counter-128;
    // Send an AUX packet with this info:
    timeEnd1 = micros();
    dt1 += (timeEnd1-timeStart1);

    MAX_packAUXsamples(buffer_reference_packet_number,0); // "0" means this is the basic AUX packet
    MAX_sendSamplesBLE();
    // the packAUXsamples routine will pop things out of the queue.  If there are still peaks left in the queue, send another aux packet
    while (IRvalleyQueue_length>0) {
      MAX_packAUXsamples(buffer_reference_packet_number,1); // "1" means this is a bonus AUX packet, due to overflow
      MAX_sendSamplesBLE();
    }
  }

}

// Pack waveform samples into a buffer for transmission via BLE
void MAX_packWFMsamples(){
  MAX_packetNumber++;
  MAX_radioBuffer[0] = (PKT_TYPE_MAX_WFM<<6);
  MAX_radioBuffer[0] |= ((MAX_packetNumber & 0x3F00) >> 8);
  MAX_radioBuffer[1] = (MAX_packetNumber & 0xFF);

  MAX_radioBuffer[2] =  ((REDvalue[0] &  0x0003FC00) >> 10);
  MAX_radioBuffer[3] =  ((REDvalue[0] &  0x000003FC) >> 2);
  MAX_radioBuffer[4] =  ((REDvalue[0] &  0x00000003) << 6);
  MAX_radioBuffer[4] |= ((IRvalue[0] &  0x0003F000) >> 12);
  MAX_radioBuffer[5] =  ((IRvalue[0] &  0x00000FF0) >> 4);
  MAX_radioBuffer[6] =  ((IRvalue[0] &  0x0000000F) << 4);
  MAX_radioBuffer[6] |= ((REDvalue[1] &  0x0003C000) >> 14);
  MAX_radioBuffer[7] =  ((REDvalue[1] &  0x00003FC0) >> 6);
  MAX_radioBuffer[8] =  ((REDvalue[1] &  0x0000003F) << 2);
  MAX_radioBuffer[8] |= ((IRvalue[1] & 0x00030000) >> 16);
  MAX_radioBuffer[9] = ((IRvalue[1] &  0x0000FF00) >> 8);
  MAX_radioBuffer[10] = (IRvalue[1] &   0x000000FF);
  MAX_radioBuffer[11] = ((REDvalue[2] &  0x0003FC00) >> 10);
  MAX_radioBuffer[12] = ((REDvalue[2] &  0x000003FC) >> 2);
  MAX_radioBuffer[13] = ((REDvalue[2] &  0x00000003) << 6);
  MAX_radioBuffer[13] |= ((IRvalue[2] & 0x0003F000) >> 12);
  MAX_radioBuffer[14] = ((IRvalue[2] &  0x00000FF0) >> 4);
  MAX_radioBuffer[15] = ((IRvalue[2] &  0x0000000F) << 4);
  MAX_radioBuffer[15] |= ((REDvalue[3] & 0x0003C000) >> 14);
  MAX_radioBuffer[16] =  ((REDvalue[3] & 0x00003FC0) >> 6);
  MAX_radioBuffer[17] =  ((REDvalue[3] & 0x0000003F) << 2);
  MAX_radioBuffer[17] |= ((IRvalue[3] & 0x00030000) >> 16);
  MAX_radioBuffer[18] =  ((IRvalue[3] & 0x0000FF00) >> 8);
  MAX_radioBuffer[19] =   (IRvalue[3] & 0x000000FF);
}

// Pack AUX data into a buffer for transmission via BLE
void MAX_packAUXsamples(long refpktno, int bonus){
  int packed_samples;

  if (IRvalleyQueue_length>6) {
    packed_samples = 6;
  } else {
    packed_samples = IRvalleyQueue_length;
  }

  for(int i=0; i<20; i++) {
    MAX_radioBuffer[i] = 0;
  }

  Serial.println(packed_samples);

  MAX_radioBuffer[0]  = (PKT_TYPE_MAX_AUX<<6);
  MAX_radioBuffer[0] |= ((bonus & 0x01) << 5); // note there are three blank spaces (bits 4-6)
  MAX_radioBuffer[0] |= ((packed_samples & 0x07) << 2);
  MAX_radioBuffer[0] |= ((refpktno & 0xC00000) >> 22);
  MAX_radioBuffer[1]  = ((refpktno & 0x3FC000) >> 14);
  MAX_radioBuffer[2]  = ((refpktno & 0x003FC0) >> 6);

  for (int i=0; i<packed_samples; i++) {
    MAX_radioBuffer[3+2*i]  = ((IRvalleyQueue_packetNumber[i] & 0x3F) << 2);
    MAX_radioBuffer[3+2*i] |= ((IRvalleyQueue_packetSampleNumber[i] & 0x03) );
    MAX_radioBuffer[4+2*i]  = ((IRvalleyQueue_instHR[0] & 0xFF) );
  }

  shift_buffer(IRvalleyQueue_packetNumber,15,packed_samples);
  shift_buffer(IRvalleyQueue_packetSampleNumber,15,packed_samples);
  shift_buffer(IRvalleyQueue_instHR,15,packed_samples);
  IRvalleyQueue_length = IRvalleyQueue_length-packed_samples;

  if (bonus==0) {
    MAX_radioBuffer[15] |= ((MAX_HR_valid & 0x01) << 1);
    MAX_radioBuffer[15] |= ((MAX_SpO2_valid & 0x01));
    MAX_radioBuffer[16]  = MAX_avg_HR & 0xFF;
    MAX_radioBuffer[17]  = MAX_avg_SpO2 & 0xFF;
    MAX_radioBuffer[18]  = tempInteger;
    MAX_radioBuffer[19]  = tempFraction;
  }

  /*
  if (IRvalleyQueue_length>0) {
    MAX_radioBuffer[3]  = (0x01) << 7;
    MAX_radioBuffer[3] |= ((IRvalleyQueue_packetNumber[0] & 0x3F) << 1);
    MAX_radioBuffer[3] |= ((IRvalleyQueue_packetSampleNumber[0] & 0x02) >> 1);
    MAX_radioBuffer[4]  = ((IRvalleyQueue_packetSampleNumber[0] & 0x01) << 7);
    MAX_radioBuffer[4] |= ((IRvalleyQueue_instHR[0] & 0xFE) >> 1);
    MAX_radioBuffer[5]  = ((IRvalleyQueue_instHR[0] & 0x01) << 7);
    packed_samples++;
  }
  if (IRvalleyQueue_length>1) {
    MAX_radioBuffer[5] |= (0x01) << 6;
    MAX_radioBuffer[5] |= ((IRvalleyQueue_packetNumber[1] & 0x3F) );
    MAX_radioBuffer[6]  = ((IRvalleyQueue_packetSampleNumber[1] & 0x03) << 6);
    MAX_radioBuffer[6] |= ((IRvalleyQueue_instHR[1] & 0xFC) >> 2);
    MAX_radioBuffer[7]  = ((IRvalleyQueue_instHR[1] & 0x03) << 6);
    packed_samples++;
  }
  if (IRvalleyQueue_length>2) {
    MAX_radioBuffer[7] |= (0x01) << 5;
    MAX_radioBuffer[7] |= ((IRvalleyQueue_packetNumber[2] & 0x3E) >> 1);
    MAX_radioBuffer[8]  = ((IRvalleyQueue_packetNumber[2] & 0x01) << 7);
    MAX_radioBuffer[8] |= ((IRvalleyQueue_packetSampleNumber[2] & 0x03) << 5);
    MAX_radioBuffer[8] |= ((IRvalleyQueue_instHR[2] & 0xF8) >> 3);
    MAX_radioBuffer[9]  = ((IRvalleyQueue_instHR[2] & 0x07) << 5);
    packed_samples++;
  }
  if (IRvalleyQueue_length>3) {
    MAX_radioBuffer[9]  |= (0x01) << 4;
    MAX_radioBuffer[9]  |= ((IRvalleyQueue_packetNumber[3] & 0x3C) >> 2);
    MAX_radioBuffer[10]  = ((IRvalleyQueue_packetNumber[3] & 0x03) << 6);
    MAX_radioBuffer[10] |= ((IRvalleyQueue_packetSampleNumber[3] & 0x03) << 4);
    MAX_radioBuffer[10] |= ((IRvalleyQueue_instHR[3] & 0xF0) >> 4);
    MAX_radioBuffer[11]  = ((IRvalleyQueue_instHR[3] & 0x0F) << 4);
    packed_samples++;
  }
  if (IRvalleyQueue_length>4) {
    MAX_radioBuffer[11] |= (0x01) << 3;
    MAX_radioBuffer[11] |= ((IRvalleyQueue_packetNumber[4] & 0x38) >> 3);
    MAX_radioBuffer[12]  = ((IRvalleyQueue_packetNumber[4] & 0x07) << 5);
    MAX_radioBuffer[12] |= ((IRvalleyQueue_packetSampleNumber[4] & 0x03) << 3);
    MAX_radioBuffer[12] |= ((IRvalleyQueue_instHR[4] & 0xE0) >> 5);
    MAX_radioBuffer[13]  = ((IRvalleyQueue_instHR[4] & 0x1F) << 3);
    packed_samples++;
  }
  if (IRvalleyQueue_length>5) {
    MAX_radioBuffer[13] |= (0x01) << 2;
    MAX_radioBuffer[13] |= ((IRvalleyQueue_packetNumber[5] & 0x30) >> 4);
    MAX_radioBuffer[14]  = ((IRvalleyQueue_packetNumber[5] & 0x0F) << 4);
    MAX_radioBuffer[14] |= ((IRvalleyQueue_packetSampleNumber[5] & 0x03) << 2);
    MAX_radioBuffer[14] |= ((IRvalleyQueue_instHR[5] & 0xC0) >> 6);
    MAX_radioBuffer[15]  = ((IRvalleyQueue_instHR[5] & 0x3F) << 2);
    packed_samples++;
  }

  if (bonus==0) {
    MAX_radioBuffer[15] |= ((MAX_MAX_HR_valid & 0x01) << 1);
    MAX_radioBuffer[15] |= ((MAX_SpO2_valid & 0x01));
    MAX_radioBuffer[16]  = MAX_avg_HR & 0xFF;
    MAX_radioBuffer[17]  = MAX_avg_SpO2 & 0xFF;
    MAX_radioBuffer[18]  = tempInteger;
    MAX_radioBuffer[19]  = tempFraction;
  } else {
    if (IRvalleyQueue_length>6) {
      MAX_radioBuffer[15] |= (0x01) << 1;
      MAX_radioBuffer[15] |= ((IRvalleyQueue_packetNumber[6] & 0x20) >> 5);
      MAX_radioBuffer[16]  = ((IRvalleyQueue_packetNumber[6] & 0x1F) << 3);
      MAX_radioBuffer[16] |= ((IRvalleyQueue_packetSampleNumber[6] & 0x03) << 1);
      MAX_radioBuffer[16] |= ((IRvalleyQueue_instHR[6] & 0x80) >> 7);
      MAX_radioBuffer[17]  = ((IRvalleyQueue_instHR[6] & 0x7F) << 1);
      packed_samples++;
    }
    if (IRvalleyQueue_length>7) {
      MAX_radioBuffer[17] |= (0x01);
      MAX_radioBuffer[18]  = ((IRvalleyQueue_packetNumber[7] & 0x3F) << 2);
      MAX_radioBuffer[18] |= ((IRvalleyQueue_packetSampleNumber[7] & 0x03) );
      MAX_radioBuffer[19]  = ((IRvalleyQueue_instHR[7] & 0xFF) );
      packed_samples++;
    }
  }
  */

}

// Send the MAX packet via BLE
void MAX_sendSamplesBLE(){
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
    WREG(ADS_CONFIG1,0x02); // sample rate of 500SPs
    WREG(ADS_CONFIG2,0xA0); // use the internal reference
    WREG(ADS_CH1SET,0x60);  // enable input normal on channel 1
    WREG(ADS_CH2SET,0x60);  // enable input normal on channel 2
    printADSregisters();    // report register values for confirmation

}


//  Start the ADS running
 void startADS(void)
 {
     ADS_sampleCounter = 0;
     ADS_internalCounter = 0;
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
     for(int i=0; i<3; i++){
         inByte = SPI.transfer(0x00);    //  read status register (1100 + LOFF_STATP + LOFF_STATN + GPIO[7:4])
         ADSstatus = (ADSstatus << 8) | inByte;
     }
     for(int i = 0; i<2; i++){
       ADS_channelDataInt[i] = 0;
         for(int j=0; j<3; j++){   //  read 24 bits of channel data in 8 3 byte chunks
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
     ECGvalue[ADS_internalCounter] = ADS_channelDataInt[0]; // collect channel 1 samples
     ADS_internalCounter++;
     if(ADS_internalCounter == 6){
       ADS_internalCounter = 0;
       if(BLEconnected){
         ADS_packsamples();
         ADS_sendSamplesBLE();
       }
     }
     if(OUTPUT_TYPE == OUTPUT_NORMAL){
       for(int i=0; i<2; i++){
         Serial.print(ADS_channelDataInt[i]); Serial.print("\t");
       }
       Serial.println(); // verbosity
     }
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
  int modPacketNumber = (ADS_packetNumber & 0x3F); // What is the packet number, mod 64?

  ADS_radioBuffer[0] = (PKT_TYPE_ADS_WFM<<6);
  ADS_radioBuffer[0] |= (ADS_packetNumber & 0x3F);
  //Serial.print(MAX_packetNumber,DEC);  Serial.print('\t'); Serial.print(MAX_radioBuffer[0],HEX); Serial.print('\n');
  ADS_radioBuffer[19] = 0;
  if(modPacketNumber == 0){ ADS_radioBuffer[19] = 0xFF & (ADS_packetNumber>>6); }  // For fun, put more of the packet number here
  if(modPacketNumber == 25){ ADS_radioBuffer[19] = 0x22; }  // arbitrary thing I'm putting in here
  if(modPacketNumber == 26){ ADS_radioBuffer[19] = 0x99; }  // another arbitrary thing.
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
      Serial.println("start running");
      MAX_packetSampleNumber = -1;
      MAX_packetNumber = 0;
      IR_buffer_counter = 0;
      IRvalleyQueue_length = 0;
      enableMAX30102(true);
      startADS();
      thatTestTime = micros();
      timeStartAll = millis();
      dt1 = 0;
      break;
    case 'a':
      Serial.println("start ADS");
      runTimer = millis();
      startADS();
      break;
    case 's':
      Serial.println("stop running");
      enableMAX30102(false);
      stopADS();
      timeEndAll = millis();
      dtAll = timeEndAll-timeStartAll;
      Serial.print("Run time: ");
      Serial.print(dtAll);
      Serial.println(" ms");
      Serial.print("Time 1: ");
      Serial.print(dt1/1000);
      Serial.print(" ms (");
      Serial.print(dt1/10/dtAll); // (dt_us/1000)/(dt_ms) *100 to get to %
      Serial.println("% cpu)");
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



/***************************************************
 *
 *  GENERAL HELPER FUNCTIONS
 *
 ***************************************************/


// shifts data in the_buffer, which has length buffer_len, to the left by shift_amt spots
void shift_buffer(uint32_t *the_buffer, uint32_t buffer_len, int shift_amt) {
  for(int i=shift_amt; i<buffer_len; i++) {
    the_buffer[i-shift_amt] = the_buffer[i];
  }
}
