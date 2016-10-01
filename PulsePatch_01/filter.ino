/*

  SIMPLE HIGH PASS FILTER

*/


float RC;
float dt;
float a;
float filterInputRED[NUM_SAMPLES];
float filterOutputRED[NUM_SAMPLES];
float filterInputIR[NUM_SAMPLES];
float filterOutputIR[NUM_SAMPLES];

void initFilter(){
  RC = 1.0/(CUTTOFF*2*PI);
  dt = 1.0/sampleRate;
  a = RC/(RC+dt);
  for(int i=0; i<NUM_SAMPLES; i++){
    filterInputRED[i] = 0.0; filterOutputRED[i] = 0.0;
    filterInputIR[i] = 0.0; filterOutputIR[i] = 0.0;
  }
}

void filterHP(int rVal, int irVal){
  filterOutputRED[0] = filterInputRED[0] = float(rVal); 
  filterOutputIR[0] = filterInputIR[0] = float(irVal);
  for(int i=1; i<NUM_SAMPLES; i++){
    filterOutputRED[i] = a*(filterOutputRED[i-1]+filterInputRED[i]-filterInputRED[i-1]);
    filterOutputIR[i] = a*(filterOutputIR[i-1]+filterInputIR[i]-filterInputIR[i-1]);
    
    filterInputRED[i] = filterInputRED[i-1];  // move the input array along
    filterInputIR[i] = filterInputIR[i-1];  // move the input array along
  }
}

