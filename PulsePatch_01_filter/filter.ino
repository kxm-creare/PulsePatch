/*

  SIMPLE HIGH PASS FILTER

*/


float RC;
float dt;
float a;
float filterInput[NUM_SAMPLES];
float filterOutput[NUM_SAMPLES];


void initFilter(){
  RC = 1.0/(CUTTOFF*2*PI);
  dt = 1.0/sampleRate;
  a = RC/(RC+dt);
}

void filterHP(){
  filterOutput[0] = filterInput[0];
  for(int i=1; i<NUM_SAMPLES; i++){
    filterOutput[i] = a*(filterOutput[i-1]+filterInput[i]-filterInput[i-1]);
    filterInput[i] = filterInput[i-1];  // move the input array along
  }
}

