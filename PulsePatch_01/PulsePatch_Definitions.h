
/*
  control bytes and register definitions for controling MAX30102
*/

//  SIMBLEE PINS
#define RED_LED 22  // red LED
#define GRN_LED 28  // green LED
#define TACT_SWITCH 30
#define MAX_INT 12    // MAX30102 interrupts on this Simblee pin
#define SCL_PIN 10
#define SDA_PIN 13
#define S25_SS  19
#define WP  17
#define CS  18
#define S25_RST 16
#define HOLD  15

// OUTPUT TYPES
#define OUTPUT_NORMAL 0
#define OUTPUT_PLOTTER 1
#define OUTPUT_BLE 2

// DATA PACKET FORMATS
#define PKT_TYPE_MAX 0x00
#define PKT_TYPE_ADS 0x01

//  MAX30102 REGISTERS
#define MAX_ADD   0x57  // slave address
#define MAX_STATUS_1  0x00
#define MAX_STATUS_2  0x01
#define MAX_ENABLE_1  0x02
#define MAX_ENABLE_2  0x03
#define MAX_FIFO_WRITE  0x04
#define MAX_OVF_COUNTER 0x05
#define MAX_FIFO_READ 0x06
#define MAX_FIFO_DATA 0x07
#define MAX_FIFO_CONFIG 0x08
#define MAX_MODE_CONFIG 0x09
#define MAX_SPO2_CONFIG 0x0A
#define MAX_RED_PA 0x0C
#define MAX_IR_PA 0x0D
#define MAX_PILOT_PA  0x10
#define MAX_MODE_CNTRL_1  0x11
#define MAX_MODE_CNTRL_2  0x12
#define MAX_TEMP_INT  0x1F
#define MAX_TEMP_FRAC 0x20
#define MAX_TEMP_CONFIG 0x21
#define MAX_PROX_INT_THRESH 0x30
#define MAX_REV_ID  0xFE
#define MAX_PART_ID 0xFF

// MAX30102 MASKS
#define MAX_A_FULL  0x80
#define MAX_PPG_RDY 0x40
#define MAX_ALC_OVF 0x20
#define MAX_PROX_INT 0x10
#define MAX_PWR_RDY 0x01
#define MAX_TEMP_RDY  0x02
#define MAX_SMP_AVE_1 0x00
#define MAX_SMP_AVE_2 0x20
#define MAX_SMP_AVE_4 0x40
#define MAX_SMP_AVE_8 0x60
#define MAX_SMP_AVE_16 0x80
#define MAX_SMP_AVE_32 0xA0
#define MAX_ROLLOVER_EN 0x10
#define MAX_SHUTDOWN  0x80
#define MAX_RESET   0x40
#define MAX_HR_MODE 0x02
#define MAX_SPO2_MODE 0x03
#define MAX_MULTI_MODE 0x07
#define MAX_ADC_RGE_2048  0x00
#define MAX_ADC_RGE_4096  0x20
#define MAX_ADC_RGE_8192  0x40
#define MAX_ADC_RGE_16348 0x60
#define MAX_SR_50 0x00
#define MAX_SR_100  0x04
#define MAX_SR_200  0x08
#define MAX_SR_400  0x0C
#define MAX_SR_800  0x10
#define MAX_SR_1000 0x14
#define MAX_SR_1600 0x18
#define MAX_SR_3200 0x1C
#define MAX_PW_69 0x00
#define MAX_PW_118  0x01
#define MAX_PW_215  0x02
#define MAX_PW_411  0x03
#define MAX_TEMP_EN 0x01

// filter stuff
#define CUTTOFF_LOW 0.5
#define CUTTOFF_HIGH 10.0
#define NUM_SAMPLES 10
