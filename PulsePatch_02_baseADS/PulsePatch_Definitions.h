
/*
  control bytes and register definitions for controling MAX30102
*/

//  SIMBLEE PINS
#define RED_LED 22  // red LED
#define GRN_LED 28  // green LED
#define TACT_SWITCH 30
#define MAX_INT 14    // MAX30102 interrupts on this Simblee pin
#define SCL_PIN 10
#define SDA_PIN 13
#define ADS_SS  2
//#define MISO  3
//#define MOSI  5
//#define SCLK  4
#define ADS_DRDY 16

// OUTPUT TYPES
#define OUTPUT_NORMAL 0
#define OUTPUT_PLOTTER 1
#define OUTPUT_BLE 2

// DATA PACKET FORMATS
#define PKT_TYPE_MAX_WFM 0x00
#define PKT_TYPE_ADS_WFM 0x02

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

// ADS REGISTERS
#define ADS_ID  0x00
#define ADS_CONFIG1 0x01
#define ADS_CONFIG2 0x02
#define ADS_LOFF  0x03
#define ADS_CH1SET  0x04
#define ADS_CH2SET  0x05
#define ADS_RLD_SENS  0x06
#define ADS_LOFF_SENS 0x07
#define ADS_LOFF_STAT 0x08
#define ADS_RESP1 0x09
#define ADS_RESP2 0x0A
#define ADS_GPIO  0x0B

// ADS Command Definitions
#define ADS_WAKEUP 0x02 // Wake-up from standby mode
#define ADS_STANDBY 0x04 // Enter Standby mode
#define ADS_RESET 0x06 // Reset the device registers to default
#define ADS_START 0x08 // Start and restart (synchronize) conversions
#define ADS_STOP 0x0A // Stop conversion
#define ADS_OFFSETCAL 0x1A  // Offset Calibration
#define ADS_RDATAC 0x10 // Enable Read Data Continuous mode (default mode at power-up)
#define ADS_SDATAC 0x11 // Stop Read Data Continuous mode
#define ADS_RDATA 0x12 // Read data by command supports multiple read back

// filter stuff
#define CUTTOFF_LOW 0.5
#define CUTTOFF_HIGH 10.0
#define NUM_SAMPLES 10
