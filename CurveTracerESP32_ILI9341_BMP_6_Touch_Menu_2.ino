//---------------------------------------------------------------
// 
//--------------------------------------------------------------------------------------------------------------------------------------
// 2023_03_16 copy from CurveTracer_2_DacV_20, but other dividers on A0,A1,A2,A3
// 2023_03_18 second encoder, only PIN_EB (not PIN_BE) for p-devices (pin 1 CCS normal high, switch automatically to low, if PIN_EB high)
// 2023_03-21 sign µ in 2/1 on position of tile (~)
// 2024_01_04 CurveTracerRP2040_Adafruit_ILI9341_SD_01: change µ-proceesor to RP2040
// 2024_01_04 CurveTracerRP2040_Adafruit_ILI9341_SD_02: test isr and encoder ("pio_encoder")
// 2024_01_16 CurveTracerESP32_ILI9341_SD_03: SD-Card , SerialBT instead of HM010
// 2024_04_20 CurveTracerESP32_ILI9341_BMP_5: SD-Card , SerialBT, new libs for MCP3204 and MCP4822, isr update
// 2024_06_01 CurveTracerESP32_ILI9341_BMP_6: s_zen: suppress zeo-point x-scale, start curve from volt s_zen
//--------------------------------------------------------------------------------------------------------------------------------------

//#define DEBUG 1
#if DEBUG == 1
#define debug(x)   Serial.print(x);
#define debugb(x)  Serial.print(x);Serial.print(" ");
#define debugln(x) Serial.print(x);Serial.print("\n");
#define debuglf()  Serial.print("\n");
#else
#define debug(x)
#define debugb(x)
#define debugln(x)
#define debuglf()
#endif

#define TOUCH
#define DAC_INT
#define BLUE_T
#define BMP
#define IEPROM
#define SD_C
//#define NAME_D
//#define OSCI

#include <Arduino.h>

#include <SPI.h>

#ifdef SD_C
#include <SD.h>  //###################
#endif

//SPISettings settingsT(10000000, MSBFIRST, SPI_MODE0); // TFT 12 MHz, cpu 240 MHz

#include "TFT_eSPI.h"
#include "Free_Fonts.h"   // Include the header file attached to this sketch

TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

#define TFT_GREY 0x5AEB

#ifdef IEPROM         // internal EEPROM
#include "EEPROM.h"
#define EEPROM_SIZE 256
#endif

#include "MCP_ADC.h"

// ESP32 PINS for HSPI :  CLK=14, MISO=12, MOSI=13
// ESP32 PINS for VSPI :  CLK=18, MISO=19, MOSI=23

MCP3204 MCP_A;    // use HWSPI  on ESP32 (apparently VSPI)

#include <Wire.h>

#include "Adafruit_MCP4725.h"

Adafruit_MCP4725 dac_i2c;        // 12 bit DAC (I2C Addrees : 60

#ifdef BLUE_T
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
#endif

//#define EEPROM_I2C_ADDRESS 0x50  // EEPROM 24LC64 I2C Address  //24LC

const int pin_DAC_CS        =  0;  // MCP4822 DAC
const int TX_D0             =  1;  // TxD0 for send data to PC
const int pin_ADC_CS        =  2;  // MCP3204 ADC//GPIO2  on board LED, must be left floating or LOW to enter flashing mode
const int RX_D0             =  3;  // RxD0 for TT
const int pin_EB            =  4;  // switch emitter / base (polarity)
//const int TFT_CS          =  5;  // V-SPI CS
//                          =  6;  // Flash
//                          =  7;  // Flash
//                          =  8;  // Flash
//                          =  9;  // Flash
//                          = 10;  // Flash
//                          = 11;  // Flash
const int pin_E_NEG         = 12;  // switch Emitter/Source/Kathode to Rs (near Gnd)   ,Collector ... to Output TCA0372)(for npn-Tran,n-MOS
//free                      = 13;  // free 
const int pin_E_POS         = 14;  // switch Emitter/Source/Kathode to (output max+12V) (for pnp-Tran,p-MOS,p-JFET,Diode(backward biased))
const int pin_CV_SW         = 15;  // switch Constant-Current to Constant-Voltage for (Mos-Fet and J-Fet)
#ifdef SD_C
const int pin_SD_CS         = 16;  // SD_CS
#endif
//const int TFT_DC          = 17;  // V-SPI DC
//const int TFT_CLK         = 18;  // V-SPI CLK
//const int TFT_MISO        = 19;  // V-SPI MISO
//                           = 20;  // ??? na
//const int I2C_SDA         = 21;  // SDA I2C for MCP4725 (DAC-base)
//const int I2C_SCL         = 22;  // SCL I2C for MCP4725 (DAC-base)
//const int TFT_MOSI        = 23;  // V-SPI MOSI
//                           = 24;  // ??? na
#ifdef OSCI
#ifdef DAC_INT 
const int pin_DAC1          = 25;  // DAC1
#endif
#endif  
const int pin_TOUCH_IRQ     = 26;  // Touch-IRQ test   // DAC2                  
const int pin_TOUCH_CS      = 27;  // Touch-CS here not used
//                           = 28;  // ??? na
//                           = 29;  // ??? na
//                           = 30;  // ??? na
//                           = 31;  // ??? na
const int Pin_32_BUT        = 32;  // PushButton  ISR-Pin
#ifdef OSCI
const int pin_ADC1_5        = 33;  // ADC1_5
#endif 
const int pin_ENC1_A        = 34;  // Encoder-1-A ISR-Pin, Pull-Up 10k ?
const int pin_ENC1_B        = 35;  // Encoder-1-B IST-Pin, Pull-Up 10k ?
const int pin_ENC2_A        = 36;  // Encoder-2-A ISR-Pin, Pull-Up 10k ?
const int pin_ENC2_B        = 39;  // Encoder-2-B ISR-Pin, Pull-Up 10k ?

const int pin_DAC_LDAC      = -1;  // MCP4822

//#include "MCP4822.h"
//MCP4822 dac = MCP4822(pin_DAC_CS);//,pin_DAC_LDAC);

#include "MCP_DAC.h"

//  HSPI uses default   SCLK=14, MISO=12, MOSI=13, SELECT=15
//  VSPI uses default   SCLK=18, MISO=19, MOSI=23, SELECT=5
SPIClass * myspi = new SPIClass(VSPI);
MCP4822 MCP_D(myspi);  // HW SPI

const int TFT_WID   =  320;
const int TFT_HGT   =  240;
#ifdef OSCI

int16_t inpBuf2[TFT_WID];
int16_t inpBuf3[TFT_WID];

uint8_t newBuf2[TFT_WID], oldBuf2[TFT_WID];
uint8_t newBuf3[TFT_WID], oldBuf3[TFT_WID];
int pos = 0;                      // the next position in the value array to read
int count = 0;                    // the total number of times through the loop
unsigned long readStartTime = 0;  // time when the current sampling started
int lastStart = 0;                // the sample at which drawing started last time (need to know this to clear the old line properly)
int16_t val_A1 = 0,val_A2,val_A3;
// Used by vsprintf.
//char buffer[32];
uint16_t SineValues[256];       // an array to store our values for sine

#endif

//const int ADC_MAX = 4095; // 12 bit MCP3402 1024; // 10 bit arduino UNO
//const int DAC_MAX = 4095;
const int tsd       = 1000;

const int mAmax       = 50; // Ic for top of screen
const int Vmax        = 12; // 12V right end of screen
const uint16_t col_t[] = {TFT_CYAN, TFT_YELLOW, TFT_MAGENTA,/* TFT_BRIGHTBLUE,*/ TFT_ORANGE,
                          TFT_PINK, /*TFT_LIGHTBLUE, TFT_LIGHTRED,*/ TFT_WHITE
                         };

const float count_to_mv   = 5020.0 / 4096.0;    // = 1.2129=Vref/maxcount ADC  //Vref = 5.02V ?? not 4.968
//const float adc_refmaxcnt = 0.0049512; //4.0*5.07/4096.0;  // MC3204  -Reference      = U-5V/12bit ADC; divider 4: 30k/10k
//const float adc0_refmaxcnt = 0.0024756; // 4949; //2.0*5.07/4096.0;  // MC3204  -Reference      = U-5V/12bit ADC; divider 2: 10k/10k A0
//const float adc1_refmaxcnt = 0.0037134; // 4949; //3.0*5.07/4096.0;  // MC3204  -Reference      = U-5V/12bit ADC; divider 3: 20k/10k A1
//const float adc_refmaxcnt = 1.110.0/1024.0;   // Internal-Reference NANO = 1.1V/10bit ADC; // divider 4.25V/1.1V ?
//const float adc_refmaxcnt = 4.41545/1024.0;   // normal  -Reference NANO = U-5V/10bit ADC; // ca 4.42V on USB
//const float adc_refmaxcnt = 4.0*5.07/4098.0;  // MC3204  -Reference      = U-5V/12bit ADC; divider
//const char seper = ',';  // seperator for Bluetooth-transfer


uint8_t bmpPNP[] PROGMEM = {
  21, 0, // width
  30, 0, // height
  0x3F, 0xFF, 0xF9, 0xFF, 0xFF, 0xCF, 0xFF, 0xFE, 0x7F, 0xFF, 0xF1, 0xFF, 0x3F, 0xC7, 0xF9, 0xFF,
  0x1F, 0xCF, 0xFC, 0x7E, 0x7F, 0xF1, 0xF3, 0xFF, 0xC7, 0x9F, 0xFF, 0x1C, 0xFF, 0xFC, 0x67, 0xFF,
  0xF1, 0x3F, 0xFF, 0xC1, 0xFF, 0xFF, 0x00, 0x1F, 0xFC, 0x00, 0xFF, 0x93, 0xFF, 0xF0, 0x9F, 0xFE,
  0x0C, 0xFF, 0xC0, 0x67, 0xFC, 0x07, 0x3F, 0xF0, 0x39, 0xFF, 0x83, 0xCF, 0xF8, 0x1E, 0x7F, 0x8D,
  0xF3, 0xF8, 0xFF, 0x9F, 0xCF, 0xFF, 0xFE, 0x7F, 0xFF, 0xF3, 0xFF, 0xFF, 0x9F, 0xFF, 0xFC
};

uint8_t bmpNPN[] PROGMEM = {
  21, 0, // width
  30, 0, // height
  0xFF, 0xFF, 0xE7, 0xFF, 0xFF, 0x3F, 0xFF, 0xF9, 0xFF, 0xFF, 0xCF, 0xE7, 0xFC, 0x7F, 0x3F, 0xC7,
  0xF9, 0xFC, 0x7F, 0xCF, 0xC7, 0xFE, 0x7C, 0x7F, 0xF3, 0xC7, 0xFF, 0x9C, 0x7F, 0xFC, 0xC7, 0xFF,
  0xE4, 0x7F, 0xFF, 0x07, 0xFC, 0x00, 0x7F, 0xE0, 0x03, 0xFF, 0xFE, 0x0F, 0xFF, 0xF2, 0x3B, 0xFF,
  0x98, 0x9F, 0xFC, 0xE0, 0x7F, 0xE7, 0x83, 0xFF, 0x38, 0x0F, 0xF9, 0x80, 0x7F, 0xCF, 0x01, 0xFE,
  0x7E, 0x0F, 0xF3, 0xFC, 0x3F, 0xFF, 0xF9, 0xFF, 0xFF, 0xCF, 0xFF, 0xFE, 0x7F, 0xFF, 0xF0
};

uint8_t bmpNMOSFET[] PROGMEM = {
  23, 0, // width
  34, 0, // height
  0xFF, 0xFF, 0xF9, 0xFF, 0xFF, 0xF3, 0xFF, 0xFF, 0xE7, 0xFF, 0xFF, 0xCF, 0xFF, 0xFF, 0x9F, 0xFF,
  0xFF, 0x3F, 0xFF, 0xFE, 0x7F, 0xFF, 0xFC, 0xFE, 0x60, 0x01, 0xFC, 0xC0, 0x03, 0xF9, 0x9F, 0xFF,
  0xF3, 0x3F, 0xFF, 0xE7, 0xFF, 0xFF, 0xCF, 0xFB, 0xFF, 0x99, 0xC7, 0xFF, 0x32, 0x0F, 0xFE, 0x60,
  0x01, 0xFC, 0xC0, 0x03, 0xF9, 0x90, 0x67, 0xF3, 0x38, 0xCF, 0xE7, 0xFD, 0x9F, 0xCF, 0xFF, 0x3F,
  0x99, 0xFE, 0x7F, 0x33, 0xFC, 0x00, 0x60, 0x00, 0x00, 0xC0, 0x03, 0xFF, 0xFF, 0xE7, 0xFF, 0xFF,
  0xCF, 0xFF, 0xFF, 0x9F, 0xFF, 0xFF, 0x3F, 0xFF, 0xFE, 0x7F, 0xFF, 0xFC, 0xFF, 0xFF, 0xF9, 0xFF,
  0xFF, 0xF0
};

uint8_t bmpPMOSFET[] PROGMEM = {

  23, 0, // width
  34, 0, // height
  0x3F, 0xFF, 0xFE, 0x7F, 0xFF, 0xFC, 0xFF, 0xFF, 0xF9, 0xFF, 0xFF, 0xF3, 0xFF, 0xFF, 0xE7, 0xFF,
  0xFF, 0xCF, 0xFF, 0xFF, 0x9F, 0xFF, 0xFF, 0x00, 0x0C, 0xFE, 0x00, 0x19, 0xFF, 0xFF, 0x33, 0xFF,
  0xFE, 0x67, 0xFF, 0xFF, 0xCF, 0xFF, 0xBF, 0x9F, 0xFC, 0x73, 0x3F, 0xE0, 0xE6, 0x7F, 0x80, 0x0C,
  0xFE, 0x00, 0x19, 0xFC, 0x07, 0x33, 0xF9, 0x8E, 0x67, 0xF3, 0xDF, 0xCF, 0xE7, 0xFF, 0x9F, 0xCF,
  0xF3, 0x3F, 0x9F, 0xE6, 0x7F, 0x00, 0x0C, 0x00, 0x00, 0x18, 0x00, 0xFF, 0xFF, 0xF9, 0xFF, 0xFF,
  0xF3, 0xFF, 0xFF, 0xE7, 0xFF, 0xFF, 0xCF, 0xFF, 0xFF, 0x9F, 0xFF, 0xFF, 0x3F, 0xFF, 0xFE, 0x7F,
  0xFF, 0xFC
};

uint8_t bmpNJFET[] PROGMEM = {
  21, 128, // width (run-length encoded)
  30, 0, // height
  0x02, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02, 0x05, 0x02, 0x0C,
  0x02, 0x05, 0x02, 0x0C, 0x09, 0x0C, 0x09, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02, 0x06, 0x01, 0x0C, 0x02, 0x04,
  0x03, 0x0C, 0x02, 0x02, 0x05, 0x05, 0x2C, 0x05, 0x02, 0x02, 0x05, 0x05, 0x02, 0x05, 0x02, 0x04, 0x03, 0x05, 0x02, 0x0D,
  0x01, 0x05, 0x02, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02, 0x13
};

uint8_t bmpPJFET[] PROGMEM = {
  21, 128, // width (run-length encoded)
  30, 0, // height
  0x00, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02, 0x0C, 0x02, 0x05,
  0x02, 0x0C, 0x02, 0x05, 0x02, 0x0C, 0x09, 0x0C, 0x09, 0x0C, 0x02, 0x13, 0x02, 0x13, 0x02, 0x0F, 0x01, 0x03, 0x02, 0x0D,
  0x03, 0x03, 0x02, 0x0B, 0x05, 0x03, 0x02, 0x07, 0x2A, 0x04, 0x05, 0x03, 0x02, 0x05, 0x02, 0x06, 0x03, 0x03, 0x02, 0x05,
  0x02, 0x08, 0x01, 0x0A, 0x02, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02, 0x13, 0x02
};

uint8_t bmpPDiodeBig[] PROGMEM = {
  20, 128, // width run-length encoded
  24, 0, // height
  0x00, 0x09, 0x02, 0x12, 0x02, 0x12, 0x02, 0x12, 0x02, 0x12, 0x02, 0x12, 0x02, 0x09, 0x14, 0x01, 0x12, 0x03, 0x10, 0x05,
  0x0E, 0x07, 0x0C, 0x09, 0x0A, 0x0B, 0x08, 0x0D, 0x06, 0x0F, 0x04, 0x11, 0x02, 0x09, 0x28, 0x09, 0x02, 0x12, 0x02, 0x12,
  0x02, 0x12, 0x02, 0x12, 0x02, 0x12, 0x02, 0x09
};

uint8_t bmpNDiodeBig[] PROGMEM = {
  20, 128, // width run-length encoded
  24, 0, // height
  0x00, 0x09, 0x02, 0x12, 0x02, 0x12, 0x02, 0x12, 0x02, 0x12, 0x02, 0x12, 0x02, 0x09, 0x28, 0x09, 0x02, 0x11, 0x04, 0x0F,
  0x06, 0x0D, 0x08, 0x0B, 0x0A, 0x09, 0x0C, 0x07, 0x0E, 0x05, 0x10, 0x03, 0x12, 0x01, 0x14, 0x09, 0x02, 0x12, 0x02, 0x12,
  0x02, 0x12, 0x02, 0x12, 0x02, 0x12, 0x02, 0x09
};


const char* txp[6] = {"min: ", "incr:", "max: ", "mist ", "inst ", "mast "};
const char* txt[9] = {"I-base [uA]", "V-gate [0.1V] ", "V-gate [0.1V] ", "Bluet.", "CCS", "x-scale[V] : ", "y-scale[mA]: ", "Batt:", "I-diode [mA]"};
const char* txc[5] = {"pnp/npn", "MosFET", "J-FET ", "n-DEPL", "Diode"};
const char* txf[9] = {"CurveTracer", "Curve+Bluetooth", "Curve+SD-Card", "Curve+Bluet.+SD-C", "const.Voltage", "const.Current", "cC:Step-Up/Down", "cC:Step-Up", "cC:Step-Down"};
const char* txk[10] = {" ", " npn", " n-Mosfet", " n-jFet", " n-Diode", " pnp", " p-Mosfet", " p-jFet", " p-Diode", " n-Depl.Mos"};
const char* sdi[2] = {"SD-Card inserted ?", "SD-initial. done"};
const char* sdo[3] = {"sd000001.txt", "SD opened: ", "SD not opned: "};
/*
  const char* pmosf  = "p-MOSFET";
  const char* nmosf  = "n-MOSFET";
  const char* pjfet  = "p-JFET";
  const char* njfet  = "n-JFET";
  const char* pdiod  = "pDiode";
  const char* ndiod  = "nDiode";
*/

const int8_t lf = 10;   // linefeed
const uint16_t ft[4][10] =
  //---- 0---1---2---3---4---5---6---7---8---9---
{ 19, 37, 37, 19, 19, 19, 19, 19, 19, 37,   // h
  82, 123, 263, 289, 129, 129, 129, 280, 280, 162, // x
  7,  0,  0, 100, 98, 148, 198, 148, 198,  0, // y
  32, 38, 38, 20, 36, 36, 44, 35, 40, 98
};  // w 36/43 33/40  8:34
//---- 0---1---2---3---4---5---6---7---8---9---
const int8_t xs_t[] = {0, 1, 2, 3, 4, 6, 12};   // x_scale
const int8_t ys_t[] = {0, 2, 5, 10, 20, 50, 100}; // y_scale
//-----------------0-------1-------2-----------3--------4--------5-------6--------7----------8----
enum TkindDUT {tkNothing, tkNPN, tkNMOSFET, tkNJFET, tkNDIODE, tkPNP, tkPMOSFET, tkPJFET, tkPDIODE};
//-----------------0-------1-------2-----------3--------4--------5-------6--------7----------8----
int8_t /*TkindDUT*/ curkind;
//enum TclassDUT {tcBipolar, tcMOSFET, tcJFET};//0,1,2
int8_t tcBipolar = 0;
int8_t tcMOSFET  = 1;
int8_t tcJFET    = 2;

int8_t CurDUTclass = tcBipolar, class_o = tcBipolar;

uint8_t  idev, run_nr = 0, error_i = 0; //,nr_l=0;
uint16_t back_col, curve_col = TFT_BLACK, text_col = TFT_WHITE, col = 0, xo, yo, wo, ho, xa = 280, ya = 196, wa = 32, ha = 19;
bool retc;
char sign, devPol = 'N';
char buf[6];  // 12
char name_t[7], txtv[16], txcv[8];
char filen_t[13];
#ifdef SD_C
File   dataFile;
//String dataString = "";
#endif
#ifdef NAME_D
int8_t  l_devna = 0;     // devname
#endif
int8_t cxs = 0, cys = 0, l_ch = 4, nrk, dcl;
uint8_t x_scale = 12; // 1V, 2V, 3V, 4V, 6V, 12V              default: 12V
uint8_t y_scale = 50; // 2mA, 5mA, 10mA, 20mA, 50mA, 100mA    default: 50mA
int16_t ifc, ifi;
int base, igain = 0, iohm = 0, ibc = 0, ild, ith, x_old = 0, y_old = 0, i_CVS_A, i_CVS_B;
int DacVcc, incDacA, incDacB;
int incpix, maxpix, minpix, para, paro;
int val = 0, ila, imax = 480; //(600/48) * 50;
int incB = 0, minB = 0, maxB = 0, xlp, ylp;
int8_t s_ohm = 0, z_ohm, s_pos, iw, s_tt = 0, s_auto = 0, s_new = 0, s_old = 0;
volatile int enc1_State, enc1_State_o, newPos1, oldPos1 = -1999;
volatile int enc2_State, enc2_State_o, newPos2, oldPos2 = -1999;

volatile int8_t s_isr = 0, s_isr1, s_isr2, s_isr32 = 0, s_two1, s_two2, efnr, rota2 = 0, rota1 = 0, s_9 = 0;
volatile long time_l = 0, time1_o = 0, time2_o = 0;
unsigned long seconds = 1000L; //Notice the L
unsigned long minutes = seconds * 60;

//#ifdef TOUCH
volatile uint8_t s_touchi = 0;
volatile long valx=0,valy=0;
//#endif

int delay_CCS, delay_CCS_bas, delay_CCS_pow;
//delay(minutes); //for 60,000 milliseconds
int value;
/*
  int MinIbase =   0;  //   0µA
  int MaxIbase = 500;  // 100µA
  int IncIbase =  10;  // 10 µA
  int MinVgatm =   0;
  int MaxVgatm = 120;  // = 12V
  int IncVgatm =  10;  //  0.1V (10 µA x 10 k)
  int MinVgate =   0;
  int MaxVgate = 120;  // = 12V
  int IncVgate =  10;  //  0.1V (10 µA x 10 k)
*/
int mini = 0, maxi = 0, inci = 0, minst = 1, incst = 1, maxst = 10;

byte    i = 0, s_blue = 9, l_func=0, funcx, funcxo=9, funcy, funcyo=9, s_sdcard = 0, s_first = 1, s_depl = 0, s_diode = 7, s_dsign = 0;
byte    s_milli, s_stair = 0, s_idss = 0, s_thresh = 0, s_pinch = 0, s_stop = 0, s_comp = 0, s_compl = 0;
int16_t s_delay1=0,s_delay2=0,s_zen=0;
int8_t s_touch = 0,s_osci = 0,s_curve = 0,s_chan1=0,s_chan2=0,s_offs1=0,s_offs2=0;
float fce, fi, fib, fid, fidio, fidss, fir, fpinch, fuc, fui = 12.0, fur, fvgate, fvthr, ubat = 0.0,ua2=0.0;
float fimax, finc;    // 0.319/x_scale //=  1023.33 * (float) x_scale; //1023 * 12 = 12280 mV
float f_gain=1.0,f_gain1=1.0,f_gain2=1.0;
uint8_t   yct[320];

#define RGB(r, g, b)  (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))
#define TFT_PAPAYA      RGB(255, 239, 213)
#define TFT_PEACH       RGB(255, 218, 185)
#define TFT_LIGHTGREY   RGB(192, 192, 192)
#define TFT_LIGHTBLUE   RGB( 40,  80, 255)
#define TFT_BRIGHTBLUE  RGB(128, 255, 255)
#define TFT_LIGHTRED    RGB(255, 100, 150)
#define TFT_UDARKGREY   RGB( 64,  64,  64)
#define TFT_NEARBLACK   RGB( 45,  45,  45)
#define TFT_BLACKALMOST RGB( 25,  25,  25)
#define TFT_WHITEALMOST RGB(208, 208, 208)
#define TFT_LIGHTTUERK  RGB(127, 255,  80)


void DrawPixel(uint16_t x, uint16_t y, uint16_t color)  {
  tft.drawPixel(x, y, color);
}

void DrawString(const char *s, uint8_t tsi, uint16_t color) {  //

  tft.setTextColor(color);
  tft.setTextSize(tsi);
  tft.print(s);
}

void DrawStringAt(uint16_t x, uint16_t y, const char *s, uint8_t tsi, uint16_t color) {  //
  tft.setTextColor(color);
  tft.setTextSize(tsi);
  tft.setCursor(x, y);
  tft.print(s);
}

void DrawCharAt(uint16_t x, uint16_t y, char c, uint8_t tsi, uint16_t color) { //
  tft.setTextColor(color);
  tft.setTextSize(tsi);
  tft.setCursor(x, y);
  tft.print(c);
}

void DrawChar(char c, uint8_t tsi, uint16_t color) { //
  tft.setTextColor(color);
  tft.setTextSize(tsi);
  tft.print(c);
}

void DrawInt(int i, uint8_t tsi, uint16_t color) { //
  tft.setTextColor(color);
  tft.setTextSize(tsi);
  tft.print(i);
}

void DrawIntAt(uint16_t x, uint16_t y, int i, uint8_t tsi, uint16_t color) { //
  tft.setTextColor(color);
  tft.setTextSize(tsi);
  tft.setCursor(x, y);
  tft.print(i);
}

void DrawFloat(float floatNumber, int dp, uint8_t tsi, uint16_t color) {  //
  tft.setTextColor(color);
  tft.setTextSize(tsi);
  tft.print(floatNumber, dp);
}

void DrawFloatAt(uint16_t x, uint16_t y, float floatNumber, int dp, uint8_t tsi, uint16_t color) {  //
  tft.setTextColor(color);
  tft.setTextSize(tsi);
  tft.setCursor(x, y);
  tft.print(floatNumber, dp);
}

void DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
  tft.drawLine(x1, y1, x2, y2, color);
}

void DrawHLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color) {
  tft.drawFastHLine(x , y, h, color);
}

void DrawVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color) {
  tft.drawFastVLine(x , y, h, color);
}

void ILI9341SetCursor(uint16_t x, uint16_t y) {
  tft.setCursor(x, y);
}

void DrawBox(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
  tft.fillRect( x, y, w, h, color);
}

void DrawFrame(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
  tft.drawRect( x, y, w, h, color);
}


/*

  void EEPROM.writeCharEEPROM(int address, byte val, int i2c_address)   // 24LC Function to EEPROM.writeChar to EEPROOM
  {

  Wire.beginTransmission(i2c_address);  // Begin transmission to I2C EEPROM

  Wire.write((int)(address >> 8));   // MSB  // Send memory address as two 8-bit bytes
  Wire.write((int)(address & 0xFF)); // LSB  // Send memory address as two 8-bit bytes

  Wire.write(val);                      // Send data to be stored

  Wire.endTransmission();               // End the transmission

  delay(5);                             // Add 5ms delay for EEPROM
  }

*/

/*

  byte readEEPROM(int address, int i2c_address) //24LC Function to read from EEPROM
  {
  byte rcvData = 0xFF;                   // Define byte for received data
  Wire.beginTransmission(i2c_address);   // Begin transmission to I2C EEPROM

  Wire.write((int)(address >> 8));   // MSB  // Send memory address as two 8-bit bytes
  Wire.write((int)(address & 0xFF)); // LSB  // Send memory address as two 8-bit bytes

  Wire.endTransmission();                // End the transmission

  Wire.requestFrom(i2c_address, 1);      // Request one byte of data at current memory address

  rcvData =  Wire.read();                // Read the data and assign to variable
   return rcvData;                        // Return the data as function output

  }

*/



void IRAM_ATTR isr_but32() {  // push-button pressed
  s_isr32 = 1;
  s_isr1 = 0;
  s_isr2 = 0;
  // long time_a = millis();
  // if (time_a > time_l + 5) {  //3  // time_e
  //   s_isr = 1;
  //   time_l = time_a;
  // }

}


void IRAM_ATTR isr_ENC1() {
  if ((millis() - time1_o) < 50)  // debounce time is 50ms
    return;
  s_isr1 = 1;
 
  if (digitalRead(pin_ENC1_B) == HIGH) {
    // the encoder is rotating in counter-clockwise direction => decrease the counter
    newPos1--;
    rota1 = -1;
  //  direction = DIR_CCW;
  } else {
    // the encoder is rotating in clockwise direction => increase the counter
    newPos1++;
    rota1 = 1;
 //   direction = DIR_CW;
  }

  time1_o = millis();
}


void IRAM_ATTR isr_ENC2() {
  if ((millis() - time2_o) < 50)  // debounce time is 50ms
    return;
//  s_isr1 = 0;
  s_isr2 = 1;
  if (digitalRead(pin_ENC2_B) == HIGH) {
    // the encoder is rotating in counter-clockwise direction => decrease the counter
    newPos2--;
    rota2 = -1;
  //  dir2 = DIR_CCW;
  } else {
    // the encoder is rotating in clockwise direction => increase the counter
    newPos2++;
    rota2 = 1;
 //   dir2 = DIR_CW;
  }

  time2_o = millis();
}





//-------------------------------------------------------------------------
// DrawKindStr
//   draws the kind of the DUT at the top of the screen
//-------------------------------------------------------------------------

void DrawKindStr(int8_t /*TkindDUT*/ kind) {
  // debug("DrawKindstr: ");debugln(kind);
  int ix  = 146; //(TFT_WID - 28)/2; // Bitmap
  int ixb = 140; //(TFT_WID - 39)/2; // bipolar
  int ixm = 120; //(TFT_WID - 80)/2; // Mosfet
  int ixj = 130; //(TFT_WID - 59)/2; // J-Fet
  tft.setFreeFont(NULL); // NULL: use (old/)5tandard font GLCD.h  //??
  if (s_thresh > 1 or s_idss > 1 or s_pinch > 1) DrawBox(201, 4, 75, 42, TFT_BLACK); // for idss,vpinch,...
  if (idev == 84) { // 84==T// kind == tkNPN or kind == tkPNP) {  // bipolar
    if (s_diode == 1) {
      DrawBox(139, 3, 59, 18, TFT_BLACK);
      DrawBox(142, 22, 26, 34, TFT_BLACK);
      tft.setCursor(ixb, 3);  // 15
      if (kind == tkPNP) {
        DrawString(txk[8]/*pdiod*/, 2, TFT_YELLOW);  // size
#ifdef BMP
        DrawBitmapMono(ix, 25, bmpPDiodeBig, TFT_YELLOW);  
#endif
      }
      else {
        DrawString(txk[4]/*ndiod*/, 2, TFT_YELLOW);  // size
#ifdef BMP
        DrawBitmapMono(ix, 25, bmpNDiodeBig, TFT_YELLOW);  
#endif
      }
      return;
    } // diode
    // pnp/npn-transistor
    tft.setCursor(ixb, 3);
    if (kind == tkPNP) {
      DrawString("PNP", 2, TFT_YELLOW);
#ifdef BMP
      DrawBitmapMono(ix, 25, bmpPNP, TFT_YELLOW);
#endif
    }
    else {
      DrawString("NPN", 2, TFT_YELLOW);
#ifdef BMP
      DrawBitmapMono(ix, 25, bmpNPN, TFT_YELLOW);
#endif
    }

    return;
  }
  tft.setCursor(ixm, 3);
  if (kind == tkPMOSFET ) {
    DrawString(txk[6]/*pmosf*/, 2, TFT_YELLOW);
#ifdef BMP
    DrawBitmapMono(ix, 25, bmpPMOSFET, TFT_YELLOW);
#endif
    return;
  }
  if (kind == tkNMOSFET) {
    DrawString(txk[2]/*nmosf*/, 2, TFT_YELLOW);
#ifdef BMP
    DrawBitmapMono(ix, 25, bmpNMOSFET, TFT_YELLOW);
#endif
    return;
  }
  tft.setCursor(ixj, 3);
  if (kind == tkPJFET) {
    DrawString(txk[7]/*pjfet*/, 2, TFT_YELLOW);
#ifdef BMP
    DrawBitmapMono(ix, 25, bmpPJFET, TFT_YELLOW);
#endif
    return;
  }
  if (kind == tkNJFET) {
    if (s_depl == 0) {  // normal j-Fet
      DrawString(txk[3]/*njfet*/, 2, TFT_YELLOW);
#ifdef BMP
      DrawBitmapMono(ix, 25, bmpNJFET, TFT_YELLOW);
#endif
      return;
    }
    else {              // depletion-mode MOS-FET
      //   txcv = txc[3];
      DrawString(txc[3], 2, TFT_YELLOW);
#ifdef BMP
      DrawBitmapMono(ix, 25, bmpNMOSFET, TFT_YELLOW);
#endif
      DrawVLine(157, 34, 17, TFT_YELLOW);  // 20 ??
      DrawVLine(158, 34, 17, TFT_YELLOW);  // 20 ??
      return;
    }
  }
  /*
    case tkPDIODE:
    case tkNDIODE:
    DrawStringAt((TFT_WID - 47) / 2, 15, "Diode", 2, TFT_YELLOW);
    #ifdef BMP
    DrawBitmapMono((TFT_WID - 20) / 2, 25, bmpPDiodeBig, TFT_YELLOW);
    #endif
    break;
    default: // unknown
  */
  // DrawStringAt(151/*(TFT_WID - 117) / 2*/, 15, "Curve Tracer", 2, TFT_YELLOW);

}


/***************************************************************************************
** Function name:           DrawBitmapMono
** Description:             Draw a black and white image stored in an array on the TFT
***************************************************************************************/
void DrawBitmapMono(int16_t x, int16_t y, uint8_t *bitmap, uint16_t color) {
  uint8_t *bmp;
  int16_t w;

  bmp = bitmap;
  w = pgm_read_byte(bmp++);
  w = pgm_read_byte(bmp++);

  if ((w & 0x80) > 0)
    DrawBitmapMonoRLE(x, y, bitmap, color);
  else
    DrawBitmapMonoBits(x, y, bitmap, color);
}

/***************************************************************************************
** Function name:           DrawBitmapMonoBits
** Description:             Draw a black and white image stored as an array of bits
***************************************************************************************/
void DrawBitmapMonoBits(int16_t x, int16_t y, const uint8_t *bitmap, uint16_t color) {
  int16_t i, j, bits, n, w, h;
  w = pgm_read_byte(bitmap++);
  w = w | (pgm_read_byte(bitmap++) << 8);
  h = pgm_read_byte(bitmap++);
  h = h | (pgm_read_byte(bitmap++) << 8);

  //tft_fastSetup();
  n = 0;
  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++ ) {
      if (n % 8 == 0)
        bits = pgm_read_byte(bitmap++);
      if ((bits & 0x80) == 0) DrawPixel(x + i, y + j, color);
      //  if ((bits & 0x80) == 0) tft_fastPixel(x + i, y + j, color);

      bits = bits << 1;
      n++;
    }
  }
}



void DrawBitmapMonoRLE(int16_t x, int16_t y, const uint8_t *bitmap, uint16_t color) {
  int16_t i, j, nb, w, h;
  bool CurIsBlack;

  w = pgm_read_byte(bitmap++);
  w = w | (pgm_read_byte(bitmap++) << 8);
  w = w & 0x7FFF;
  h = pgm_read_byte(bitmap++);
  h = h | (pgm_read_byte(bitmap++) << 8);

  // tft_fastSetup();

  nb = 0;
  CurIsBlack = true;
  j = 0;
  i = 0;

  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++ ) {
      while (nb == 0) {
        nb = pgm_read_byte(bitmap++);
        CurIsBlack = !CurIsBlack;
      }
      if (!CurIsBlack) DrawPixel(x + i, y + j, color);

      //  if (!CurIsBlack) tft_fastPixel(x + i, y + j, color);
      nb--;
    }
  }
}


//-------------------------------------------------------------------------
// TurnOffLoad
//   sets load current to zero
//-------------------------------------------------------------------------

void TurnOffLoad() {

  digitalWrite(pin_E_NEG, LOW); // disconnect Emitter/Source/Cathode from ground
  //delay(4);
  digitalWrite(pin_E_POS, LOW); // disconnect Emitter/Source/Anode   from V+
  // delay(4);
  // digitalWrite(pin_BE, LOW);    // disconnect current-source
  // delay(4);
  digitalWrite(pin_EB, LOW);      // switch current-source to p-device
  // delay(4);
  // digitalWrite(pin_POW_CCS , LOW); // used for ccs-i2c, use pin_E_NEG /pin_E_POS : truncate power from CVS(TCA0372)-board
  //delay(10);
  digitalWrite(pin_CV_SW, LOW);    // switch to constant-current source PinA2 bipolar
  delayMicroseconds(15);
 // Serial.println("vor settingsD");
  MCP_D.setSPIspeed(12000000); // SPI.beginTransaction(settingsD);  // DAC: 12MHz
 // Serial.println("nach settingsD");
  MCP_D.fastWriteA(0);             // MCP4822
  MCP_D.fastWriteB(0);             // MCP4822
  delay(20);
 // Serial.println("vor i2c.setvolt");
  dac_i2c.setVoltage(0, false);  // MCP4725
  delayMicroseconds(20);         //delay(1); //  10;
 // Serial.println("vor settingsT");
 // SPI.beginTransaction(settingsT);  // TFT: 20MHz
 // Serial.println("nach settingsT");
}

/*

// Send value to DAC channel (0 or 1) MCP4822

void setValue_DAC(byte channel, uint16_t val) {
  uint16_t data = val;
  //data <<= SHIFT_BITS_CNT; //0 for MCP4822
  data |= (((uint16_t)channel << CHANNEL_BIT_POS)
           | ((uint16_t)GAIN_1X << GAIN_BIT_POS)
           | ((uint16_t)MCP48X2_CH_ACTIVE << SHDN_BIT_POS));

  digitalWrite(pin_DAC_CS, LOW);  // Select chip (active low)
  SPI.beginTransaction(SPISettings(ADC_CLK, MSBFIRST, SPI_MODE0));
  SPI.transfer16(data);
  SPI.endTransaction();
  digitalWrite(pin_DAC_CS, HIGH);  // deselect chip (active low)
}
*/

//-------------------------------------------------------------------------
// InitGraph
//   draws the grid background of the graph
//-------------------------------------------------------------------------

void InitGraph(int8_t /*TkindDUT*/ kind) {
  long ix, x = 0, iy, y, iw;
  float fx = 0;
  uint8_t s_min = 0;

  if (devPol == 'P') s_min = 1; //neg.scale //if (kind >== tkPNP || kind == tkPMOSFET || kind == tkPJFET || kind == tkPDIODE) s_min = 1; // neg.scale

  tft.fillScreen(TFT_BLACK);// ClearDisplay(TFT_BLACK);
  tft.setFreeFont(NULL); // NULL: use (old/)5tandard font GLCD.h
  tft.setTextSize(1);
  DrawBox(286, 80, 34, 80, TFT_BLACK); // Blackalmost
  DrawLine(0, 0, 0, TFT_HGT, TFT_UDARKGREY);
  DrawLine(0, TFT_HGT - 1, TFT_WID, TFT_HGT - 1, TFT_UDARKGREY);

  uint16_t disx = 312 / 12; // 26  vertical grid
  uint16_t max_scale = Vmax;  // 12

  for (ix = 1; ix <= max_scale; ix++) {  // vertical lines   of the grid
    x += disx;   // #################// 26,13,3
    DrawLine(x, 0, x, TFT_HGT - 10, TFT_UDARKGREY);
  }

  for (iy = 1; iy <= 9; iy++) {          // horizontal lines of the grid
    uint8_t y_h = 240 - iy * 24;

    DrawLine(12 , y_h, TFT_WID, y_h, TFT_UDARKGREY);
  }
  if (s_osci == 1) {
   // s_osci = 0;
    return; 
  }
  
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setCursor(2, TFT_HGT - 11);
  if (s_zen > 0) tft.print(s_zen);  //s_zen
  else tft.print('0'); //DrawCharAt(2, TFT_HGT - 4, '0', 1, TFT_LIGHTGREY);  //string


  uint16_t scq = max_scale / x_scale; // 1,2,
  x = 0;
  tft.setTextColor(TFT_LIGHTGREY);
  for (ix = 1; ix <= max_scale; ix++) {  // x-scale
    x += disx;   // #################// 26,13,3
    //  DrawLine(x, 0, x, TFT_HGT - 10, TFT_UDARKGREY);
    if (s_zen > 0) fx = (float) s_zen + (float)(ix) * 0.2;  //??????
    else           fx = (float) ix / (float)scq;  //??????
    tft.setCursor(x - 4, TFT_HGT - 11); //HGT - 3
    if (ix < max_scale) tft.print(fx, 1); //DrawFloat(fx, 1, 1, TFT_LIGHTGREY);
  }
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setCursor(301, TFT_HGT - 11);  // 3
  if (s_min == 1) tft.print('-');  //DrawCharAt(300, TFT_HGT - 3, '-', 1, TFT_LIGHTGREY); //string
  tft.print('V');  //DrawCharAt(310, TFT_HGT - 3, 'V', 1, TFT_LIGHTGREY); //string

  //  y_scale = 100;//mAmax; // 50mA ,100mA 10 mA
  uint8_t y_div = 10;
  if (y_scale < 10) y_div = y_scale;
  for (iy = 0; iy < y_scale; iy += y_scale / y_div) { //0,10,20,30,40,50  0,20,40,60,80
    y = TFT_HGT - 1 - iy * TFT_HGT / y_scale;
    tft.setCursor(2, y + 3);
    if (iy > 0) tft.print(iy); //DrawIntAt(2, y + 3, iy, 1, TFT_LIGHTGREY);
    //  DrawLine(12 , y, TFT_WID, y, TFT_UDARKGREY);
  }

  tft.setTextColor(TFT_LIGHTGREY);
  tft.setCursor(2, 8);
  if ( s_min == 1)  tft.print('-'); //DrawCharAt(2, 8, '-', 1, TFT_LIGHTGREY); //string
  tft.print("mA"); //DrawString("mA", 1, TFT_LIGHTGREY);
  DrawKindStr(kind);

  tft.setFreeFont(FF33); // tft.setFreeFont(&FreeSerif9pt7b);
  // Serial.println("initGraph end");
}



void EndScan(int8_t /*TkindDUT*/ kind) {

  DrawKindStr(kind);
  tft.setFreeFont(FF33); // tft.setFreeFont(&FreeSerif9pt7b);  // font??
  if (idev == 77) {   // 77=M //kind == tkPMOSFET  or kind == tkNMOSFET) {
    // DrawKindStr(kind);
    if (s_thresh == 2) {
      tft.setCursor(202,38);
      DrawString(" Vth=", 1, TFT_CYAN);
      DrawFloat(fvthr, 1, 1, TFT_CYAN);
      // fvthr = 0.0;
      s_thresh = 0;
    }
    return;
  }
  if (idev == 70) { // 70==F // kind == tkPJFET  or kind == tkNJFET) {
    if (s_idss == 2) {
      tft.setCursor(202,38);
      DrawString(" Idss=", 1, TFT_CYAN);  // 2
      DrawFloat(fidss, 1, 1, TFT_CYAN);   // 2
      //     fidss = 0.0;
      s_idss = 0;
    }
    if (s_pinch == 2) {
      tft.setCursor(202, 14);
      DrawString(" Vp=", 1, TFT_CYAN);
      if (kind == tkNJFET) DrawChar('-', 1, TFT_CYAN);//string
      DrawFloat(fpinch, 1, 1, TFT_CYAN);
      // fpinch = 0.0;
      s_pinch = 0;
    }
  }

}

void changePolar(int8_t /*TkindDUT*/ kind)  {   //
  s_compl = 1;
  int8_t iki = /*(int)*/ kind;
  // debug("switch from old-p/n-kind");debug(kind);
  if (iki <= 4)  iki = iki + 4; // from 'n' to 'p': 5,6,7,8
  else           iki = iki - 4; // from 'p' to 'n': 1,2,3,4
  kind = /*(TkindDUT)*/ iki;
  curkind = kind;
  devPol = 'N';
  if (kind >= 5) devPol = 'P';
  // debug("to new-n/p-kind");debugln(kind);
}


void nextDevice(int8_t /*TkindDUT*/ kind)  {   //    stopCurves or compare/complemenary-mode
  int8_t ic;
  char* txm[4] = {"Stop ?", "Param ?", "Compare ?", "Complem. ?"};
#ifdef SD_C
  if (s_sdcard == 1) {
    dataFile.print(0x04);  // EOF 0x1A ?
    dataFile.close();
  }
#endif
  tft.setFreeFont(FF33); // tft.setFreeFont(&FreeSerif9pt7b);  // font??
 
  s_isr2 = 0;
  s_isr32 = 0;
  while (1)      {
    for (ic = 0; ic <= 3; ic++) {
      DrawBox(27, 53, 92, 18, TFT_BLACK);//DARKGREY);
   //   tft.setCursor(27, 65); tft.setTextColor(TFT_WHITE);tft.print(txm[ic]);
      DrawStringAt(27, 65,txm[ic], 1, TFT_WHITE);
      delay(1200);
      if (s_isr2 == 1 or  s_isr32 == 1 or s_touchi == 1) {
        s_isr2  = 0;
        s_isr32 = 0;
        s_touchi = 0;
        if      (ic == 0) s_stop  = 1;
        else if (ic == 1) s_stop  = 2;
        else if (ic == 2) s_comp  = 1;
        else if (ic == 3) changePolar(kind);   // s_compl = 1
        return;
      } // if
    }   // for
  }     // while
}



void setDac_A_B() {

  int idelay = 1, ifco = 0, isa, isb, isba=0,isi = 1, ianz = 1;
  float fis = 0;

  ifc  = 0;
  ifco = 0;
  ifi  = 0;
  // Serial.println("setDac_A_B");
  MCP_D.setSPIspeed(12000000); // SPI.beginTransaction(settingsD);  // DAC: 2 wegen ADC, max: 12MHz
  //dac.setGain1X_B();    //xx 0...2048mV,       0.5 mV/step  ###
  //dac.setGain1X_A();    //xx 0...2048mV,       0.5 mV/step  ###
  MCP_D.fastWriteA(0);
  MCP_D.fastWriteB(0);
  delay(10);   // 50

  if (s_blue == 4) {                 // constVoltage 2 sd_c
    MCP_D.fastWriteA(i_CVS_A);  // 4000 = 80mV
    delay(30);
    if (i_CVS_B > 0) {
      MCP_D.fastWriteB(i_CVS_B);      //## 2000 mV steps 40mV
      delay(60);
    }
    delay(10000);
    return;
  }

  if (devPol == 'P') idelay = 10;
  //intToBLE(tsd+65);                                // Serial.write('A'); // analogwerte =ta ##############################
 
  isba = 0;
  if (s_zen > 0) isba = s_zen * 200;
    
  for (isb = isba; isb < 4000; isb += 400) { // 2V steps
    //
    MCP_D.fastWriteB(isb);            //## 2000 mV steps
   
 // Serial.println(isb);        
    if (s_blue <= 3) delay(20);      // 60 4 100 1 sd_c
    /*  MCP_D.fastWriteB(isb);            //## 2000 mV steps
      if (s_blue <= 3) delay(10);    // 4 100 1 sd_c
      MCP_D.fastWriteB(isb);            //## 2000 mV steps
      if (s_blue <= 3) delay(10);    // 4 100 1 sd_c
      MCP_D.fastWriteB(isb);            //## 2000 mV steps
      if (s_blue <= 3) delay(10);    // 4 100 1 sd_c
    */
    else delay(4);
    
    MCP_D.fastWriteA(0);
    delay(10);
    if (isb > 20) readADC_01();

    for (isa = 0; isa < 4000; isa += isi) {
      MCP_D.fastWriteA(isa);          //#### 0.5mV steps
   // delayMicroseconds(10);          // 0 30 
      /*  if (isa < 200) {
          delayMicroseconds(50);
          MCP_D.fastWriteA(isa);
        }
      */
      //   if (isa < 500) delayMicroseconds(5);
      //
      //  else delayMicroseconds(6);
      readADC_01();                 // ####################################################################
      if (error_i == 2)  return;    // max-collector-current

      // debug(isb);debug(isa);debug(ifc);debug(ifi);debugln(error_i);
      if (ifc <= ifco) {            // accumulate currents
        fis = fis + fi;
        ianz++;
      }
      else {                        // average current
        fis = fis / (float)ianz;
        ifi = (int) fis;
        yct[ifco] = ifi;
        ifco = ifc;
        fis = fi;
        ianz = 1;
      }
      if (ifc > 319)     return;
      if (error_i == 1)  return;   // max-diode-current
    }
    if (s_diode == 0) isi++;       // stepwidth: 1,2,3,4,5;
  }

}


void drawMyP( int16_t x, int16_t y, uint16_t col) {

  const uint8_t txy[56] = { 0, 0, 0, 0, 0, 0, 0,
                            0, 1, 0, 0, 0, 1, 0,
                            0, 1, 0, 0, 0, 1, 0,
                            0, 1, 0, 0, 0, 1, 0,
                            0, 1, 0, 0, 0, 1, 0,
                            0, 1, 1, 1, 1, 1, 1,
                            0, 1, 0, 0, 0, 0, 0,
                            0, 1, 0, 0, 0, 0, 0
                          };
  int16_t ux = 0, uy = 0, ub = 0, lm = 0;
  for (int16_t lt = 0; lt < 56; lt++) {
    if (txy[lt] == 1) tft.drawPixel(x + ux, y + uy, col);
    lm++;
    ux = lm;

    if (lm == 7) {  // new Line
      uy = uy + 1;
      ux = 0;
      lm = 0;
    }

  }
}
void drawMyL( int16_t x, int16_t y, uint16_t col) {

  const uint8_t txy[18] = { 0, 0,
                            0, 0,
                            0, 0,
                            0, 0,
                            1, 1,
                            1, 1,
                            1, 1,
                            1, 1,
                            1, 1
                          };
  int16_t ux = 0, uy = 0, ub = 0, lm = 0;
  for (int16_t lt = 0; lt < 18; lt++) {
    if (txy[lt] == 1) tft.drawPixel(x + ux, y + uy, col);
    lm++;
    ux = lm;

    if (lm == 2) {  // new Line
      uy = uy + 1;
      ux = 0;
      lm = 0;
    }

  }
}

void drawOmega( int16_t x, int16_t y, uint16_t col) {

  int16_t ux = 0, uy = 0, ub = 0, lm = 0;
  uint8_t txy[56] = { 0, 0, 1, 1, 1, 0, 0,
                      0, 1, 0, 0, 0, 1, 0,
                      1, 0, 0, 0, 0, 0, 1,
                      1, 0, 0, 0, 0, 0, 1,
                      0, 1, 0, 0, 0, 1, 0,
                      0, 0, 1, 0, 1, 0, 0,
                      0, 0, 1, 0, 1, 0, 0,
                      0, 1, 1, 0, 1, 1, 0
                    };
  for (int16_t lt = 0; lt < 56; lt++) {
    if (txy[lt] == 1) tft.drawPixel(x + ux, y + uy, col);
    lm++;
    ux = lm;

    if (lm == 7) {  // new Line
      uy = uy + 1;
      ux = 0;
      lm = 0;
    }

  }
}


void drawBeta( int16_t x, int16_t y, uint16_t col) {

  int16_t ux = 0, uy = 0, ub = 0, lm = 0;
  uint8_t txy[70] = { 0, 0, 0, 1, 1, 0, 0,
                      0, 0, 1, 0, 0, 1, 0,
                      0, 1, 0, 0, 0, 1, 0,
                      0, 1, 0, 0, 1, 0, 0,
                      0, 1, 0, 0, 0, 1, 0,
                      0, 1, 0, 0, 0, 0, 1,
                      0, 1, 0, 0, 0, 0, 1,
                      0, 1, 0, 0, 0, 1, 0,
                      0, 1, 0, 1, 1, 0, 0,
                      0, 1, 0, 0, 0, 0, 0
                    };
  for (int16_t lt = 0; lt < 70; lt++) {
    if (txy[lt] == 1) tft.drawPixel(x + ux, y + uy, col);
    lm++;
    ux = lm;

    if (lm == 7) {  // new Line
      uy = uy + 1;
      ux = 0;
      lm = 0;
    }

  }
}
/*

  void drawOmega0( uint16_t x,uint16_t y, uint16_t col) {
      y = y - 7;
      DrawCircle(    x,     y, 3, col);
      DrawHLine( x , y + 3, 1, TFT_BLACK);
      DrawVLine( x - 1, y + 4, 1, col);//3
      DrawVLine( x + 1, y + 4, 1, col);//3
      DrawHLine( x - 2, y + 5, 2, col);//-3,7
      DrawHLine( x + 1, y + 5, 2, col);//+2,7

  }
*/

void traceEnd(int ib, int8_t /*TkindDUT*/ kind)  {

  uint16_t xlg = ifc, ylg = 239 - ifi; // from readADC_01()
  tft.setTextSize(1);  //#1#
  tft.setFreeFont(NULL); // NULL: use (old/)5tandard font GLCD.h
  if (error_i == 2) {  // collector-current
    tft.setCursor(30, 40);
    tft.setTextColor(TFT_RED);
    tft.print("### i-collector >  mA"); //DrawStringAt(30, 40, "### i-collector >  mA", 1, TFT_RED);
    tft.print(imax);      //"DrawInt(imax, 2, TFT_RED);
    tft.print(" mA !!");  //imax);DrawString(" mA !!", 2, TFT_RED);
#ifdef BLUE_T
    if (s_blue == 1 or s_blue == 3) {
      intToBLE(tsd + 69); //charToBLE('E');  // Serial.write('E'); Serial.write("\n"); // error imax
      intToBLE(imax);  // i > imax
    }
#endif
    delay(4000);

    return;
  }  // end: error collector-current
  if (xlg < 0 or xlg > 290) xlg = 290;
  if (ylg > 234) {
    ylg = 234;
    DrawBox(289, 222, 23, 8, TFT_BLACK); // Blackalmost
    DrawBox(313, 222, 7, 8, TFT_BLACK); // Blackalmost
  }
#ifdef BLUE_T
  if (s_blue == 1 or s_blue == 2 or s_blue == 3) {
    if (idev == 84) intToBLE(tsd + 66);    // charToBLE('B'); //Serial.write('B'); Serial.write("\n")  // base-c T
    else            intToBLE(tsd + 71);    // charToBLE('G');  // Mosfet/jFet
    intToBLE(ib);                           // base-current/vgate
  }
  if (iohm > 0) {
    intToBLE(tsd + 79); // charToBLE('O');  // Ohm at 0.5V
    intToBLE(iohm);                         // resistance of j-Fet
  }
  // intToBLE(tsd+90); // charToBLE('Z'); //Serial.write('Z'); Serial.write("\n");  // end curve
#endif
  tft.setCursor(xlg, ylg - 5);

  if (idev == 84) {      // if (kind == tkNPN or kind == tkPNP) {
    // debug(xlg);debug(ylg);debug(fui);debug(ib);debugln(igain);
    if (s_diode == 0) {  // npn/pnp-Transistor
      tft.setTextColor(curve_col);
      tft.print(ib); //DrawInt(ib, 1, curve_col);       // base-current
      //  tft.print(' ');
      DrawString(" uA", 1, curve_col);  // uA ?? µA ~~~~ ??? ###################################
      tft.drawFastVLine(xlg + 18, ylg + 1, 3, curve_col); // 24 // u --- > µ ??? ####################
      // drawMyL( xlg+35,ylg-8, curve_col); // µ???
      fid   = (fir - fib) / fib; // ??? isolated const. current
      fid   = fir / fib;      // test ??? isolated const. current
      igain = (int)fid;
      if (igain > 0) {
        // xlg = xlg+100; // test##################################
        tft.setCursor(xlg - 30, ylg - 4);
        drawBeta(xlg - 39, ylg - 5, curve_col); // beta ??
        //tft.print("$ ");   // DrawStringAt(xlg - 30, ylg - 4, "$ ", 1, curve_col); // beta on position $ ##############################
        tft.print(igain);  // DrawInt(igain, 1, curve_col); // hfe,beta
      }

      // debug(ib);debugln(igain);

#ifdef BLUE_T
      if (s_blue == 1  or s_blue == 2 or s_blue == 3) {

        intToBLE(tsd + 72); // charToBLE('H');  // Serial.write('H'); Serial.write("\n"); // hfe / beta
        intToBLE(igain);     // gain bipolar

        // intToBLE(tsd+90);   // charToBLE('Z'); //Serial.write('Z'); Serial.write("\n");  // end curve
      }
#endif
      igain = 0;
      return;
    }
    else {  // diode
      int iint = float(fidio);
      if (xlg > 290) xlg = 280;
      tft.setCursor(xlg + 5, ylg + 12);
     /* tft.print(iint);*/ DrawIntAt(xlg + 5, ylg + 2, iint, 1, curve_col);
     /* tft.print(" mA");*/ DrawString(" mA", 1, curve_col);
      //   tft.setCursor(xlg+5,ylg+13); DrawFloat(fuc/1000.0, 1, 1, curve_col);  // U-Diode V
      DrawFloatAt(xlg + 5, ylg + 13, (fuc + s_zen * 1000.0) / 1000.0, 1, 1, curve_col); // U-Diode V
      tft.setCursor(xlg + 5, ylg + 13);
      tft.print(" V");   // DrawString(" V", 1, curve_col);
      MCP_D.fastWriteA(0);
      MCP_D.fastWriteB(0);
      delay(20);
#ifdef BLUE_T
      if (s_blue == 1  or s_blue == 3) {
        intToBLE(tsd + 68);  //charToBLE('D');   //Serial.write('D'); Serial.write("\n"); // diode(current)
        intToBLE(iint);       // diode-current
        intToBLE(tsd + 85);  //charToBLE('U');   //Serial.write('U'); Serial.write("\n"); // u-diode
        int idua = (int)fuc;  //mV
        intToBLE(idua);       // diode-voltage mV

        //      intToBLE(tsd+90);    //charToBLE('Z'); //Serial.write('Z'); Serial.write("\n");  // end curve
      }
#endif
    }
    tft.setFreeFont(FF33);//  tft.setFreeFont(&FreeSerif9pt7b);  // font??
   // s_zen = 0;  //?
    return;
  }  //end:  npn or pnp
  //   else if (kind == tkNMOSFET or kind == tkNJFET
  //         or kind == tkPMOSFET or kind == tkPJFET) {
  tft.print(fvgate / 1000.0, 2); //DrawFloat(fvgate / 1000.0, 2, 1, curve_col);
  tft.print('V');              //DrawString(" V", 1, curve_col);


  if (iohm > 0)   {
    // if (iohm < 1200) {
    tft.setCursor(xlg - 44, ylg - 6);
    tft.print(iohm); //DrawIntAt(xlg - 35, ylg - 6, iohm, 1, curve_col);
    drawOmega(xlg - 23, ylg - 7, curve_col); // ???? not tested
    //tft.print(" ^"); //DrawString(" ^", 1, curve_col); //drawOmega0(xlg-10,ylg-3,curve_col);  //#### ^O=Omega
    //  }
    iohm = 0;
  }
  if (idev == 70)      {  // 70='F' kind == tkNJFET or kind == tkPJFET) {
    if (fib == 0.0) {
      fidss = fir;
      s_idss = 2;
    }
    if (s_pinch == 1) {
      if (fir < 0.25) {
        fpinch  = 0.0025 * (int)base; //mA  1 µA = 4 steps DAC
        s_pinch = 2;
      }
    }

    /*     if  (x_scale == 1  and y_scale == 5 or y_scale == 10)   {   // 1 and 10
           uint16_t rg = 50;
           if (y_scale == 5) rg+=rg;
           drawResistance( 95, 50,  rg,155,  0,TFT_CYAN);   //x,y,value,x-line_end,y-line_end,color
           drawResistance(212, 50,2*rg,316,  0,TFT_YELLOW); //x,y,value,x-line_end,y-line_end,color
           drawResistance(265,120,4*rg,316,119,TFT_RED);    //x,y,value,x-line_end,y-line_end,color
           drawResistance(265,175,8*rg,316,179,TFT_GREEN);  //x,y,value,x-line_end,y-line_end,color
         }
    */
  }


  tft.setFreeFont(FF33);//  tft.setFreeFont(&FreeSerif9pt7b);

}



void switchPins(int8_t /*TkindDUT*/ kind, int minBase, int maxBase, int incBase) {

  TurnOffLoad();
  // Serial.println("nach turnoff");
  idev = 84;                        // T: Transistor bipolar
  if (CurDUTclass == 1) idev = 77;  // M: Mosfet
  if (CurDUTclass == 2) idev = 70;  // F: jFet
  if (kind < 5) {                   // n-devices :  npn, n-Mos, n-jFet

    if (kind == tkNJFET)            // n-jFET or n-depletion-mode-MOS-FET
      digitalWrite(pin_EB, HIGH);   // DUT source to + (=Drain Currentsource), positive to DUT-gate                 Pin17=A3 nJFet
    //  else
    //    digitalWrite(pin_EB ,LOW);    // DUT source to groud, DUT gate to + (=Drain Currentsource),                            npn
    if (s_blue <= 4) {                // 2 sd_c
      digitalWrite(pin_E_NEG, HIGH);  // connect Emitter/Source/Cathode to ground
      delay(80);                      // for TCA0372
    }
    delayMicroseconds(20);//delay(10);

  }
  else {                            // p-devices :  pnp, p-Mos,p-jFet
    if (kind != tkPJFET)            // pnp, P-Mos Emitter positiv to base/Gate
      digitalWrite(pin_EB, HIGH);   //
    //    if (kind == tkPJFET)          // P-jfET
    //      digitalWrite(pin_EB ,LOW);  // DUT source to groud, DUT gate to + (=Drain Currentsource),                            pJFet

    if (s_blue <= 4) {                // 2 sd_c
      digitalWrite(pin_E_POS, HIGH);  // connect Emitter/Source/Cathode to + (Drain CCS)
      delay(80);                      // for TCA0372
    }

    delayMicroseconds(20);
  }

  minpix = minBase;                // 1 counts / µA maximal 4095 counts: 1µA ... 4.095 mA Metro
  maxpix = maxBase;
  incpix = incBase;
  /*
    if (s_blue != 2) {               // not for constant-voltage-source
    digitalWrite(pin_POW_CCS , HIGH); // used for ccs-i2c: switch +5V to the board
    delayMicroseconds(100);        // for start ccs-board
    delay(50);
    Wire.begin();
    dac_i2c.begin(0x60);   // I2C-DAC MCP4725 used for base-current/ gate-voltage
    delay(250);
    }
  */
  if (s_blue > 5)                  // 3 sd_c staircase generator delay between steps
    delay_CCS = delay_CCS_bas * delay_CCS_pow;      // 0 ... 16 µS * pow between steps

  if (idev == 84) return;          //(kind == tkPNP or kind == tkNPN) return; //##############################

  minpix = minBase * 10;           // 100mV = 10k * 10 µA ,  1µA = 1 counts v-gate  Mosfet/J_Faet
  maxpix = maxBase * 10;
  incpix = incBase * 10;           // 100mV = 10k * 10 µA
  digitalWrite(pin_CV_SW, HIGH);   // gateVoltage instead of baseCurrent
  delayMicroseconds(15);
}


void readADC_01() {  // read ADC0 for current on throgh RS and ADC1 for voltage on TCA0372 output

  // int icc,icr;
  delayMicroseconds(50);   //xx 8
  
  int icr = MCP_A.analogRead(0); // ADC0 counts , current threw Rs
    
/*  icr += MCP_A.analogRead(0);
  icr += MCP_A.analogRead(0);
  icr += MCP_A.analogRead(0);
  icr /= 4;*/
  //delayMicroseconds(10);   //xx 8   // 0 ???
  // Serial.print(" adc0: ");Serial.println(icr);
  //   if (idev==2) delayMicroseconds(30);
  int icc = MCP_A.analogRead(1);  // ADC1 counts, voltage
  // Serial.print(" adc1: ");Serial.println(icc);
  delayMicroseconds(50);   //50 xx 8
  
  //   intToBLE(2000+ifc);                               //  ifc  0...320 test =ta  #########################################
  //   intToBLE(3000+icr);                               //  a0-value     test =ta  #########################################
  fur = (float)icr;
  fur = fur * count_to_mv; // Vref /maxcount
  fur = fur + fur; // divider 10k/10k A0
  fir = fur / 50.0; //fi mA, frq: 50 Ohm or 500 Ohm now: 50 Ohm
 // MCP_D.setSPIspeed(12000000); // SPI.beginTransaction(settingsD);  // DAC: 12MHz
  error_i = 0;
  if (s_diode > 0 and fir > fidio) {
    digitalWrite(pin_E_POS , LOW); // disconnect Emitter/Source/Cathode from V+   Pin15=A1 pnp
    digitalWrite(pin_E_NEG , LOW); // disconnect Emitter/Source/Cathode from gnd  Pin14=A0 npn
    delayMicroseconds(8);
    // debug(icc);debug(icr);debug(fur);debug(fi);debugln(fidio);
    MCP_D.fastWriteA(0);
    MCP_D.fastWriteB(0);
    delayMicroseconds(20);
    // delay(8);

    error_i = 1;
    //  return;   //
  }
  else if (fir > imax) { //200.0) {   // 100.0
    digitalWrite(pin_E_POS , LOW); // disconnect Emitter/Source/Cathode from V+   Pin15=A1 pnp
    digitalWrite(pin_E_NEG , LOW); // disconnect Emitter/Source/Cathode from gnd  Pin14=A0 npn
    delayMicroseconds(8);
    MCP_D.fastWriteA(0);
    MCP_D.fastWriteB(0);
    delayMicroseconds(20);

    error_i = 2;
    //    return;  //
  }
  if (s_thresh == 1) {        // MosFet
    if (icr > 0) {
      fvthr = fvgate / 1000.0; // thresh.volt in volt
      s_thresh = 2;
    }
  }
  fuc  = (float)icc;
  fuc  = fuc * count_to_mv * 3.0;// divider 20k/10k A1 // metro mini/Nano
  fuc  = fuc - fur;    // Uce
  if (s_zen > 0) {
   // Serial.println(fuc);
    fuc = fuc - (float)(s_zen * 1000);  // -2V, -4V, -6V, -8V
  }
  fce  = fuc * finc;   // finc = 0.319/x_scale = 26mV (12V-scale) s_zen > 0:  0.159mV/pixel
  
  fi = fir * fui;         // 240/y_scale: 2=120,5=48,10=24,20=12,50=4.8,100=2.4     // 1mA = 4.8 Pixel
  if (fi > 239.0) fi = 239.0;
  //   fi = fi - 239.0;
  //   ifi = (int)fi;

  if (fce < 0.0) fce = 0.0;
  ifc = (int) fce;
  // debug(icc);debug(fuc);debug(fce);debug(ifc);debug("i:");debug(icr);debug(fur);debug(fi);debugln(ifi);
 // SPI.beginTransaction(settingsT);  // TFT: 24MHz
  return;

}

void charToBLE(char c) { // char to Bluetooth BLE
  Serial.write(c);
  Serial.write(lf);  // linefeed
}


void intToBLE(int ip) {   // int to Bluetooth BLE
  itoa(ip, buf, 10);  // 2
  char c;
  if (s_blue == 1  or s_blue == 3) {
    for (int8_t ilx; ilx < 5; ilx++)  {
      c = buf[ilx];
      SerialBT.write(c);
    }
    // Serial.write(buf);
    SerialBT.write(lf);//"\n");
  }
#ifdef SD_C
  if (s_sdcard == 1) {
    dataFile.print(buf);
    dataFile.print(',');   // or lf: \n
  }
#endif
}



#ifdef SD_C
/*
  void EEPROM.writeCharDataSD(TkindDUT kind)  {        // EEPROM.writeChar curve-header to SD-Card

  dataString = "";
  dataString += "X,";                     // x-scale,
  dataString += String(x_scale);
  dataString += ",Y,";                    // ,y-scale,
  dataString += String(y_scale);
  dataString += ',';                      // ,
  dataString += devPol;                   // N / P
  dataString += ',';                      // ,
  if      (idev == 84) dataString += 'T'; // T=Transisptor bipolar
  else if (idev == 77) dataString += 'M'; // M=Mosfet
  else if (idev == 70) dataString += 'F'; // F=j-Fet
  else                 dataString += 'D'; // F=j-Fet
  dataString += ",C,";                    // ,C, Start Curve
  }
*/
#endif

void transferData() {  // send data to PC or SmartPhone via Bluetooth (2.0) or SD-Card
  // char    type_d;//seper=',';
  // char    buf[12];
  // int     iy = 0;

  intToBLE(tsd + 88); // charToBLE('X'); //Serial.write('X');     Serial.write("\n");   // x-scale
  intToBLE(x_scale);  // itoa(x_scale, buf, 10); Serial.write(buf);     Serial.write("\n");
  intToBLE(1100 + cxs); // Serial.write(buf);     Serial.write("\n");  // 1101,...1106,
  intToBLE(tsd + 89); // charToBLE('Y');//Serial.write('Y');     Serial.write("\n");  // y-scale
  intToBLE(y_scale);  // itoa(y_scale, buf, 10); Serial.write(buf);     Serial.write("\n");
  intToBLE(1110 + cys); // Serial.write(buf);     Serial.write("\n"); // 1111,...1116,
  if (devPol == 'P') intToBLE(tsd + 80); // p-device  // charToBLE('P'); //Serial.write('P'); // 5=PNP,6=pMOSFet,7=pJFet,8=pDiode

  else               intToBLE(tsd + 78); // n-device //  charToBLE('N'); //Serial.write('N'); // 1=NPN,2=nMOSFet,3=nJFet,4=nDiode

  intToBLE(tsd + idev); // Serial.write('T','M','F'); // idev=84=T=Transistor(bipolar),77=M=Mosfet,70=F=j-Fet

  intToBLE(tsd + 67);                              //  Serial.write('C'); Serial.write("\n"); // curve Start
  /*    /// test
       for (uint16_t ilx=0;ilx<=ifc;ilx++)  {     // ilx: Pixel x, yct[ilx]: Pixel y (0=top of display)

      // itoa(ilx, buf, 10); Serial.write(buf);  Serial.write("\n");           // x-value (0...319)


         Serial.write(itoa(yct[ilx],buf,10));    // y-value (0...239) // test
         Serial.write(lf);//"\n");
       }
      // intToBLE(1100); //Serial.write(itoa(600,buf,10)); Serial.write("\n"); // y-value 1100 for end // ##### test

       intToBLE(tsd+81); //charToBLE('Q'); //  Serial.write('Q');   Serial.write("\n"); // curve End
  */
}

/*
  void drawGraph() {     // pixel-grafik for curves  ##################################
    uint8_t yh = 0;
    for (uint16_t ilx=0;ilx<=ifc;ilx++)  { // ilx: Pixel x, yct[ilx]: Pixel y (0=top of display)
      yh = 239 - yct[ilx];

      DrawPixel(ilx,yh,curve_col);//TFT_WHITE);   // curve to screen ##################################
      yct[ilx] = 0;
    }
  }
*/


void drawGraph() {  // pixel-grafik for curves jfet ##################################

  float fohm;
  uint8_t yh = 0;

  z_ohm = s_ohm;
  for (uint16_t ilx = 0; ilx <= ifc; ilx++)  { // ilx: Pixel x, yct[ilx]: Pixel y (0=top of display)
    yh = yct[ilx];
    if (z_ohm == 1)     {   // only j-Fet small x-scale ######################################
      if (ilx >= 160)   {  // 500 mV at 1 V scale
        fohm = (float)(yh) / fui; // i [mA] 10 mA scale  fui=240/y_scale: 2=120,5=48,10=24,20=12,50=4.8,100=2.4

        if (fohm > 1.0) z_ohm = 2; // mA

      }
    }
    yh = 239 - yh;

    DrawPixel(ilx, yh, curve_col); //TFT_WHITE);   // curve to screen ##################################
    yct[ilx] = 0;
    itoa(yh, buf, 10);
#ifdef BLUE_T
    if (s_blue == 1  or s_blue == 3) {
      //  Serial.write(itoa(yh, buf, 10));  // y-value (0...239) // test
      // Serial.write(buf);  // y-value (0...239) // test
      char c;
      for (int8_t ilx; ilx < 4; ilx++)  {
        c = buf[ilx];
        SerialBT.write(c);
      }
      SerialBT.write(lf);//"\n");
    }
#endif
#ifdef SD_C
    if (s_sdcard == 1) {
      //  dataFile.print(buf);
      dataFile.print(buf);  // y-value (0...239) // test
      dataFile.print(',');   // or lf: \n
    }
#endif
  }
  /*
    #ifdef BLUE_T
      if (s_blue == 1  or s_blue == 3) intToBLE(tsd+81); //charToBLE('Q'); //  Serial.write('Q');   Serial.write("\n"); // curve End
    #endif
  */
  if (s_ohm == 0) return;
  iohm = (int) (500.0 / fohm);  // 500mV/foh mA
  // Serial.print(z_ohm);Serial.print(" ");Serial.print(fohm);Serial.print(" ");Serial.println(iohm);



}


void specialParam(int8_t /*TkindDUT*/ kind) {

  s_idss   = 0;
  s_pinch  = 0;
  s_ohm    = 0;
  s_thresh = 0;
  if (idev == 70) { //if (kind == tkNJFET or kind == tkPJFET) {
    if (x_scale == 1) {
      s_pinch = 1;   // >=1 ??
      s_ohm   = 1;   // >=1 ??
    }
    else if (x_scale >= 3) s_idss = 1;  // 6 ??
    return;
  }

  if (kind == tkNMOSFET and x_scale >= 1) s_thresh = 1;  // 2 ??

}



void generCurve(int8_t /*TkindDUT*/ kind) {  // TCT with (1) or without bluetooth (0)

  //    uint16_t xl=0,yl=239;
  xlp = 0;
  ylp = 0;
  if (s_blue <= 3) {  // 1 sd_c
    igain = 0;
    //   ibc   = base/4;   // 1µA = 4 counts Dac-I  Nano
    ibc     = base;     // 1µA = 1 counts Dac-I  Metro
    // debug("maxpix");debug(maxpix);debug("base");debug(base);debug("ibc");debugln(ibc);
    fib = (float)ibc;
    fib = fib / 1000.0; // ibase in mA
    //drawBox_base(ibc,kind);          //#################
    fvgate = (float)(ibc * 10);  // mv
  }
  // Serial.println("vor settingsD");
  MCP_D.setSPIspeed(12000000); // SPI.beginTransaction(settingsD); //??  3.6 MHz DAC + 2 MHz ADC
  delay(1);   // 1

 // Serial.println("nach settingsD");
  MCP_D.fastWriteA(0);                   // ?? why ??
  MCP_D.fastWriteB(0);                   // ?? why ??
  delay(1);
  
// Serial.println("vor ubat = ... analogRead(3)");
// SPI.beginTransaction(settingsA);  // ADC: 2MHz
#ifdef OSCI
  ubat = count_to_mv * (float)analogRead(pin_ADC1_5);  // A1_5 ESP32 (GPIO33) 
#else
  MCP_A.setSPIspeed(2000000);  // 2 MHz only for analogRead(3)
  delay(1);
  ubat = count_to_mv * (float)MCP_A.analogRead(3);  // ADC3 MCP3204 
#endif  
  ubat = 0.001 * (ubat + ubat + ubat); //* adc0_refmaxcnt;  // ADC3 MCP3204  divider : 20k, 10k#########,maximal 10V ?121
  // Serial.println("vor setDac_A_B()");
  setDac_A_B();                        // dac incr and read ADC0/1   ###################################
  // Serial.println("nach setDac_A_B()");
//  SPI.beginTransaction(settingsT);  // TFT: 24MHz
  delay(1);
  // Serial.println("nach settingsT");
  batVolt();                           // display battery voltage
  // Serial.println("nach batVolt()");
 
  tft.setTextSize(1);
  tft.setFreeFont(FF33); // tft.setFreeFont(&FreeSerif9pt7b);
  if (s_blue == 4) return;             // constVoltage 2 sd_c
#ifdef BLUE_T
  if (s_blue == 1  or s_blue == 3) transferData(); // HM10 send header for curve to PC/SmatPhone via bluetooth (2.0) HC-05
#endif

#ifdef SD_C
  if (s_sdcard == 1) transferData();// EEPROM.writeChar header for curve to SD-Card
#endif

  drawGraph();                         // #####################

  //if (ibc >= 0) traceEnd(ibc, kind);   // ##########################
  traceEnd(ibc, kind);   // #

}



void startScan( int minBase, int maxBase, int incBase) {

  imax    = 480;
 // s_diode = 0;               //###############################
 // if (incBase == 0)   {      // diode ??
  if (s_diode == 1) {
    imax    = maxBase;// ??/10;//  50mA
    fidio   = (float)maxBase / 10.0; // current limit diode
  }
  fimax = (float)imax;
  fvgate = 0;
  if (s_zen > 0) finc = 0.319 / 2.0;            // mV to pixel 159.5 mV 2V-scale start with s_zen
  else           finc = 0.319 / (float)x_scale; // mV to pixel 26.58 mV
  incB = incBase;
  minB = minBase;
  maxB = maxBase;

  iohm    = 0;
  s_stop  = 0;


}


void changeCol() {

  run_nr++;
  curve_col = col_t[run_nr];     // change curve color
  // debug("col-p");debugln(run_nr);
  if (run_nr > 9) run_nr = 0;
  s_comp  = 0;
  s_compl = 0;
#ifdef BLUE_T
  if (s_blue == 1  or s_blue == 3)
    intToBLE(tsd + 90); //  charToBLE('Z'); //Serial.write('Z'); Serial.write("\n");  // end curve
#endif
#ifdef SD_C
  if  (s_sdcard == 1)
    intToBLE(tsd + 90); //  charToBLE('Z'); //Serial.write('Z'); Serial.write("\n");  // end curve
#endif
}


void  drawConstCurr()        {   // const-current (s_blue == 5)  // 3 sd_c
  DrawIntAt(60, 40, maxpix, 1, curve_col);
  //  tft.setCursor(27,65);tft.setTextColor(TFT_WHITE);
  //     tft.print(txm[ic]);
  DrawStringAt(110, 40, "uA", 1, curve_col);
  //  DrawCharAt(126,40,'A', 2, curve_col);
}

void drawConstCurrVolt(int ili) {   // const-voltage : s_blue == 4, const-current : s_blue == 5
  char* tx = " uA";   // µA   ????
  if (s_blue == 4) tx = " mV";
  //   if (s_diode == 1) tx = " mA";
  DrawIntAt(60, 40, ili, 1, curve_col);
  DrawString(tx, 2, curve_col);
}


uint8_t stopCurves()     { 
  if (s_touchi == 1) {
    uint16_t z = tft.getTouchRawZ();
    s_touchi = 0;
    if (z < 250)  return 0;  // not really touched
  }

  if (s_isr2  == 1) s_isr2  = 0;
  if (s_isr32 == 1) s_isr32 = 0;
  s_stop  = 0;
  TurnOffLoad();
  return 1;
}

//-------------------------------------------------------------------------
// ScanAllPos
//   draw curves for a component on the NPN side of the DUT socket
//
//   if user touches the screen at the end of a line, the scan is terminated and the function returns true
//-------------------------------------------------------------------------

void ScanAllPos(int8_t /*TkindDUT*/ kind, int minBase, int maxBase, int incBase) {  // nd1:

  int ili, ilx;
  startScan( minBase, maxBase, incBase);  // start-values

  // debug("allpos min");debug(minBase);debug("max");debug(maxBase);debug("inc");debug(incBase);


run_2n:   // nd2:
  s_isr2  = 0;
  s_isr32 = 0;
  // Serial.println("vor switchPins");
  switchPins(kind, minBase, maxBase, incBase);  // switch collector, emitter, base  #########
  // Serial.println("nach switchPins");
  if (s_blue == 4) {                            // ConstVoltage 2 sd_c
    ili = minBase + 10 * incBase + 100 * maxBase;
    i_CVS_B = ili / 2000;
    i_CVS_A = ili - i_CVS_B * 2000;
    i_CVS_B *= 400;
    drawConstCurrVolt(ili);
    s_diode = 1;                                // no base current

  }


  // dac_i2c.setVoltage(0,false);                  // set base-current/gate-voltage to 0 with i2C-DAC MCP4725 nd in TurnOff
  // delay(2);

  curve_col = TFT_WHITE;
  if (s_comp == 1 or s_compl == 1) changeCol();

  fib = 0.0;
  specialParam(kind);   // s_idss, s_pinch, s_ohm, s_thresh
  if (s_blue == 5) {    // 3 sd_c const-current ibase
    dac_i2c.setVoltage(maxpix, false);   // set base-current/gate-voltage with i2C-DAC MCP4725 nd-ib
    drawConstCurrVolt(maxpix);
    // DrawIntAt(60,40,maxpix,1,curve_col);
    goto end_n;
  }

run_2ns:
  s_isr2  = 0;
  s_isr32 = 0;
  ild = 0;
  if (s_diode == 1) minpix = maxpix;
  // Serial.print(" min: "); Serial.print(minpix);Serial.print(" inc: "); Serial.print(incpix);Serial.print(" max: "); Serial.println(maxpix);
  for (base = minpix; base <= maxpix; base += incpix) {
    s_isr32 = 0;
    // Serial.print("base n: "); Serial.println(base);
    //delayMicroseconds(100);//##################################################
    dac_i2c.setVoltage(base, false);   // set base-current/gate-voltage with i2C-DAC MCP4725 nd-ib

    if (s_blue > 5) {     // 3 sd_c
      delayMicroseconds(delay_CCS);   // 0µs= 14.5kHz, 100µs = 9.2kHz ,400µs = 4.
      ild++;
      if (s_blue == 8 and ild < 6) delayMicroseconds(1500); // 6 sd_c ######
    }

    else generCurve(kind);                          // curves #######################################################
    
    if (s_isr32 == 1 or s_touchi == 1) { // or s_isr2 == 1) {  // stop curves
      if (stopCurves() == 1) return;
    }

    if (s_diode == 1) goto end_n;  //?
    // ?? // if (s_pinch == 2) break;
  } // end for ...
  if (s_blue == 6) goto run_2ns;   // 4 sd_cstep-up
  if (s_blue >= 7) {               // 5 sd_c staircase-generator
    for (base = maxpix; base >= minpix; base -= incpix) {  // step-down
      dac_i2c.setVoltage(base, false);   // set base-current/gate-voltage with i2C-DAC MCP4725 nd-ib
      delayMicroseconds(delay_CCS);   // 0µs= 14.5kHz, 100µs = 9.2kHz ,400µs = 4.

      if (s_isr32 == 1 or s_isr2 == 1 or s_touchi == 1) {  // stop curves
        if (stopCurves() == 1) return;
      }  
    
    }
    if (s_blue == 7) goto run_2ns;       // 5 sd_c step-up
    if (s_blue == 8)  {                  // 6 sd_c change to other polarity
      changePolar(kind);                 // s_compl = 1;
      return;
    }

  }

  TurnOffLoad();  // ndy:

  EndScan(kind);

  if (s_stop == 1) return;      // back to menu
  if (s_comp == 1) goto run_2n; // compare with similar device same polarity

end_n:

  nextDevice(kind);  // nd

  if (s_comp  == 1) goto run_2n;           // compare with similar device same polarity

}  // ndz:



//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//-------------------------------------------------------------------------
// ScanAllNeg
//   draw curves for a component on the PNP side of the DUT socket
//
//   if user touches the screen at the end of a line, the scan is terminated and the function returns true
//-------------------------------------------------------------------------

void ScanAllNeg(int8_t /*TkindDUT*/ kind, int minBase, int maxBase, int incBase) {   // pd1

  int ili, ilx;
  startScan( minBase, maxBase, incBase);  // start-values

  // debug("allneg min");debug(minBase);debug("max");debug(maxBase);debug("inc");debug(incBase);

run_2p:   // pd2:
  s_isr2 = 0;
  s_isr32 = 0;
  switchPins( kind, minBase, maxBase, incBase);  // switch collector, emitter, base  #############################
  // dac_i2c.setVoltage(0,false);                   // set base-current/gate-voltage to 0 with i2C-DAC MCP4725 in TurnOffLoad()

  curve_col = TFT_WHITE;

  if (s_comp == 1 or s_compl == 1) changeCol();

  fib = 0;
  specialParam(kind);   // s_idss, s_pinch, s_ohm, s_thresh

  if (s_blue == 5) {   // 3 sd_c
    dac_i2c.setVoltage(maxpix, false);   // set base-current/gate-voltage with i2C-DAC MCP4725 nd-ib
    drawConstCurrVolt(maxpix);                     //
    // DrawIntAt(60,40,maxpix,1,curve_col);
    goto end_p;    // stop ?
  }



run_2ps:
  s_isr2  = 0;                 // ??
  s_isr32 = 0;
  ild = 0;
  if (s_diode == 1) minpix = maxpix;
  for (base = minpix; base <= maxpix; base += incpix) {
    dac_i2c.setVoltage(base, false);   // set base-current/gate-voltage with i2C-DAC MCP4725 nd-ib
    // delayMicroseconds(100);//##################################################
    // Serial.print("base_p: "); Serial.println(base);
    if (s_blue > 5) {  // 3 sd_c
      delayMicroseconds(delay_CCS);   // 0µs= 14.5kHz, 100µs = 9.2kHz ,400µs = 4.
      ild++;
      if (s_blue == 8 and ild < 6) delayMicroseconds(1500);//## 6 sd_c
    }
    else generCurve(kind);

    if (s_isr32 == 1 or s_isr2 == 1 or s_touchi == 1)  {  // stop curves
      if (stopCurves() == 1) return;
    }

    if (s_diode == 1) goto end_p;
    //  if (s_pinch == 2) break;
  }
  if (s_blue == 6) goto run_2ps;  // 4 sd_c ??
  s_isr2  = 0;
  s_isr32 = 0;
  if (s_blue >= 7) {              // 5 sd_c staircase-generator
    for (base = maxpix; base >= minpix; base -= incpix) {  // step-down
      dac_i2c.setVoltage(base, false);   // set base-current/gate-voltage with i2C-DAC MCP4725 nd-ib
      delayMicroseconds(delay_CCS);   // 0µs= 14.5kHz, 100µs = 9.2kHz ,400µs = 4.

      if (s_isr32 == 1 or s_isr2 == 1 or s_touchi == 1) {  // stop curves
        if (stopCurves() == 1) return;
      } 
    }
    if (s_blue == 7) goto run_2ps;  // 5 sd_c
    if (s_blue == 8) {       //  6 sd_c change to other polarity
      changePolar(kind);     // s_compl = 1;
      return;
    }

  }

  TurnOffLoad();  // pdy:

  EndScan(kind);

  if (s_stop == 1) return;       // back to menu
  if (s_comp == 1) goto run_2p;  // compare with similar device same polarity
end_p:   //
  s_isr2 = 0;
  nextDevice(kind); // pd

  if (s_comp  == 1) goto run_2p; // compare with similar device same  polarity

}




void drawBox_base(int i_base, TkindDUT kind) {
  int ivgate;
  DrawBox(14, 30, 85, 9 , TFT_DARKGREY);//BLACK);
  tft.setCursor(14, 37);
  // txtv = txt[(int)CurDUTclass];
  DrawString(txt[CurDUTclass], 1, TFT_ORANGE);  //
  if (CurDUTclass == 0) return; // bipolar

  // Mosfet and jFet
  ivgate = i_base * 10;
  DrawInt(ivgate, 1, TFT_ORANGE); //mV
  fvgate = (float)(ivgate);  // mv

}


//-------------------------------------------------------------------------
// ScanKind
//   draw curves for a component
//-------------------------------------------------------------------------

void ScanKind(int8_t /*TkindDUT*/ kind) {
  //  int minBase, maxBase, incBase, base_adj;
  //debug("ScanKind(kind):");debug(kind);debug("s_compl");debugln(s_compl);

  devPol = 'N';
  if (kind >= 5) devPol = 'P';
  InitGraph(kind);
  int minBase = 0;
  int maxBase = 200;
  int incBase = 10;
#ifdef IEPROM
  minBase = EEPROM.readShort(CurDUTclass * 20 + 4);
  maxBase = EEPROM.readShort(CurDUTclass * 20 + 6);
  incBase = EEPROM.readShort(CurDUTclass * 20 + 8);
#endif

  // Serial.println(" ScanKind");
nxt_kind:
  if (devPol == 'P') ScanAllNeg(kind, minBase, maxBase, incBase);  // (kind == tkPNP or kind == tkPMOSFET or kind == tkPJFET)
  else               ScanAllPos(kind, minBase, maxBase, incBase);  // (kind == tkNPN or kind == tkNMOSFET or kind == tkNJFET)


  if (s_compl == 1) {
    minBase = minB;
    incBase = incB;
    maxBase = maxB;

    kind = curkind;
    if (s_blue <= 3) {   // 1 sd_c
      if (kind == tkPNP or kind == tkNPN) DrawBox(138, 0, 43, 60, TFT_BLACK);
      else                                DrawBox(132, 0, 55, 60, TFT_BLACK);
      DrawKindStr(kind);
    }
    // debug("kind");debug(kind);debug("minB");debug(minBase);debug("incB:");debug(incBase);debug("maxB");debugln(maxBase);
    goto nxt_kind;
  }
}


void drawEdge(uint8_t nr, uint8_t s_pos) {  // frame
  uint8_t nr_a = nr;

   DrawFrame(xa, ya, wa, ha, TFT_NEARBLACK); //BLACK);  // erase old frame
  if (s_pos == 6 or s_pos == 7)  {  // 4,6 : isr from push-button
    if   (s_pos == 6)    DrawFrame(xa, ya, wa, ha, TFT_CYAN);   // 6
    else                 DrawFrame(xa, ya, wa, ha, TFT_RED);    // 7
    return;
  }
  if (s_pos == 4) {
    s_pos = 1;                     // cyan
    if      (rota2 > 0) nr++;
    else if (rota2 < 0) nr--;
    if (nr > 9) nr = 1;
    if (nr < 1) nr = 9;
    if (nr == 3 and rota2 < 0) nr = 8; // or 2 ??
    rota2 = 0;
    efnr = nr; //############################## new efnr ??
    if (s_isr1 == 1) s_isr1 = 0;   // D2  //?1
 // Serial.print("drawEdge efnr ");Serial.println(efnr);
 // if (nr == 9) s_9 = 1;
  }

 // if (nr_a >= 10) nr = nr_a - 6;  // minst,incst,maxts same position as amin,ainc,amax

  newParPos(nr, s_pos);           // new param.positions and frame for s_pos = 0,2,4,8


}

void newParPos(uint8_t nr, uint8_t s_pos) {
  
  xa = ft[1][nr];
  ya = ft[2][nr];
  wa = ft[3][nr];
  ha = ft[0][nr];// 19;
  //if (nr == 1 or nr == 2 or nr == 9) ha = 37;
  //  nr_l = nr;

  // if (s_pos == 0) DrawFrame(xa,ya,wa,ha,TFT_WHITE);
  if (s_pos == 1) DrawFrame(xa, ya, wa, ha, TFT_CYAN);
  if (s_pos == 2) DrawFrame(xa, ya, wa, ha, TFT_RED);
  if (s_pos == 3) DrawFrame(xa, ya, wa, ha, TFT_YELLOW); // orange
  //  if (s_pos == 9) DrawFrame(xa,ya,wa,ha,TFT_BLACK);

}

/*

  uint8_t HaveTouch() {
  int16_t x, y;

  // debug(" s_touch ");debugln(s_touch);

  GetTouch(&x, &y);
  /*
  // if (x < 70) {
  //   if     ( y <  80 ) return 5;  // bipolar
  //   else if( y < 160 ) return 6;  // Mosfet
  //   else if( y < 240 ) return 7;  // j-Fet
  // }

  if (x < 160 ) {
    if     ( y <  50 ) return 1;  // P-device
  }
  else if (x < 260 ) {
    if ( y <  50 )     return 3;  // return
  }
  else if (x < 300 ) {
    if ( y <  50 )     return 2;  // N-device
  }
  if ( y > 60  and
       x > 280)        return 4;  // return

  }

*/


uint16_t r_g_b(uint8_t r, uint8_t g, uint8_t b) {
  uint16_t  col_16 = 0;
  col_16 = (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
  return col_16;
}

/*
  #ifdef BMP

  //*************************************************************************************
  // Function name:           DrawBitmapMonoBits
  // Description:             Draw a black and white image stored as an array of bits
  //*************************************************************************************
  void DrawBitmapMonoBits(int16_t x, int16_t y, const uint8_t *bitmap, uint16_t color) {
  int16_t i, j, bits, n, w, h;
  w = pgm_read_byte(bitmap++);
  w = w | (pgm_read_byte(bitmap++) << 8);
  h = pgm_read_byte(bitmap++);
  h = h | (pgm_read_byte(bitmap++) << 8);
  //tft.setTextColor(color);
  // tft_fastSetup();
  n = 0;
  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++ ) {
      if (n % 8 == 0)
        bits = pgm_read_byte(bitmap++);

      if ((bits & 0x80) == 0)
        tft.drawPixel(x + i, y + j, color); //pgm_read_word(bitmap++));
      //   tft_fastPixel(x + i, y + j, color);

      bits = bits << 1;
      n++;
    }
  }
  }


  //***************************************************************************************
  // Function name:           DrawBitmapMono
  // Description:             Draw a black and white image stored in an array on the TFT
  //***************************************************************************************
  void DrawBitmapMono(int16_t x, int16_t y, const uint8_t *bitmap, uint16_t color) {
  uint8_t bmp; //bmp;
  int16_t w;

  bmp = bitmap;//(int)bitmap;
  w = pgm_read_byte(bmp++);
  w = pgm_read_byte(bmp++);

  if ((w & 0x80) > 0)  // 0x80 = 128
    DrawBitmapMonoRLE(x, y, bitmap, color);
  else
    DrawBitmapMonoBits(x, y, bitmap, color);
  }



  //**************************************************************************************
  // Function name:           DrawBitmapMonoRLE
  // Description:             Draw a black and white image stored as RLE
  //**************************************************************************************
  void DrawBitmapMonoRLE(int16_t x, int16_t y, const uint8_t *bitmap, uint16_t color) {
  int16_t i, j, nb, w, h;
  bool CurIsBlack;

  w = pgm_read_byte(bitmap++);
  w = w | (pgm_read_byte(bitmap++) << 8);
  w = w & 0x7FFF;
  h = pgm_read_byte(bitmap++);
  h = h | (pgm_read_byte(bitmap++) << 8);

  //tft_fastSetup();

  nb = 0;
  CurIsBlack = true;
  j = 0;
  i = 0;

  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++ ) {
      while (nb == 0) {
        nb = pgm_read_byte(bitmap++);
        CurIsBlack = !CurIsBlack;
      }

      if (!CurIsBlack)
        tft.drawPixel(x + i, y + j, color);   //tft_fastPixel(x + i, y + j, color);
      nb--;
    }
  }
  }

  #endif
*/


/*
#ifdef TOUCH
void touch_calibrate()
{
  uint16_t calData[5];
  uint8_t calDataOK = 0;

  // check file system exists
  if (!SPIFFS.begin()) {
    Serial.println("formatting file system");
    SPIFFS.format();
    SPIFFS.begin();
  }

  // check if calibration file exists and size is correct
  if (SPIFFS.exists(CALIBRATION_FILE)) {
    if (REPEAT_CAL)
    {
      // Delete if we want to re-calibrate
      SPIFFS.remove(CALIBRATION_FILE);
    }
    else
    {
      File f = SPIFFS.open(CALIBRATION_FILE, "r");
      if (f) {
        if (f.readBytes((char *)calData, 14) == 14)
          calDataOK = 1;
        f.close();
      }
    }
  }

  if (calDataOK && !REPEAT_CAL) {
    // calibration data valid
    tft.setTouch(calData);
  } else {
    // data not valid so recalibrate
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(20, 0);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    tft.println("Touch corners as indicated");

    tft.setTextFont(1);
    tft.println();

    if (REPEAT_CAL) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.println("Set REPEAT_CAL to false to stop this running again!");
    }

    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Calibration complete!");

    // store data
    File f = SPIFFS.open(CALIBRATION_FILE, "w");
    if (f) {
      f.write((const unsigned char *)calData, 14);
      f.close();
    }
  }
}

#endif

*/

#ifdef TOUCH
///------------------------------------------------------------------------------------------

void IRAM_ATTR isr_touch() {  // IRQ-Pin Touch
  s_touchi = 1;
 
}

#endif



void setup(void) {

  Serial.begin(9600); // rs232 TX ??
// Serial.begin(115200); // For debug
// delay(2000);

// Serial.println(" nach Serial.begin");
#ifdef TOUCH
  pinMode(pin_TOUCH_IRQ, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin_TOUCH_IRQ), isr_touch, FALLING); //?
#endif
  pinMode(Pin_32_BUT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Pin_32_BUT), isr_but32, FALLING); //?

  pinMode(pin_ENC1_A, INPUT);
  pinMode(pin_ENC1_B, INPUT);
  pinMode(pin_ENC2_A, INPUT);
  pinMode(pin_ENC2_B, INPUT);
  enc1_State_o = digitalRead(pin_ENC1_A);     //Read First Position of Channel A
  attachInterrupt(digitalPinToInterrupt(pin_ENC1_A), isr_ENC1, RISING);
  attachInterrupt(digitalPinToInterrupt(pin_ENC2_A), isr_ENC2, RISING);
  // Serial.println(" nach attach");
  pinMode(pin_ADC_CS, OUTPUT);
  digitalWrite(pin_ADC_CS, HIGH);
  pinMode(pin_DAC_CS, OUTPUT);
  digitalWrite(pin_DAC_CS, HIGH);   // disable MCP4822 DAC (active low)
  pinMode(pin_CV_SW , OUTPUT);
  digitalWrite(pin_CV_SW , LOW);
  pinMode(pin_E_NEG , OUTPUT);
  digitalWrite(pin_E_NEG , LOW);
  pinMode(pin_E_POS , OUTPUT);
  digitalWrite(pin_E_POS , LOW);
  pinMode(pin_EB , OUTPUT);
  digitalWrite(pin_EB , LOW);
//  pinMode(pin_TOUCH_CS , OUTPUT);
//  digitalWrite(pin_TOUCH_CS , HIGH);  // disable Touch

#ifdef OSCI
#ifdef DAC_INT
  pinMode(pin_DAC1, OUTPUT);
#endif  
#endif
  
  // Serial.println(" vor tft.init");
  tft.init();     
  tft.setRotation(1);
 
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
 
  tft.setFreeFont(FF33);// tft.setFreeFont(&FreeSerif9pt7b);  //#####

 /*
 #ifdef TOUCH
   // Calibrate the touch screen and retrieve the scaling factors
//  touch_calibrate();
 #endif
 */
  Wire.begin();
  dac_i2c.begin(0x60);   // I2C-DAC MCP4725 used for base-current/ gate-voltage
  delay(50);

 
#ifdef  BLUE_T
//  tft.println("Serial Bluetooth-Test ");
//  SerialBT.begin("ESP32");  //########## inactive
//  tft.println("BLE-device started ");
  s_blue = 1;
  
 /* while(1) {
    for (int16_t il=0;il<=20;il++) {
      intToBLE(tsd + il); 
      delay(1);  
    }  
  }
 */
#endif

// Serial.println("vor init_EEPROM");
  init_EEPROM();         // only first time ? #####################################################
  
  if (CurDUTclass == 8) {
    s_diode = 1;
    CurDUTclass = 0; // bipolar
  }
  if (CurDUTclass == 9) {
    s_depl = 1;
    CurDUTclass = 2; // jFet
  }
  // header("Using print() method", TFT_NAVY);
  // tft.print("nach CDC "); tft.println(CurDUTclass);
  // delay(2000);
  // SPI.begin();
  // Serial.println("vor DAC");
  // tft.setCursor(20,40);tft.println("vor DAC");delay(1000);

  MCP_A.selectVSPI();           // CLK=18,MISO=19,MOSI=23(,CS=5) 
  MCP_A.begin(2);               // ADC chip select pin CS=2

  MCP_A.setSPIspeed(2000000);   // seems to be the max ADC-speed. use 1.6 MHz (default) to be safe


#ifdef OSCI
// drawSinus();
  readADC_23();
  s_isr32 = 0;
  s_osci = 0;
  MCP_A.setSPIspeed(2000000); 
#endif
  
  
  myspi->begin();
  MCP_D.begin(0); //pin_DAC_CS);     //##################################################   // 

 // MCP_A.setSPIspeed(2000000);  // 2 MHz ?
  
  TurnOffLoad();
  
}


#ifdef OSCI

#ifdef DAC_INT
void selCurveDac()  {      //    select curve for DAC1
  int8_t ic;
  s_curve=0;
  char* txm[6] = { "No ?", "Sine ?", "Square ?", "Triangle ?", "Step-Up", "Step-Down ?"};

  tft.setFreeFont(FF33); // tft.setFreeFont(&FreeSerif9pt7b);  // font??
 
  s_isr2 = 0;
  s_isr32 = 0;
  while (1)      {
    for (ic = 0; ic <= 5; ic++) {
      DrawBox(27, 53, 92, 18, TFT_BLACK);//DARKGREY);
   //   tft.setCursor(27, 65); tft.setTextColor(TFT_WHITE);tft.print(txm[ic]);
      DrawStringAt(27, 65,txm[ic], 1, TFT_WHITE);
      delay(1200);
      if (s_isr2 == 1 or  s_isr32 == 1) {
        s_isr2  = 0;
        s_isr32 = 0;
        if      (ic == 0) s_curve = 0;  // no
        else if (ic == 1) s_curve = 1;  // sine
        else if (ic == 2) s_curve = 2;  // square
        else if (ic == 3) s_curve = 3;  // triangle
        else if (ic == 4) s_curve = 4;  // step_up
        else if (ic == 5) s_curve = 5;  // step-down
       
        return;
      } // if
    }   // for
  }     // while
}
#endif

void DrawBox20(uint16_t x, uint16_t y, uint16_t back_col, uint16_t sign_col ) {
  
   DrawBox(x, y, 20, 20, back_col);
   
   DrawLine(x+4, y +9, x+15, y +9, sign_col); // horis.
   DrawLine(x+4, y+10, x+15, y+10, sign_col); // horis.
   if (back_col == TFT_RED) {                 // vertical
     DrawLine(x +9, y+4,x +9,y+15, sign_col);
     DrawLine(x+10, y+4,x+10,y+15, sign_col);
   }   
}  

#ifdef TOUCH

void touchCurveDac() {
   uint16_t t_x = 200, t_y = 0; // To store the touch coordinates
   tft.setFreeFont(FF33); // tft.setFreeFont(&FreeSerif9pt7b);  // font??
   
   if (s_curve > 0) goto osci_param;  // change only offset, gain, delay

   s_chan1 = 0;
   s_chan2 = 0;
   DrawBox(  2, 4, 45, 40, TFT_CYAN);
   DrawStringAt(6, 20, "no", 1, TFT_BLACK);
   DrawStringAt(5, 34, "DAC", 1, TFT_BLACK);
   DrawBox( 49, 4, 47, 40, TFT_YELLOW);
   DrawStringAt(52, 27, "Sine", 1, TFT_BLACK);
   DrawBox(98, 4, 74, 40, TFT_LIGHTTUERK);
   DrawStringAt(100, 27, " Square", 1, TFT_BLACK);
   DrawBox(174, 4, 68, 40, TFT_LIGHTRED);
   DrawStringAt(176, 27, "Triangle", 1, TFT_BLACK);
   DrawBox(244, 4, 75, 40, TFT_BRIGHTBLUE);
   DrawStringAt(246, 20, "Staircase", 1, TFT_BLACK);
   DrawStringAt(246, 34, " up  down", 1, TFT_BLACK);
   
   DrawBox( 2, 46, 68, 40, TFT_ORANGE);
   DrawStringAt(4, 72, "Channel", 1, TFT_BLACK);
   DrawBox( 72, 46, 81, 40, TFT_LIGHTTUERK);
   DrawStringAt(78, 72, "  Offset", 1, TFT_BLACK);
   DrawBox( 155, 46, 81, 40, TFT_CYAN);
   DrawStringAt(160, 72, "   Gain", 1, TFT_BLACK);
   DrawBox( 238, 46, 81, 40, TFT_LIGHTRED);
   DrawStringAt(244, 72, "  Delay", 1, TFT_BLACK);
      
   delay(200);
  
   s_curve = 9;
   while(s_curve == 9) {
     bool pressed = tft.getTouch(&t_x, &t_y);
     if (pressed) {
     
       if(t_y > 4 and t_y < 38) {
         if      (t_x >   4 and t_x <  40) s_curve = 0; 
         else if (t_x >  50 and t_x <  80) s_curve = 1; 
         else if (t_x >  85 and t_x < 150) s_curve = 2; 
         else if (t_x > 165 and t_x < 240) s_curve = 3; 
         else if (t_x > 250 and t_x < 285) s_curve = 4; 
         else if (t_x > 290 and t_x < 318) s_curve = 5; 
     
       }
     }  
   
   }
   if      (s_curve == 0) tft.drawRect(  4, 5, 43, 38, TFT_RED);
   else if (s_curve == 1) tft.drawRect( 50, 5, 45, 38, TFT_RED);
   else if (s_curve == 2) tft.drawRect( 99, 5, 72, 38, TFT_RED);
   else if (s_curve == 3) tft.drawRect(175, 5, 66, 38, TFT_RED);
   else if (s_curve == 4) tft.drawRect(245, 5, 30, 38, TFT_RED);
   else if (s_curve == 5) tft.drawRect(278, 5, 38, 38, TFT_RED);

   delay(1);
   
  
  osci_param:
  
   DrawBox(14, 110, 20, 20, TFT_BRIGHTBLUE);  //< 112 110
   DrawCharAt(20, 126, '1', 1, TFT_BLACK);
 
   DrawBox(14, 160, 20, 20, TFT_LIGHTTUERK);  //<184 160
   DrawCharAt(20, 176, '2', 1, TFT_BLACK);    //<198 176

   if (s_chan1 == 0 and s_chan2 == 0) {
     while(1) {
       bool pressed = tft.getTouch(&t_x, &t_y);
       if (pressed) {
     //    tft.fillCircle(t_x, t_y, 2, TFT_WHITE);
         if (t_x > 10 and t_x < 40) {
           if (t_y >  90 and t_y < 125) s_chan1 = 1;  
           if (t_y > 140 and t_y < 175) s_chan2 = 1;   //< 150 190
           if (t_y > 210 and t_y < 237) break; 
         }  
         if (t_x > 280 and t_y > 210 and t_y < 237) break;
       }
       
       if (s_chan1 == 1) DrawFrame(16,112,16,16,TFT_RED);
       if (s_chan2 == 1) DrawFrame(16,162,16,16,TFT_RED);  //<186 162
     }
   }
   if (s_chan1 == 0 and s_chan2 == 0) return;
   if (s_chan1 == 1) {
   
     DrawBox20( 77, 110, TFT_RED, TFT_WHITE);  //DrawBox(77, 110, 20, 20, TFT_RED);
     DrawBox20(128, 110, TFT_BLUE, TFT_WHITE); //DrawBox(128, 110,    20, 20, TFT_BLUE);
     DrawIntAt(98, 126, s_offs1, 1, TFT_WHITE);
     
     DrawBox20(160, 110, TFT_RED, TFT_WHITE);  //DrawBox(160, 110, 20, 20, TFT_RED);
     DrawBox20(214, 110, TFT_BLUE, TFT_WHITE); //DrawBox(214, 110, 20, 20, TFT_BLUE);
     DrawFloatAt(180,126, f_gain1, 1, 1, TFT_BRIGHTBLUE); 
     
     DrawBox20(242, 110, TFT_RED, TFT_WHITE);  //DrawBox(242, 110, 20, 20, TFT_RED);
     DrawBox20(295, 110, TFT_BLUE, TFT_WHITE); //DrawBox(295, 110, 20, 20, TFT_BLUE);
     DrawIntAt(264, 126, s_delay1, 1, TFT_WHITE);
   }
   if (s_chan2 == 1) {
          
     DrawBox20( 77, 160, TFT_RED, TFT_WHITE);  //DrawBox(77, 182, 20, 20, TFT_RED);
     DrawBox20(128, 160, TFT_BLUE, TFT_WHITE); //DrawBox(128, 182, 20, 20, TFT_BLUE);
     DrawIntAt(98, 176, s_offs2, 1, TFT_WHITE);
     
     DrawBox20(160, 160, TFT_RED, TFT_WHITE);  //DrawBox(160, 182, 20, 20, TFT_RED);
     DrawBox20(214, 160, TFT_BLUE, TFT_WHITE); //DrawBox(214, 182, 20, 20, TFT_BLUE);
     DrawFloatAt(180,176, f_gain2, 1, 1, TFT_BRIGHTBLUE); 
     
     DrawBox20(242, 160, TFT_RED, TFT_WHITE);  //DrawBox(242, 182, 20, 20, TFT_RED);
     DrawBox20(295, 160, TFT_BLUE, TFT_WHITE); //DrawBox(295, 182, 20, 20, TFT_BLUE);
     DrawIntAt(264, 176, s_delay2, 1, TFT_WHITE);
    
   }
   while(1) {
     bool pressed = tft.getTouch(&t_x, &t_y);
     if (pressed) {
      //   tft.fillCircle(t_x, t_y, 2, TFT_WHITE);
       if (s_chan1 and t_y > 90 and t_y < 125) {  //<160 135
         if(t_x > 70 and t_x < 100) {
           s_offs1++;
           if (s_offs1 > 100) s_offs1 = 100;
           DrawBox(98,  110, 29, 20, TFT_BLACK);
           DrawIntAt(98, 126, s_offs1, 1, TFT_WHITE);
         }
         if(t_x > 110 and t_x < 140) {
           s_offs1--;
           if (s_offs1 < -99) s_offs1 = -99;
           DrawBox(98,  110, 29, 20, TFT_BLACK);
           DrawIntAt(98, 126, s_offs1, 1, TFT_WHITE);
         }
         if(t_x > 160 and t_x < 190) {
           f_gain1 = f_gain1 + 0.1;
           if (f_gain1 > 20.0) f_gain1 = 20.0;
           DrawBox(180, 110, 32, 20, TFT_BLACK);
           DrawFloatAt(180,126, f_gain1, 1, 1, TFT_BRIGHTBLUE); 
         }
         if(t_x > 210 and t_x < 240) {
           f_gain1 = f_gain1 - 0.1;
           if (f_gain1 < 0.1) f_gain1 = 0.1;
           DrawBox(180, 110, 32, 20, TFT_BLACK);
           DrawFloatAt(180,126, f_gain1, 1, 1, TFT_BRIGHTBLUE); 
         }
         if(t_x > 250 and t_x < 280) {
           s_delay1++;
           if (s_delay1 > 255) s_delay1 = 255;
           DrawBox(263, 110, 31, 20, TFT_BLACK);
           DrawIntAt(264, 126, s_delay1, 1, TFT_WHITE);
         }
         if(t_x > 290 and t_x < 318) {
           s_delay1--;
           if (s_delay1 < -255) s_delay1 = -255;
           DrawBox(263, 110, 31, 20, TFT_BLACK);
           DrawIntAt(264, 126, s_delay1, 1, TFT_WHITE);
         }
       } // if s_chan1 ...
       else if (s_chan2 and t_y > 140 and t_y < 175) {  //<180 220  150 190
         if(t_x > 70 and t_x < 100) {
           s_offs2++;
           if (s_offs2 > 100) s_offs2 = 100;
           DrawBox(98,  160, 29, 20, TFT_BLACK);      //<182 160
           DrawIntAt(98, 176, s_offs2, 1, TFT_WHITE); //<198 176
         }
         if(t_x > 110 and t_x < 140) {
           s_offs2--;
           if (s_offs2 < -99) s_offs2 = -99;
           DrawBox(98,  160, 29, 20, TFT_BLACK);      //<182 160
           DrawIntAt(98, 176, s_offs2, 1, TFT_WHITE); //<198 176
         }
         if(t_x > 160 and t_x < 190) {
           f_gain2 = f_gain2 + 0.1;
           if (f_gain2 > 20.0) f_gain2 = 20.0;
           DrawBox(180, 160, 32, 20, TFT_BLACK);                 //<182 160
           DrawFloatAt(180,176, f_gain2, 1, 1, TFT_BRIGHTBLUE);  //<198 176 
         }
         if(t_x > 210 and t_x < 240) {
           f_gain2 = f_gain2 - 0.1;
           if (f_gain2 < 0.1) f_gain2 = 0.1;
           DrawBox(180, 160, 32, 20, TFT_BLACK);                 //<182 160
           DrawFloatAt(180,176, f_gain2, 1, 1, TFT_BRIGHTBLUE);  //<198 176 
         }
         if(t_x > 250 and t_x < 280) {
           s_delay2++;
           if (s_delay2 > 255) s_delay2 = 255;
           DrawBox(263, 160, 31, 20, TFT_BLACK);         //<182 160
           DrawIntAt(264, 176, s_delay2, 1, TFT_WHITE);  //<198 176 
         }
         if(t_x > 290 and t_x < 318) {
           s_delay2--;
           if (s_delay2 < -255) s_delay2 = -255;
           DrawBox(263, 160, 31, 20, TFT_BLACK);         //<182 160
           DrawIntAt(264, 176, s_delay2, 1, TFT_WHITE);  //<198 176 
         }
       }
     }
     if (t_x > 250 and t_x < 310) {
       if (t_y >  45 and t_y <  75) return;
       if (t_y > 190 and t_y < 230) return;
     } 
   }    
   delay(200);
}
#endif  // Touch

void readADC_23() {
  uint16_t anzVal = TFT_WID;
  
 // char*   txm[6] = { "NO DAC ","  Sine", "  Square", "Triangle", "Step-Up", "StepDown"};
  s_osci = 1;
  s_isr32 = 0;
  s_isr1  = 0;
  s_curve = 0;
  int delx = 0,cnt_a2=0;
  unsigned long sampleTime=0;
  

  
#ifdef DAC_INT  
#ifdef TOUCH 
  delay(1); 
 xy_param:
  touchCurveDac();
#else  
  selCurveDac();
#endif  

  if ( s_curve > 0) {
    if (s_curve == 1) { 
 
      float ConversionFactor=(2*PI)/256;        // convert my 0-255 bits in a circle to radians
                                            // there are 2 x PI radians in a circle hence the 2*PI
                                            // Then divide by 256 to get the value in radians
                                            // for one of my 0-255 bits.
      float RadAngle;                           // Angle in Radians
      // calculate sine values
      for(uint16_t MyAngle=0;MyAngle<256;MyAngle++) {
        RadAngle=MyAngle*ConversionFactor;           // 8 bit angle converted to radians
        SineValues[MyAngle]=(sin(RadAngle)*126)+127; // get the sine of this angle and 'shift' up so
                                                 // there are no negative values in the data
                                                 // as the DAC does not understand them and would
                                                 // convert to positive values.
      }
    }
    else if (s_curve == 2) {  // square-wave
    }
    else if (s_curve == 3) {  // triangle
    }
    else if (s_curve == 4) {  // step-up
    }
    else if (s_curve == 5) {  // step-down
    }
  }  
#endif // DAC_INT

  s_isr32 = 0;
  while (s_isr32 == 0) {
    InitGraph(0);
    uint16_t idac;
    uint8_t state = 0;
    tft.setCursor(80,60); tft.setTextColor(TFT_BLACK);  tft.print("time: ");tft.println(sampleTime);  
    MCP_A.setSPIspeed(6000000);  // 2 MHz ? ADC
    delay(1);
    int8_t s_start = 0;
    uint16_t ilr = 0;
 //   readStartTime = micros();
    uint8_t val_high = 1,incr=1,s_trig=1;  //################
    int16_t maxVal = 255;
    anzVal = 320;
    if (s_delay1 < 0)  maxVal = -s_delay1;
    
    if (s_delay2 < 0)  maxVal = -s_delay2;
       
    readStartTime = micros();
    
    while (ilr < anzVal) {  // 320*4 ??
#ifdef DAC_INT
      if (s_curve == 1) {  // sine
        if (ilr <= 255) idac = ilr;
        else            idac = ilr - 255;  
        dacWrite(pin_DAC1,SineValues[idac]); // Dac1 GPIO25  ########################
     // tft.setCursor(80,80); tft.setTextColor(TFT_WHITE);  tft.print("idac: "); tft.println(SineValues[idac]);
      }  
      else if (s_curve == 2) { // square
        if (val_high == 1) {
          dacWrite(pin_DAC1,maxVal); // Dac1 GPIO25   
        //  if (s_delay1 > 0) delayMicroseconds(s_delay1);
          val_high = 0;
        }
        else if (val_high == 0) {
          dacWrite(pin_DAC1,0); // Dac1 GPIO25 
      //    if (s_delay1 > 0) delayMicroseconds(s_delay1);  
          val_high = 1;
          
        }
      }
      else if (s_curve == 3) { // triangle
        if (incr == 1) {       
          if (val_high < maxVal-1) {
            val_high++;
            dacWrite(pin_DAC1,val_high); // Dac1 GPIO25   
          }
          else incr = 0;
        }    
        else if (incr == 0) {
          if (val_high >= 1) {
            val_high--;
            dacWrite(pin_DAC1,val_high); // Dac1 GPIO25   
          }
          else incr = 1;
        }
      }
      else if (s_curve == 4) { // staircase : step up
        if (val_high < maxVal - 1) val_high++;
        else val_high = 0;
        dacWrite(pin_DAC1,val_high); // Dac1 GPIO25   
      }
      else if (s_curve == 5) { // staircase : step down
        if (val_high > 0) val_high--;
        else val_high = maxVal - 1;
        dacWrite(pin_DAC1,val_high); // Dac1 GPIO25  
      }
#endif   // DAC_INT

      if (s_trig == 0) {
        if (MCP_A.analogRead(2) < 100) {
          s_trig = 1;
          readStartTime = micros();
        }
      }
      if (s_trig == 1) {
       // if (s_delay1 > 0) delayMicroseconds(s_delay1);
       // if (s_delay2 > 0) delayMicroseconds(s_delay2);
        if (s_chan1 == 1) inpBuf2[ilr] = MCP_A.analogRead(2);
     
        if (s_chan2 == 1) inpBuf3[ilr] = MCP_A.analogRead(3);
    
        ilr++;
        
      }  
     
    }  
    sampleTime = micros() - readStartTime;
   
    tft.setCursor(80,60); tft.setTextColor(TFT_WHITE);  tft.print("time: ");tft.println(sampleTime); 
    
    if (s_chan1 == 1) drawCurve1();
    if (s_chan2 == 1) drawCurve2();

    /*
    uint16_t ilm=0,ilx1=0,ilx2=0;
    for(uint16_t ilx=0;ilx<320;ilx++) {
    
      val_A2 = inpBuf2[ilx];
      val_A2 = val_A2/17 + s_offs1;  //
      val_A2 = val_A2 * f_gain1;
      if (val_A2 > 239) val_A2 = 239; 
      val_A2 = 239 - val_A2;         // 240 ####
      tft.drawPixel(ilx, oldBuf2[ilx], TFT_BLACK);
      newBuf2[ilx] = val_A2;
      
      tft.drawPixel(ilx, val_A2, TFT_LIGHTTUERK);//GREENYELLOW); 
      ilx1++;
      oldBuf2[ilx1] = val_A2; 
      if (s_delay1 > 0) {
        ilm = s_delay1;
        for (int ilq = ilm;ilq>0;ilq--) {
          ilx1++;
          if (ilx1 < 320) {
            tft.drawPixel(ilx1, val_A2, TFT_LIGHTTUERK);//GREENYELLOW); 
            oldBuf2[ilx1] = val_A2; 
          }
        }
      }
      
   
      val_A3 = inpBuf3[ilx];
      val_A3 = val_A3/17 + s_offs2;
      val_A3 = val_A3 * f_gain2;
      if (val_A3 > 239) val_A3 = 239; 
      val_A3 = 239 - val_A3;         // 240 ####
      tft.drawPixel(ilx2, oldBuf3[ilx], TFT_BLACK);
      newBuf3[ilx2] = val_A3;
      if (ilx2 < 320) ilx2++;
      tft.drawPixel(ilx2, val_A3, TFT_CYAN); 
      
      oldBuf3[ilx2] = val_A3; 
        if (s_delay2 > 0) {
        ilm = s_delay2;
        for (int ilq = ilm;ilq>0;ilq--) {
          ilx2++;
          if (ilx2 < 320) {
            tft.drawPixel(ilx2, val_A3, TFT_CYAN); 
            oldBuf3[ilx2] = val_A3; 
          }
        }
      }
   
    }
    */
    uint16_t itx=0,t_x,t_y; 
    while (itx < 1000) {

      bool press_l = tft.getTouch(&t_x, &t_y);
      if (press_l) {
        if (t_y > 200 and t_x > 280) goto xy_param;
      }
      itx++;
      if (s_isr32 == 1) return;
    }  
  }
  
  
}


void drawCurve1() {
    uint16_t ilm=0,ilx1=0;
    for(uint16_t ilx=0;ilx<320;ilx++) {
    
      val_A2 = inpBuf2[ilx];
      val_A2 = val_A2/17 + s_offs1;  //
      val_A2 = val_A2 * f_gain1;
      if (val_A2 > 239) val_A2 = 239; 
      val_A2 = 239 - val_A2;         // 240 ####
      tft.drawPixel(ilx1, oldBuf2[ilx], TFT_BLACK);   // delete old
      newBuf2[ilx1] = val_A2;
      tft.drawPixel(ilx1, val_A2, TFT_LIGHTTUERK);    // paint new 
      if (ilx1 < 320) ilx1++;
      else break;
      oldBuf2[ilx1] = val_A2; 
      if (s_delay1 > 0) {
        ilm = s_delay1;
        for (int ilq = ilm;ilq>0;ilq--) {
          ilx1++;
          if (ilx1 < 320) {
            tft.drawPixel(ilx1, val_A2, TFT_LIGHTTUERK);//GREENYELLOW); 
            oldBuf2[ilx1] = val_A2; 
          }
          else return;
        }
      }
    }
}


void drawCurve2() {
    uint16_t ilm=0,ilx2=0;
    for(uint16_t ilx=0;ilx<320;ilx++) {
    
      val_A3 = inpBuf3[ilx];
      val_A3 = val_A3/17 + s_offs2;  //
      val_A3 = val_A3 * f_gain2;
      if (val_A3 > 239) val_A3 = 239; 
      val_A3 = 239 - val_A3;         // 240 ####
      tft.drawPixel(ilx2, oldBuf3[ilx], TFT_BLACK);   // delete old
      newBuf3[ilx2] = val_A3;
      tft.drawPixel(ilx2, val_A3, TFT_CYAN);          // paint new 
      if (ilx2 < 320) ilx2++;
      else break;
      oldBuf3[ilx2] = val_A3; 
      if (s_delay2 > 0) {
        ilm = s_delay2;
        for (int ilq = ilm;ilq>0;ilq--) {
          ilx2++;
          if (ilx2 < 320) {
            tft.drawPixel(ilx2, val_A3, TFT_CYAN);
            oldBuf3[ilx2] = val_A3; 
          }
          else return;
        }
      }
    }
}


#endif





void init_EEPROM() {
  
  EEPROM.begin(256);
  CurDUTclass = EEPROM.readChar(99);  // 0=bipolar,1=Mosfet,2=jFet          ##### from last run ###
  // ?? 3 =const.volt.source, 4 = const.curr.source
  // 8=s_diode, 9=s_depl
  // Update EEprom_fields
  //Serial.print(" cdc "); Serial.println(CurDUTclass);
  //################################################################################
    int ila = 0;          // CurDUTclass;  //0 bip, 1 mos, 2 jfet 8 diode 9 depletion ##########
    
    EEPROM.writeChar(1 + ila, 0);  // s_blue
    EEPROM.writeChar(2 + ila, 6); // x_scale
    EEPROM.writeChar(3 + ila, 4);  // y_scale
    EEPROM.writeShort(4 + ila, 0); // amin);
    EEPROM.writeShort(6 + ila, 180); //amax);
    EEPROM.writeShort(8 + ila, 20); // ainc);
    EEPROM.writeShort(10 + ila,  1); //minst);
    EEPROM.writeShort(12 + ila, 10);// maxst);
    EEPROM.writeShort(14 + ila, 1); // incst);
    EEPROM.writeShort(16 + ila, 1); //delay_CCS_bas);
    EEPROM.writeShort(18 + ila, 1); //delay_CCS_pow);

    ila = 20; // mos: 1*20  CurDUTclass; //0 bip, 1 mos, 2 jfet 8 diode 9 depletion ##########
    EEPROM.writeChar(1 + ila, 0);  // s_blue
    EEPROM.writeChar(2 + ila, 6);  // x_scale
    EEPROM.writeChar(3 + ila, 5);  // y_scale
    EEPROM.writeShort(4 + ila, 8); // amin);
    EEPROM.writeShort(6 + ila, 120); //amax);
    EEPROM.writeShort(8 + ila,  1); // ainc);
    EEPROM.writeShort(10 + ila,  1); //minst);
    EEPROM.writeShort(12 + ila, 1);// maxst);
    EEPROM.writeShort(14 + ila, 1); // incst);
    EEPROM.writeShort(16 + ila, 1); //delay_CCS_bas);
    EEPROM.writeShort(18 + ila, 1); //delay_CCS_pow);

    ila = 40; // fet: 2*20  CurDUTclass; //0 bip, 1 mos, 2 jfet 8 diode 9 depletion ##########
    EEPROM.writeChar(1 + ila, 0);  // s_blue
    EEPROM.writeChar(2 + ila, 6);  // x_scale
    EEPROM.writeChar(3 + ila, 3);  // y_scale
    EEPROM.writeShort(4 + ila, 0); // amin);
    EEPROM.writeShort(6 + ila, 30); //amax);
    EEPROM.writeShort(8 + ila,  1); // ainc);
    EEPROM.writeShort(10 + ila, 1); //minst);
    EEPROM.writeShort(12 + ila, 1);// maxst);
    EEPROM.writeShort(14 + ila, 1); // incst);
    EEPROM.writeShort(16 + ila, 1); //delay_CCS_bas);
    EEPROM.writeShort(18 + ila, 1); //delay_CCS_pow);

     ila = 160; // Diode 8*20  CurDUTclass; //0 bip, 1 mos, 2 jfet 8 diode 9 depletion ##########
    EEPROM.writeChar(1 + ila, 0);  // s_blue
    EEPROM.writeChar(2 + ila, 6);  // x_scale
    EEPROM.writeChar(3 + ila, 4);  // y_scale
    EEPROM.writeShort(4 + ila, 0); // amin);
    EEPROM.writeShort(6 + ila,200); //amax);
    EEPROM.writeShort(8 + ila,  0); // ainc);
    EEPROM.writeShort(10 + ila, 1); //minst);
    EEPROM.writeShort(12 + ila, 1);// maxst);
    EEPROM.writeShort(14 + ila, 1); // incst);
    EEPROM.writeShort(16 + ila, 1); //delay_CCS_bas);
    EEPROM.writeShort(18 + ila, 1); //delay_CCS_pow);

      ila = 180; // depl. 9*20  CurDUTclass; //0 bip, 1 mos, 2 jfet 8 diode 9 depletion ##########
    EEPROM.writeChar(1 + ila, 0);  // s_blue
    EEPROM.writeChar(2 + ila, 6);  // x_scale
    EEPROM.writeChar(3 + ila, 6);  // y_scale
    EEPROM.writeShort(4 + ila, 0); // amin);
    EEPROM.writeShort(6 + ila,400); //amax);
    EEPROM.writeShort(8 + ila,  1); // ainc);
    EEPROM.writeShort(10 + ila, 1); //minst);
    EEPROM.writeShort(12 + ila, 1);// maxst);
    EEPROM.writeShort(14 + ila, 1); // incst);
    EEPROM.writeShort(16 + ila, 1); //delay_CCS_bas);
    EEPROM.writeShort(18 + ila, 1); //delay_CCS_pow);

    EEPROM.commit();
  // #############################################################
}

#ifdef TOUCH
long mapi(long x, long in_min, long in_max, long out_min, long out_max)
{
  // return  (x /*- in_min*/ ) * (out_max - out_min) / (in_max - in_min) + out_min; // original map
   return out_min + x * (out_max - out_min) / (in_max - in_min);
}
#endif


#ifdef TOUCH
void getTouchCoord() {
      uint16_t x, y;
      int xi,yi;
      tft.getTouchRaw(&y, &x);
      
      if ( y > 2600) y = 2600;
      if ( y <  195) y =  195;
      if ( x > 3880) x = 3880;
      if ( x <  404) x =  404;
    
      y = 2600 - y; //162; 
      x = 3880 - x; //420;
      xi = x;
      yi = y;
   // float valx = mapf(xi, 404, 3880, 0, 319);
   // float valy = mapf(yi, 195, 2600, 0, 239);
      valx = mapi(xi, 404, 3880, 0, 319);
      valy = mapi(yi, 195, 2600, 0, 239);
  
}
#endif

void loop(void) {
  tft.setFreeFont(FF33);
  clearMenu();
  drawFunctions();             // CT or other (CCV/CCS) ...
  // tft.println(" nach clear"); delay(1000);
  // Serial.println(" loop: nach clear");
#ifdef  NAME_D
  setDevName();                // device-Name
#endif


ser1_read:

// tft.print("vor readserial nrk: ");
  s_isr32 = 0;
  int8_t class_0123 = 9,s_param=0;
  nrk = readSerial();
// tft.println(nrk);
  if (nrk > 0) {  // connection to TT: DUT found
    readParamEEPROM(CurDUTclass);
    retc = ExecSetupMenu( mini, maxi, inci);
 // clearMenu();
  }
  else {  // no connection to TT or no DUIT found
 // CurDUTclass = EEPROM.readChar(99);
 // Serial.print("Class: ");Serial.println(CurDUTclass);
 //  s_diode = 0;
 // readParamEEPROM(readChar(99));           // params from last run
  
  while (1) {
#ifdef TOUCH
  
  if (s_touchi == 1) {
    s_touchi = 0;
    uint16_t z = tft.getTouchRawZ();

    if (z > 250)  {
      getTouchCoord();   // x y between 320 and 240
   //   DrawBox(235,0,110,35, TFT_BLACK);
   //   DrawIntAt(235,14, valx, 1, TFT_WHITE);
   //   DrawIntAt(235,33, valy, 1, TFT_WHITE);
   //   DrawIntAt(280,14, z, 1, TFT_WHITE);
      if (valx > 36 and valx < 75) {                                  // select class ##########################
        if (valy < 90) {  // Transistor / Diode
       //   DrawStringAt(85,63,"Tran/Diode",1,TFT_WHITE);
         
          if (class_0123 == 9 or class_0123 == 1) class_0123 = 0;
          else  class_0123 = 1;
          dcl = class_0123;
          delay(50);
          goto newcross;
        }
        else if (valy < 170) {  // MosFet
       //   DrawStringAt(85,143,"Mosfet",1,TFT_WHITE);
          dcl = 2;
          class_0123 = 2;
          delay(20);
          goto newcross;
        }
        else if (valy > 180 and valy < 220) {  // jFet
       //   DrawStringAt(85,223,"J-Fet",1,TFT_WHITE);
          dcl = 3;
          class_0123 = 3;
          delay(20);
          goto newcross;
        }
      }
           
      else if
       (valx > 90 and valx < 125 and      // functions
        valy >  5 and valy < 160)      {  // 0=CT,1=ct+Blue,2=CT+SD,3=CT+Blu+SD,4=const.Volt.Source
        if      (valy <  30) funcy = 0;
        else if (valy <  60) funcy = 1;
        else if (valy <  90) funcy = 2;
        else if (valy < 120) funcy = 3;
        else if (valy < 160) funcy = 4; 
        if (funcy < 5) {
          paintCrossFuncy(funcy);                     // activate function 0...4 
        }
      }
      else if
        (valx > 130 and valx < 290 and
         valy > 200 and valy < 230)    {
        if      (valx < 145) funcx = 5;
        else if (valx < 188) funcx = 6;
        else if (valx < 255) funcx = 7;
        else if (valx < 285) funcx = 8;
        if (funcx < 9) {
          paintCrossFuncx(funcx);                     // activate function 5...8 
        }
      } 
      else if (valx > 95 and valx < 120 and valy > 200 and valy < 230 and funcx < 9) {
        l_func = 9;
        if      (funcyo < 5) l_func = funcyo;
        else if (funcxo < 9) l_func = funcxo;
       
        if (l_func == funcyo) {  
          DrawBox(96,  9 + funcyo*32, 15, 15, TFT_BLACK);      // no focus
          DrawBox(99, 12 + funcyo*32,  9,  9, TFT_WHITE);
        }
        if (l_func == funcxo) {  
          DrawBox(130 + (funcxo-5) * 45 + 4, 212 + 4, 15, 15, TFT_BLACK);
          DrawBox(130 + (funcxo-5) * 45 + 7, 212 + 7, 9, 9, TFT_WHITE);
       
        }
        if (l_func < 9) {
          s_blue  = l_func;
          s_param = 1;
          goto nxt_run;
        }


           
      } 
      else if (valy >= 238);
    }  // z > 1000
  }    // s_touchi

#endif
      if (s_isr32 == 1) goto nxt_run;
      if (Serial.available() > 0) goto ser1_read; //break;
      if (rota2 != 0) {
        if (rota2 > 0 )      dcl--;
        else if (rota2 < 0)  dcl++;
        if      (dcl > 3) dcl = 0;  //r2
        else if (dcl < 0) dcl = 3;  //r2
        rota2 = 0;
   newcross:     
        DrawBox( 30, CurDUTclass * 80 + 52, 18, 18, TFT_WHITE); // erase old Cross
     // Serial.print("dcl ");Serial.print(dcl);Serial.print(" CDC ");Serial.print(CurDUTclass);Serial.print(" rota2 ");Serial.println(rota2);
        CurDUTclass = 0;
        s_new = 0;
        if (dcl >= 2) {  // mosfet, jFet
          CurDUTclass = dcl - 1; //r2 new
          s_new = CurDUTclass;
          s_diode = 0;
        }  
        else {   // diode or pnp/npn
          if (dcl == 0) {
            s_diode = 1;      // paint diode curves
            s_dsign = 1;      // change icon to diode on screen
            DrawCheckBox( CurDUTclass, r_g_b(0xFF, 0xFF, 0x80)); // diode
          }
          else {              // old: diode
            s_diode = 0;      // paint transistor curves
            s_dsign = 0;      // change icon to pnp/npn on screen
            DrawCheckBox( CurDUTclass, r_g_b(0xFF, 0xFF, 0x80));   // pnp/npn
          }   
        }  // else  : diode or pnp/npn
        paintCross(); // erase old cross and place new Cross
       
     // Serial.println(dcl);
      }  // if (rota2 != 0
      if (s_isr32 == 1 or s_param == 1) { 
   nxt_run:  
        s_param = 0;   
        DrawBox(81,0,238,240,TFT_BLACK);     // erase functions-menu
       // EEPROM.writeChar(20*CurDUTclass, CurDUTclass);    // 0=bipolar,1=Mosfet,2=J-Fet
        readParamEEPROM(CurDUTclass);  //######################///
        class_o = CurDUTclass;
        retc = ExecSetupMenu( mini, maxi, inci);  //#####################<<<<<<<<<<<<<<<<<<<<<<<
        break;
      }
      
    } // while
    s_isr32 = 0;
  } // else

}




void initSD_Card() {
#ifdef SD_C
  s_isr32 = 0;
  debugln("initialization SD-Card :");
 // SPI.beginTransaction(settingsM);
  while (!SD.begin(pin_SD_CS)) {

    debugln("initialization failed. Things to check:");
    debugln(" 1. is a card inserted?");
    debugln(" 2. is your wiring correct?");
    debugln(" 3. did you change the SD_CS-pin (8?) ?");
    //   debugln("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
    DrawStringAt(178, 70, "SD inserted ?", 1, TFT_MAGENTA); // 173 SD
    if (s_isr32 == 1) { 
      s_blue = 0;
      s_isr32 = 0;
      delay(40);
      s_isr32 = 0;
      DrawStringAt(178, 70, sdi[0]/*"SD-Card inserted ?"*/, 1, TFT_BLACK);   // SD
      return;
    }  
  }

  debugln("initialization done.");
  DrawStringAt(178, 70, "SD inserted ?", 1, TFT_BLACK);   // SD
  DrawStringAt(178, 70, "SD-initialized", 1, TFT_BRIGHTBLUE);  // SD

#endif

}



void openSD_Card() {
#ifdef SD_C

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  String file_p = sdo[0]/*"sd654321.txt"*/;       // file-name preset for ESP32

  if (efnr == 1) file_p.setCharAt(1, 'P'); //0,
  else           file_p.setCharAt(1, 'N'); //0,
  if      (CurDUTclass == tcBipolar) file_p.setCharAt(2, '0'); //0=bipolar//1,
  else if (CurDUTclass == tcMOSFET)  file_p.setCharAt(2, '1'); //1=Mosfet //1,
  else if (CurDUTclass == tcJFET)    file_p.setCharAt(2, '2'); //2=j-Fet  //1,
#ifdef NAME_D
  if (l_devna > 2) {
    for (int in = 0; in < 6; in++) {
      file_p.setCharAt(in + 3, name_t[in]); //+2
    }
  }
#endif
  dataFile = SD.open(file_p, FILE_WRITE); //"datalog.txt", FILE_WRITE);
  if (dataFile) {
    DrawStringAt(178, 70, "file: ", 1, TFT_BRIGHTBLUE);  // 173 SD
    tft.print(file_p); //DrawString(file_p,1,TFT_BRIGHTBLUE);  // SD
    dataFile.close();  // test######################################################
  }
  else {     // if the file isn't open, pop up an error:
    DrawStringAt(178, 70, "file not open: ", 1, TFT_ORANGE);  // SD
    tft.print(file_p);   //DrawString(file_p,1,TFT_ORANGE);  // SD
  }

  delay(3000);

#endif

}

void drawButton_N_P( uint16_t xleft, uint16_t ytop, uint16_t dim_x, uint16_t dim_y, char n_p_tx, uint16_t col_tx, uint16_t col_back) {

  DrawBox(xleft, ytop, dim_x, dim_y, TFT_DARKGREY);
  DrawFrame(xleft, ytop, dim_x, dim_y, TFT_WHITE);
  DrawBox(xleft + 2, ytop + 2, dim_x - 4, dim_y - 4, col_back);
  DrawCharAt( xleft + 5, ytop + 27, n_p_tx, 2, col_tx);  //###size 2###

}

void drawMenuTitle() {  // draw fixed titles of the menu
  tft.setTextSize(2);
  drawButton_N_P(125, 1, 34, 34, 'P',  TFT_RED, TFT_WHITE);          // draw "P"  button
  drawButton_N_P(265, 1, 34, 34, 'N', TFT_BLUE, TFT_WHITE);          // draw "N" button
  tft.setTextSize(1);  //#1#
  DrawStringAt(88, 112, txp[0], 1, TFT_WHITE);     // " min "
  DrawStringAt(88, 162, txp[1], 1, TFT_WHITE);     // " incr"
  DrawStringAt(88, 212, txp[2], 1, TFT_WHITE);     // " max "


  //  DrawStringAt(179, 90, "Bluet:", 1, TFT_BRIGHTBLUE); //txt[3], 1, TFT_BRIGHTBLUE);  // "Bluet:"
  //  DrawStringAt(252, 90, "SD:", 1 , TFT_LIGHTRED);     //txt[4], 1, TFT_LIGHTRED);      // "SD"    // "CCS"


  DrawStringAt(179, 112, txt[3], 1, TFT_BRIGHTBLUE);   // "Bluet"
  DrawChar('/', 1, TFT_WHITE);                     // '/'"
  DrawString(txt[4], 1, TFT_LIGHTRED);                 // "CCS"
  DrawString(" :", 1, TFT_WHITE);                  // " :"
  DrawStringAt(179, 162, txt[5], 1, TFT_WHITE);    // "x-scale"
  DrawStringAt(179, 213, txt[6], 1, TFT_WHITE);    // "y-scale"


}



#ifdef NAME_D


void drawSign(int ic, int ir, uint16_t colb) {
  DrawBox(134 + ir * 17, 40, 19, 18, colb);
  DrawCharAt(135 + ir * 17, 52, ic, 1, TFT_WHITE);
  if (ir < 6) name_t[ir] = ic;
  //  if (ic != 60) DrawCharAt(135 + ir * 17, 52, ic, 1, TFT_WHITE);
}

void setDevName() {


  s_isr1  = 0;
  s_isr32 = 0;
  DrawStringAt(82, 52, "dev?:", 1, TFT_WHITE);

  while (s_isr32 == 0) {
    delay(100);
    if (s_isr1 == 1) break;
  }
  if (s_isr32 == 1) {
    s_isr32 = 0;
    DrawBox(81, 40, 46, 17, TFT_BLACK);
    return;
  }

  s_isr1 = 0;

  int ic = 65, il, ir = 0, timew = 0;  // ic = 65dec = 'A'

  int8_t tok[] = {43, 60, 32, 57};


  while (1) {
    int8_t id = 0;
    if (s_isr32 == 1) goto wait_end;
    delay(70);
    timew++;
    if (timew >= 150) goto wait_end;         // stop after 6 s ??
    if (s_isr2 == 1) {   //1?
      if (ic == 60) {                                         // ic = 60 : '<' erase last sign
        drawSign(ic, ir, TFT_ORANGE); //BLACK);
        delay(1000);
        drawSign(32, ir, TFT_BLACK); //BLACK);  // ausblanken
        //     DrawBox(134+ir*17,40,19,18,TFT_BLACK);
        ir -= 1;
        if (ir < 0) ir = 0;
        //       ic = EEPROM.readChar(101 + ir); // ??
        goto newsign;
      }
      if (ic != 62)  {                                        // ic = 62 : '>'  back to program
        drawSign(ic, ir, TFT_BLACK);
        //   DrawBox(134 + ir*17,40,19,18,TFT_BLACK);
        //   DrawCharAt(135 + ir*17,52,ic,2,TFT_WHITE);
#ifdef IEPROM      // ESP32: EEPROM.writeChar instead of update 
        EEPROM.writeChar(101 + ir, ic);                          // save new sign in EEPROM
        EEPROM.commit();
#endif
        ir++;
        if (ir > 10) goto wait_end;
      }
      else    {                                               // ic = 62 : '>' back to program
wait_end:
        s_isr32 = 0;
        l_devna = ir;
#ifdef IEPROM   // ESP32: EEPROM.writeChar instead of update      
        EEPROM.writeChar(100, ir);                               // name-length in address 60
        EEPROM.commit();
#endif
        DrawBox(81, 40, 239, 18, TFT_BLACK);              // delete dev-name-string with space 15 (with DrasCharAT(...
        return;
      }
newsign:
      //  if (ic == 60) drawSign(ic, ir, TFT_ORANGE);         // ic = 60 : '<' erase last sign
      drawSign(ic, ir, TFT_UDARKGREY); //TFT_DARKGREY);
      //  DrawBox(134 + ir*17,40,19,18,TFT_DARKGREY);
      //  DrawCharAt(135 + ir*17,52,63,2,TFT_WHITE);     // ?
      delay(800);                                             // 80 ??

      timew = 0;
      s_isr2 = 0;
    }    // isr

    if (newPos1 != oldPos1) {                                  // choose next Sign
      timew = 0;

      id = 2;
      if (newPos1 < oldPos1) ic--;
      else  {
        ic++;
        id = 0;
      }

      if      (ic < 32)             ic = 90;
      else if (ic > 32 and ic < 43) ic = tok[id];  // 43 32    int8_t tok[] = {43,60,32,57};
      else if (ic > 57 and ic < 60) ic = tok[id + 1]; // 60 57    int8_t tok[] = {43,60,32,57};
      else if (ic > 90)             ic = 32;
      drawSign(ic, ir, TFT_UDARKGREY); //TFT_DARKGREY);
      //  DrawBox(134 + ir*17,40,19,18,TFT_DARKGREY);
      //  DrawCharAt(135 + ir*17,52,ic,2,TFT_WHITE);
      oldPos1 = newPos1;
    } // if (newPos1 !=

  }  // while

}


#endif


#ifdef NAME_D

void drawDevName() {
  tft.setCursor(135, 52);
#ifdef IEPROM
  for (int8_t n = 0; n < l_devna; n++) {              // l_devna = EEPROM.readChar(100);// Length of the device-name
    DrawChar(readChar(n + 101), 1, TFT_WHITE);
  }
#endif
}
#endif




int8_t readSerial() {  // Interface to TT

  char ser1 = ' ';
  uint16_t ilz = 0;
  while (Serial.available() == 0) {
    if (ilz > 2000) return 0;  // no connection to tt or device passiv (c,r etc)
    delay(1);
    ilz++;
  }

ser1_nxt:
  ser1 = Serial.read();
  // Serial.println(ser1);  //##
  if (ser1 == '\n' or ser1 == '\r') goto ser1_nxt;
  if (ser1 == '0')  return 0;
  if (ser1 >  '9')  return 0;
  // const char* txk[10] = {" ",": bip.npn ",": n-Mosfet",": n-jFet  ",": n-Diode ",": bip.pnp ",": p-Mosfet",": p-jFet  ",": p-Diode ",": n-Depl.Mos"};
  nrk = ser1 - '0';               // ### char to int ####
  if (nrk >= 1 and nrk <= 9 ) {
    DrawIntAt(126, 50, nrk, 1, TFT_WHITE);
    DrawStringAt(135, 50, txk[nrk], 1, TFT_WHITE);
    //else         DrawStringAt(135,50,txc[4],2,TFT_WHITE);
    // Serial.println(nrk);  //##############################################
    s_diode = 0;
    s_dsign = 0;
    s_depl  = 0;
    if (nrk == 4 or nrk == 8) {
      s_diode = 1;
      s_dsign = 1;
      class_o = 0;   // diode
      curkind = nrk - 3;

      DrawCheckBox( 0, r_g_b(0xFF, 0xFF, 0x80)); // diode
    }
    else if (nrk == 9) {
      s_depl  = 1;   // depletion-mode
      class_o = 2;   // jfet
      curkind = 3;   // n-depletion Mos similar n-Jfet
      nrk = 3;
    }
    else {
      curkind = nrk;
      class_o = curkind - 1;
      if (curkind >= 5) class_o = curkind - 5;   // 0,1,2
    }
    //  DrawIntAt(126,50,nrk, 1, TFT_ORANGE);
    s_tt    = 1; // connection to TT
    s_auto  = 1;

    dcl     = class_o;  // new class
    CurDUTclass = dcl; //dcl_t;
    s_new = CurDUTclass;
    devPol = 0;
    if (nrk >= 5 and nrk <= 8) devPol = 1;

    paintCross();          // erase old Cross and place new Cross
    // Serial.print(" kind ");Serial.print(curkind);Serial.print(" class ");Serial.println(class_o);
    return nrk;
/*
    char* tam[2] = {"automat. ?", "manually ?"};
    s_isr32 = 0;
    uint8_t ilz = 0;
    while (1)      {

      for (uint8_t ic = 0; ic <= 1; ic++) {

        DrawStringAt(94, 70, tam[ic], 1, TFT_WHITE);
        delay(800);  //1200
        DrawBox(94, 58, 81, 18, TFT_BLACK);
        ilz++;
        if (ilz > 5)   return nrk;  // 5 changes : auto
        if (s_isr32 == 1) {
          s_isr32 = 0;
          if (ic == 1) s_auto  = 0;
          return nrk;
        } // if

      }   // for
    }     // while
    */
   
  }
}





void clearMenu() {

  rota2   = 0;
  s_isr1  = 0;
  s_isr2  = 0;
  s_isr32 = 0; // push-button
  s_tt    = 0;
  s_depl  = 0;
  s_first = 0;
  s_diode = 0;
  l_ch    = 4; // last change: 4= min: 
  CurDUTclass = EEPROM.read(99);  // 0=bipolar,1=Mosfet,2=jFet  
  if (CurDUTclass == 8) {  // diode similar pnp/npn
    CurDUTclass = 0;
    s_diode = 1;
  }
  if (CurDUTclass == 9) {  // depl.Mosfet similar nJfet
    CurDUTclass = 2;
    s_depl = 1;
  }
  class_o = CurDUTclass;
  dcl     = class_o;
  nrk     = 0;

  // tft.println(" vor  TurnoffLoad");delay(2000);
  // TurnOffLoad();  //#############################################
  //tft.setFont(&FreeSerif9pt7b);
  // tft.println(" nach TurnoffLoad");delay(2000);
  tft.fillScreen(TFT_BLACK);//ClearDisplay(TFT_BLACK);       //##############################################################################
  // tft.println("vor DrawCheckBox ");delay(2000);

  DrawCheckBox(0, r_g_b(0xFF, 0xFF, 0x80));  // pnp/npn / Diode
  DrawCheckBox(1, r_g_b(0x80, 0xFF, 0xFF));  // Mosfet
  DrawCheckBox(2, r_g_b(0xFF, 0x80, 0xFF));  // jFet
  s_new = CurDUTclass;
  s_old = 0;
  paintCross();
  

}

void drawBox_xy(uint16_t x, uint16_t y) {
    DrawBox(x, y, 22, 22, TFT_WHITE); // erase old cross 28, yo=old*80+50, 50, yo+22
    //DrawFrame( x, y, 23, 23, TFT_WHITE);
    DrawFrame( x + 1, y + 1, 23 - 2, 23 - 2, TFT_BRIGHTBLUE);
    DrawFrame( x + 2, y + 2, 23 - 4, 23 - 4, TFT_BLUE);
}

void drawFunctions() {

// 
    uint8_t y = 20;
    DrawStringAt(130, y, txf[0], 1, TFT_LIGHTGREY);  // "CurveTracer"
    drawBox_xy(92,y-15);
    y += 32;
    DrawStringAt(130, y, txf[1], 1, TFT_LIGHTGREY);  // "Curve+Bluetooth"
    drawBox_xy(92,y-15);
    y += 32;
    DrawStringAt(130, y, txf[2], 1, TFT_LIGHTGREY);  // "Curve+SD-Card"
     drawBox_xy(92,y-15);
    y += 32;
    DrawStringAt(130, y, txf[3], 1, TFT_LIGHTGREY);  // "Curve+Bluet.+SD-Card"
     drawBox_xy(92,y-15);
    y += 32;
    DrawStringAt(130, y, "const.Voltage-Source", 1, TFT_GREEN);  // "const.Voltage"
     drawBox_xy(92,y-15);
    y += 32;
    
    DrawFrame(124, 165, 180, 74, TFT_WHITE);
    DrawStringAt(130, y, "const.Current-Source", 1, TFT_RED);  // "const.Current"
    y += 20;  // 200
    DrawStringAt(130, y, "==,up/down,down, up", 1, TFT_WHITE);  // 
    drawBox_xy(130,y+12);  // permanent
    drawBox_xy(175,y+12);  // up/down
    drawBox_xy(220,y+12);  // down 
    drawBox_xy(265,y+12);  // up
    DrawBox(83, 200, 36, 36, TFT_WHITE);
    DrawFrame(85, 202, 32, 32, TFT_RED);
    DrawFrame(86, 203, 30, 30, TFT_RED);
    DrawFrame(87, 204, 28, 28, TFT_RED);
    tft.drawLine(87,204,115,232,TFT_RED);
    tft.drawLine(87,232,115,204,TFT_RED);
    

}



#ifdef IEPROM
void readParamEEPROM(int8_t l_cur) {
  if      (s_diode == 1) l_cur = 8;
  else if (s_depl  == 1) l_cur = 9;
  int ilv = l_cur * 20; //CurDUTclass*20;
  if (s_blue < 9) s_blue = s_blue;  // from functions-menu  
  else s_blue   = EEPROM.readChar(ilv + 1); //  0,1,2  3,4,5  8,9  ?#?#?#
  cxs           = EEPROM.readChar(ilv + 2); // x_scale
  cys           = EEPROM.readChar(ilv + 3); // y_scale
  mini          = EEPROM.readShort(ilv + 4);
  maxi          = EEPROM.readShort(ilv + 6);
  inci          = EEPROM.readShort(ilv + 8);
  minst         = EEPROM.readShort(ilv + 10);
  maxst         = EEPROM.readShort(ilv + 12);
  incst         = EEPROM.readShort(ilv + 14);
  delay_CCS_bas = EEPROM.readShort(ilv + 16);
  delay_CCS_pow = EEPROM.readShort(ilv + 18);
  x_scale       = xs_t[cxs];
  y_scale       = ys_t[cys];

}


int readShort(int address)
{
  byte byte1 = EEPROM.readChar(address);
  byte byte2 = EEPROM.readChar(address + 1);
  return (byte1 << 8) + byte2;
}

#endif


void drawVar(int x, int y, int l, int h, int xb, int yb, int var) {

  DrawBox(    x,   y, l, h, back_col);
  DrawIntAt(xb, yb, var, 1, text_col);
}

void paintCrossFuncy(int16_t nr) {  // function 0,1,2,3,4
   int16_t xl =  92;
   int16_t yo =   5;
   if (funcyo < 5) {
     DrawBox(xl + 2, yo + 2 + funcyo*32 , 18, 18, TFT_WHITE); // erase old cross
   }  
   if (funcxo < 9) {
     int16_t xf = 0;
     xf = (funcxo-5) * 45; 
     DrawBox(130 + xf + 2, 212 + 2 , 18, 18, TFT_WHITE);     // erase old cross x
     funcxo = 9;
   }
   DrawBox(xl + 4, yo + 4 + nr*32, 15, 15, TFT_RED);
   DrawBox(xl + 7, yo + 7 + nr*32, 9, 9, TFT_WHITE);
   funcyo = nr;
   /*
   if (nr <= 4) {
     for (int16_t nrx=0;nrx<=4;nrx++) {
       if (nrx != nr)  { // erase
         DrawBox(xl + 2, yo + 2 + nrx*32 , 18, 18, TFT_WHITE); // erase old cross
       }
       else {
         DrawBox(xl + 4, yo + 4 + nrx*32, 15, 15, TFT_RED);
         DrawBox(xl + 7, yo + 7 + nrx*32, 9, 9, TFT_WHITE);
       }
     }
   }
   */
}


void paintCrossFuncx(int16_t nr) {  // function 5,6,7,8
   int16_t xl = 130;
   int16_t yo = 212;
   int16_t xf = 0;
   if (funcxo < 9) {
     xf = (funcxo-5) * 45; 
     DrawBox(xl + xf + 2, yo + 2 , 18, 18, TFT_WHITE); // erase old cross
   }  
   if (funcyo < 5) {
     DrawBox(92 + 2, 5 + 2 + funcyo*32 , 18, 18, TFT_WHITE); // erase old cross
     funcyo = 9;
   }    
   xf = (nr-5) * 45; 
   DrawBox(xl + xf + 4, yo + 4, 15, 15, TFT_RED);
   DrawBox(xl + xf + 7, yo + 7, 9, 9, TFT_WHITE);
   funcxo = nr;
  /* if (nr <= 8) {
     for (int16_t nrx=5;nrx<=8;nrx++) {
       xf = (nrx-5) * 45; 
       if (nrx != nr)  { // erase
         DrawBox(xl + xf + 2, yo + 2 , 18, 18, TFT_WHITE); // erase old cross
       }
       else {
         DrawBox(xl + xf + 4, yo + 4, 15, 15, TFT_RED);
         DrawBox(xl + xf + 7, yo + 7, 9, 9, TFT_WHITE);
       }
     }
   }*/
}
void paintCross() {    // xl,yo, xr,yu
  int16_t xl = 28;
  int16_t xr = 50;
  int16_t yo = s_old * 80 + 50;
  int16_t yu = s_old * 80 + 72;

  DrawBox(xl + 2, yo + 2, 18, 18, TFT_WHITE); // erase old cross
  // if (s_old == s_new) return;
  s_old = s_new;
  yo = s_new * 80 + 50;
  yu = s_new * 80 + 72;
  DrawLine( xl, yo, xr, yo + 22, TFT_BLACK);
  DrawLine( xl, yu, xr, yo + 22, TFT_BLACK);
  DrawBox(xl + 4, yo + 4, 15, 15, TFT_RED);
  DrawBox(xl + 7, yo + 7, 9, 9, TFT_WHITE);


}


//-------------------------------------------------------------------------
// DrawCheckBox with BMP Icons or without
//   draw a CheckBox on the main menu screen
//-------------------------------------------------------------------------


void DrawCheckBox( int8_t cur_cl, uint16_t color) {

  const int WH      = 79; //
  const int BoxWH   = 23; //
  //const int BoxL  = 28; //
  //const int BoxT  = 50; //
  int y = 0;
  if (cur_cl > 0) y = 80 * cur_cl;  // 80, 160
  DrawFrame(0, y, WH, WH, TFT_WHITE);
  DrawBox(2, y + 2, WH - 4, WH - 4, color);
  DrawFrame(1, y + 1, WH - 2, WH - 1, TFT_BLACK);
  DrawBox(28, y + 50, 22, 22, TFT_WHITE); // erase old cross 28, yo=old*80+50, 50, yo+22
  // DrawBox( 28 + 4, y + 50 + 4, BoxWH - 5, BoxWH - 5, TFT_WHITE);
  DrawFrame( 28, y + 50, BoxWH, BoxWH, TFT_BLACK);
  DrawFrame( 28 + 1, y + 50 + 1, BoxWH - 2, BoxWH - 2, TFT_BLACK);
  //  if (checked) paintCross( 28, y + 50, 28 + BoxWH - 1, y + 50 + BoxWH - 1);

  int16_t ibl = 11;

  if (y == 0) {        // pnp/npn or Diode

    if (s_diode == 1)  DrawStringAt(16, y + 18, txc[4], 1, TFT_BLACK);  // Diode
    else               DrawStringAt( 9, y + 18, txc[0], 1, TFT_BLACK);  // npn/pnp
#ifdef BMP
    drawBMP_Pair(8, y);
#endif
    return;
  }
  if (y == 80) {     // Mosfet
    DrawStringAt(7, y + 18, txc[1], 1, TFT_BLACK);
#ifdef BMP
    drawBMP_Pair(8, y);
#endif
    return;
  }
  else  {            // JFet
    if (s_depl == 0)  DrawStringAt(17, y + 18, txc[2], 1, TFT_BLACK);
    else              DrawStringAt(17, y + 18, txc[3], 1, TFT_BLACK);
#ifdef BMP
    drawBMP_Pair(8, y);
#endif
    return;
  }
}


#ifdef BMP

void drawBMP_Pair(int16_t x, int16_t y) {
  int16_t yp = y + 21;
  if (y == 0) {      // pnp / npn trans. or diode
    if (s_diode == 1) {
      DrawBitmapMono(    x, yp, bmpPDiodeBig, TFT_BLACK);
      DrawBitmapMono( x + 39, yp, bmpNDiodeBig, TFT_BLACK);
      return;
    }
    DrawBitmapMono(    x, yp, bmpPNP, TFT_BLACK);
    DrawBitmapMono( x + 39, yp, bmpNPN, TFT_BLACK);
  }
  if (y == 80) {     // Mosfet
    DrawBitmapMono(    x, yp, bmpPMOSFET, TFT_BLACK);
    DrawBitmapMono( x + 39, yp, bmpNMOSFET, TFT_BLACK);
  }
  if (y == 160) {    // jFet
    DrawBitmapMono(    x - 3, yp, bmpPJFET, TFT_BLACK);
    DrawBitmapMono( x + 39 + 6, yp, bmpNJFET, TFT_BLACK);
  }
}

#endif







int8_t blinkFrame(uint8_t time_nr) {   // Blinking frame
  s_isr2  = 0;
  s_isr32 = 0;
  uint8_t istep = 1;
  if (efnr == 0) istep = 0;  // bat-empty long-time blinking
  for (uint8_t il = 0; il < time_nr; il = il + istep) { // time_nr * 100 ms waiting for push-button pressed to go back

    if (s_isr32 == 1) {  // button-switch pressed
      s_isr32 = 0;
      s_isr2  = 1;       // simulation encoder2-rotation
    }
    if (s_isr2 == 1)    return 1;  //1?
    drawEdge(efnr, 7);   // red frame
    delay(100);
    drawEdge(efnr, 1);   // cyan
    delay(50);
  }
  return 0;  //1?

}


void   drawOldParam() {    // from EEPROM
  back_col = TFT_BLACK;
  text_col = TFT_WHITE;

  if (s_blue >= 1) {
    back_col = TFT_WHITE;
    text_col = TFT_BLUE;
    if (s_blue > 3) text_col = TFT_RED;          // 1 sd_c
    // if (s_blue > 5) s_stair = 1; // staircase-gen // 3 sd_c
  }

  // s_pos = 1;  // : get only erase and draw-positions
  newParPos(3, 0); //drawEdge(3,1); // get only erase/draw-positions    3=blueth pos
  drawVar(xa + 2, ya + 2, wa - 4, ha - 4, xa + 6, ya + 14, s_blue); // bluet #################?#?#?#?#
  back_col = TFT_BLACK;
  text_col = TFT_WHITE;
  newParPos(4, 0); // amin pos
  drawVar(xa + 1, ya + 2, wa - 4, ha - 3, xa + 5, ya + 16, mini); //*amin);    // *amin
  newParPos(5, 0); // ainc pos
  drawVar(xa + 1, ya + 2, wa - 4, ha - 3, xa + 5, ya + 16, inci); //*ainc);     // *ainc
  newParPos(6, 0); // amax pos
  drawVar(xa + 1, ya + 2, wa - 4, ha - 3, xa + 5, ya + 16, maxi); //*amax);     // *amax
  drawPar_7_8();    // x_scale,y_scale or starcase step-time #######################################

}


void drawPar_7_8() {

  newParPos(7, 0);                                                         // x-scale pos
  drawVar(xa + 1, ya + 2, wa - 4, ha - 3, xa + 5, ya + 16, x_scale);       // xscale
  if (s_blue > 5) drawVar(xa + 1, ya + 2, wa - 4, ha - 3, xa + 5, ya + 16, delay_CCS_bas); // staircase bas // 3 sd_c
  newParPos(8, 0);                                                         // y-scale pos
  drawVar(xa + 1, ya + 2, wa - 2, ha - 3, xa + 5, ya + 16, y_scale);       // yscale
  if (s_blue > 5) drawVar(xa + 1, ya + 2, wa - 2, ha - 3, xa + 5, ya + 16, delay_CCS_pow); // staircase pot // 3 sd_c
  else fui = 240.0 / (float)y_scale;               // 2mA:120, 5mA:48, 10mA=24, 20mA:12; 50mA:4.8; 100mA:2.4;                                     // ###
}


//-------------------------------------------------------------------------
// ExecSetupMenu
//   draw and executes the setup menu screen
//   returns true if a DUT is inserted
//-------------------------------------------------------------------------

bool ExecSetupMenu( int amin, int amax, int ainc) { //, int valMax, int valInc) {

  int x, y;
  int8_t cbl = 0;

  tft.setFreeFont(FF33);//tft.setFont(&FreeSerif9pt7b);
  s_stop = 0;
  s_zen  = 0;
  int8_t class_npr = 9;
  
// s_9 = 0;
#ifdef NAME_D
  l_devna = EEPROM.readChar(100);           // Length of the name at 100
  if (l_devna > 0) drawDevName();       // in test
#endif
  DrawBox(170, 13, 83, 19, TFT_BLACK);  // 60
  if (s_diode == 1) DrawStringAt(172, 25, txc[4], 1, TFT_YELLOW);
  else              DrawStringAt(172, 25, txc[CurDUTclass], 1, TFT_YELLOW);       // str1 //" J-FET "  // 40
  DrawBox(83, 61, 115, 20, TFT_BLACK);
  if (s_diode == 0) DrawStringAt(85, 75, txt[CurDUTclass], 1, TFT_WHITE);         // str2 //"I-Base [µA]","V-Gate ", //I-Diode
  else              DrawStringAt(85, 75, txt[8], 1, TFT_WHITE);
  if (CurDUTclass == 0 and s_diode == 0) drawMyL(85 + 8 * 7, 69, TFT_WHITE);      // bipolar: µ = u + Myl()
  tft.setTextSize(1);
  drawMenuTitle();  // header and titles
  drawOldParam();   // from EEPROM
#ifdef TOUCH
    DrawBox(184, 78, 135, 18, TFT_BLACK);
    DrawStringAt(185, 90, txf[s_blue], 1, text_col);
    newParPos(3, 0); //drawEdge(3,1); // get only erase/draw-positions    3=blueth pos
    drawVar(xa + 2, ya + 2, wa - 4, ha - 4, xa + 6, ya + 14, s_blue); // 
#endif
  efnr  = l_ch - 1; // 3;
  tft.setTextSize(1);
  drawEdge(efnr, 1); // cyan
  
start_0:

 // efnr  += 1;  // starts with last ?
  uint8_t r_touch = 0;  
  
  long time_s;
  s_isr2 = 1;
  rota2 = 0;

  if (s_auto == 1) {    //
    if (devPol == 1) efnr = 1;
    else             efnr = 2;
   // Serial.print("auto ");Serial.println(efnr);
    goto ret_1_2;
  }
  s_isr32 = 0;
  s_isr1  = 0;
  s_isr2  = 0;  //??new
  rota2   = 0;  //??new
  delay(10);
  s_isr32 = 0;
  s_touchi = 0;
  while (s_isr1 == 0) {                   // scroll threw parameter-fields
  
    if (s_touchi == 1) {
      s_touchi = 0;
      uint16_t z = tft.getTouchRawZ();

      if (z > 250)  {
        getTouchCoord();   // x y between 320 and 240
      // DrawBox(110,45,200,20, TFT_BLACK); 
        if (valy > 8 and valy < 50)   {
          if (valx > 110 and valx < 150) {       // start P-device
         // DrawStringAt(125,50," P ",1,TFT_WHITE);
            efnr = 1;
            r_touch = 1;
            s_tt = 1;   // no waiting for exceute
            delay(500);
            goto ret_1_2;
          }
          else if (valx > 170 and valx  < 240) { // back to menu class-select
       // DrawStringAt(180,50,"class ?",1,TFT_WHITE);
            efnr = 9;
            r_touch = 1;
            delay(500);
            goto ret_9;
          }
          else if (valx > 280 and valx < 310)  {  // start N-device
         // DrawStringAt(270,50," N ",1,TFT_WHITE);
            efnr = 2;
            r_touch = 1;
            s_tt = 1;   // no waiting for exceute
            delay(500);
            goto ret_1_2;
          }
    
        }
        else if (valy > 90 and valy < 215) {
          if (valx > 90 and valx < 150) { 
            if (valy > 95 and valy < 120)  {
              efnr = 4;
              r_touch = 1;
              s_tt = 1;   // no waiting for exceute
              delay(100);
              goto param_4_8;  
            } 
            else if (valy > 155 and valy < 180)  {
              efnr = 5;
              r_touch = 1;
              s_tt = 1;   // no waiting for exceute
              delay(100);
              goto param_4_8;  
            }
            else if (valy > 200 and valy < 230)  {
              efnr = 6;
              r_touch = 1;
              s_tt = 1;   // no waiting for exceute
              delay(100);
              goto param_4_8;  
            }
          }
      
          else if (valx > 190 and valx < 315) {
            if (valy > 155 and valy < 180)  {
              efnr = 7;
              r_touch = 1;
              s_tt = 1;   // no waiting for exceute
              delay(100);
              goto param_4_8;  
            }
            else if (valy > 200 and valy < 230)  {
              efnr = 8;
              r_touch = 1;
              s_tt = 1;   // no waiting for exceute
              delay(100);
              goto param_4_8;  
            }
          }
        }
      }  
    }

    time_s = millis();
    if (rota2 != 0) {
     
      if      (rota2 > 0)   efnr++;
      else if (rota2 < 0)   efnr--;
      if (efnr >  9)  efnr = 1;
      if (efnr <  1)  efnr = 9;
 
      if (efnr == 3)  efnr = 4;   // efnr == 3 here not changeable
   

 param_4_8:
      rota2 = 0;
      
      drawEdge(efnr, 1);                     // cyan
  //    DrawBox(100, 40, 60, 18,TFT_UDARKGREY);
  //    DrawIntAt(100,53,newPos1  ,1,TFT_WHITE); //
  //    DrawIntAt(150,53,efnr,1,TFT_WHITE); //
      back_col = TFT_BLACK;
      uint16_t back_colp = TFT_WHITE;
      uint16_t back_coln = TFT_WHITE;
      uint16_t lett_colp = TFT_RED;
      uint16_t lett_coln = TFT_BLUE; 
      if      (efnr == 1) {
        back_colp = TFT_LIGHTRED;                 // "P"      button pressed 
        lett_colp = TFT_WHITE;
      }
      else if (efnr == 2) {
        back_coln = TFT_LIGHTBLUE;                // "N"      button pressed 
        lett_coln = TFT_WHITE;
      }
      drawButton_N_P(125, 1, 34, 34, 'P',  lett_colp, back_colp); // draw "P" button
      drawButton_N_P(264, 1, 34, 34, 'N',  lett_coln, back_coln); // draw "N" button
        
      delay(200);  // 400 test
    }  // rota2 != 0
    if (s_tt == 1 and millis() - time_s > 4000) goto ret_1_2;  // start run
    if (s_isr32 == 1)    {
      r_touch = 0;
      if      (efnr == 9)              goto ret_9;      // return to select class menu
      else if (efnr == 1 or efnr == 2) goto ret_1_2;    // start curves
    //  else if (efnr >= 4)              goto field_4;    // **test**
      else                             goto sel_3;      // select SD,Bluetooth,CCS ...
    }
  } // while (s_isr1 == 0 
  s_isr1 = 0;
  drawEdge(efnr, 2);                 //red    s_pos = 2
 // DrawBox(200, 40, 60, 18,TFT_UDARKGREY);
 // DrawIntAt(200,53,newPos1  ,1,TFT_WHITE); //
 // DrawIntAt(250,53,efnr,1,TFT_WHITE); //
  time_s = millis();
  
  oldPos1 = newPos1+2;
  if (efnr == 9) { // or s_9 == 1) {                        // go back to select bipolar,Mosfet,J-Fet
ret_9:
 // s_9     = 0;
    s_isr32 = 0;
 // DrawBox(100, 40, 60, 18,TFT_UDARKGREY);
 // DrawIntAt(100,53,newPos1  ,1,TFT_ORANGE); //
 // DrawIntAt(150,53,efnr,1,TFT_ORANGE); //
 // delay(4000);
 // Serial.print("== 9 efnr ");Serial.println(efnr);
 
   if (r_touch == 1) {
      s_isr32 = 0;
      s_isr2  = 0;
      r_touch = 0;
      return true;            
   }
 
    if (blinkFrame(20) == 1 ) {    // 3s = 20 * 150 ms break with isr2,isr32
 
      delay(5);
      s_isr32 = 0;
      s_isr2  = 0;
     
      return true;                 // return true; // wait max 2s = 20*100 ms button pressed
    }
    
  }
  
  if (efnr == 1 or efnr == 2) {   // 1='P', 2='N' run curves
 // Serial.print("1 or 2 efnr ");Serial.println(efnr);
ret_1_2:
    delay(20);
    s_touchi = 0;
   // Serial.print("ret_1_2 efnr ");Serial.println(efnr);
    if (rota2 != 0 and s_auto == 1) {
      s_auto = 0;
      goto start_0;
    }
    s_isr32 = 0;
    s_isr2 = 0;
    if (blinkFrame(10) == 1 or s_tt == 1)   {          // wait max 1.5s = 10*150ms button pressed
      s_isr32 = 0;
      s_isr2  = 0;
      if (s_blue > 3)  { // 1 sd_c
        if (s_blue == 4) CurDUTclass = 3; // const.volt.source 2 sd_c
        else             CurDUTclass = 4; // const.curr.source/staircase
      }
      else {             // 0 or 1:  curve-tracer
        if (s_sdcard == 1)  openSD_Card();  // open file sd-card

      }
      if      (s_diode == 1) CurDUTclass = 8;
      else if (s_depl == 1)  CurDUTclass = 9;
      EEPROM.writeChar(99, CurDUTclass);     // 0, 1, 2,  3, 4, 8, 9
      int ilv = 20 * CurDUTclass;         // 0,20,40, 60,80

      EEPROM.writeChar(ilv, CurDUTclass);    // 0=bipolar,1=Mosfet,2=J-Fet
      EEPROM.writeChar(1 + ilv, s_blue);
      EEPROM.writeChar(2 + ilv, cxs); // x_scale);
      EEPROM.writeChar(3 + ilv, cys); // y_scale);
      EEPROM.writeShort(4 + ilv, amin);
      EEPROM.writeShort(6 + ilv, amax);
      EEPROM.writeShort(8 + ilv, ainc);
      EEPROM.writeShort(10 + ilv, minst);
      EEPROM.writeShort(12 + ilv, maxst);
      EEPROM.writeShort(14 + ilv, incst);
      EEPROM.writeShort(16 + ilv, delay_CCS_bas);
      EEPROM.writeShort(18 + ilv, delay_CCS_pow);

      EEPROM.commit();

      if (CurDUTclass == 8 ) {
        CurDUTclass = 0;
        s_diode = 1;
      }
      if (CurDUTclass == 9 ) {
        CurDUTclass = 2; // n-depletion Mos similar n-Jfet
        s_depl = 1;
      }

      if (efnr == 1) curkind = CurDUTclass + 5; // "P" Button 5,6,7,8
      else           curkind = CurDUTclass + 1; // "N" Button 1,2,3,4

      s_auto = 0;
      ScanKind(curkind);            //#####################################
 
      s_isr2 = 0;
      s_touchi = 0;
      delayMicroseconds(10);
      return true;
    }
    else s_auto = 0;
 
  }
 
  if (efnr == 3) {   // bluetooth / CCS
 sel_3:   
    char bl_t[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};  // 0=CT,1=CT+Bl,2=CT+SD,3=CT+BT+SD, 4...8
    char cbc, old_b = s_blue; // cbl 0;
    s_isr2 = 0;  // s_isr1 = 0;
    while (s_isr2 == 0) { // s_isr1 == 1

      if (rota1 != 0) { // (newPos1 != oldPos1) {
        if (rota1 == 1) cbl++; //(newPos1 > oldPos1) cbl += 1;
        else            cbl--; //            cbl -= 1;
        rota1 = 0;
        if (cbl < 0) cbl = 8;  // sd_c: 8
        if (cbl > 8) cbl = 0;  // sd_c: 8
        s_blue = bl_t[cbl];
      //  delay(180);  // 50
        text_col = TFT_WHITE;
        if (s_blue <= 3) {      // normal TCT/KS
          back_col = TFT_BLACK;
          text_col = TFT_WHITE;
          if (s_blue == 1 or s_blue == 3) text_col = TFT_BRIGHTBLUE;

          // DrawBox(184, 75, 134,18, TFT_UDARKGREY);
          // DrawStringAt(185, 90, txf[s_blue], 1, TFT_WHITE);
#ifdef SD_C
          s_sdcard = 0;
          if (s_blue == 2 or s_blue == 3) s_sdcard = 1;
#endif
        }
        else  { // TCT/KS with Bluetooth(==1) or CCS (==4) or Staircase==5,6,7 or ConstVolt==8 )

          back_col = TFT_BLACK;//LIGHTGREY; //WHITE;
          text_col = TFT_WHITE;//GREEN; // 2=ConstVolt.,3==CCS-fixed ,4,5,6== Staircase(4:only up,6:up/down,5:up/down+/-
        }

        DrawBox(184, 78, 135, 18, TFT_BLACK);
        DrawStringAt(185, 90, txf[s_blue], 1, text_col);
        newParPos(3, 0); //drawEdge(3,1); // get only erase/draw-positions    3=blueth pos
        drawVar(xa + 2, ya + 2, wa - 4, ha - 4, xa + 6, ya + 14, s_blue); //
        
      } // rota1 != 0
    }   // while isr2 == 0 
    if (old_b != s_blue) {
      if (old_b <= 3 and s_blue <= 3);  // 1 sd_c
      else {
        if      (s_blue <= 3) cbc = CurDUTclass; // 1 sd_c
        else if (s_blue == 4) cbc = 3;           // 2 sd_c
        else                  cbc = 4;
    
        readParamEEPROM(cbc);  // CurDUTclass
        drawOldParam();
        newParPos(3,0); //drawEdge(3,1); // get only erase/draw-positions    3=blueth pos
        drawVar(xa+2,ya+2,wa-4,ha-4,xa+6,ya+14,s_blue); //
      }
      old_b = s_blue; // ??
    }  // if old
    
    //
    if (s_sdcard == 1)    initSD_Card();
    drawEdge(efnr, 4);     // erase frame
    text_col = TFT_WHITE;
    back_col = TFT_BLACK;
    if (s_blue > 5 and s_blue < 8) drawPar_7_8();   // 3 sd_c draw Param x_scale,y_scale or staircase-times
    l_ch = 3;
   
  }
 field_4: 
  if (efnr == 4) {    // min  ##############################################// test##
    s_isr2 = 0;
    para = amin;          // virtual  field 10
    paro = amin;          // original field  6
    updateParam(efnr);    // min-step  // test##
    amin = para;
    iw    = 0;
    l_ch  = 4; //##
    drawEdge(efnr, 4);   // erase frame
   
  }
  if (efnr == 5) {    // inc-  ##############################################// test##
    s_isr2 = 0;
    para = ainc;          // virtual  field 10
    paro = ainc;          // original field  6
    updateParam(efnr);     // inc-step  // test##
    ainc = para;
    iw    = 0;
    l_ch  = 5; //##
    drawEdge(efnr, 4);   // erase frame
    
  }
  if (efnr == 6) {    // max-  ##############################################// test##
    s_isr2 = 0;
    para = amax;           // virtual  field 10
    paro = amax;           // original field  6
    updateParam(efnr);     // max-step  // test##
    amax = para;
    iw    = 0;
    l_ch = 6; //##
    drawEdge(efnr, 4);     // erase frame
  
  }

  if (efnr == 7) {     // x-scale
    //int8_t xs_t[] = {0,1,2,3,4,6,12};
    uint8_t cmax = 6, cmix = 1;

    if (s_blue > 5) cmax = 10;        // 3 sd_c ccs: 4, staircase: 5,6 delay-base
    s_isr2 = 0;
    s_isr1 = 1;
    
    while (s_isr1 == 1) {       //?1

      if (newPos1 != oldPos1) {
        if (newPos1 > oldPos1) cxs += 1;
        else                   cxs -= 1;
        if (cxs < cmix) cxs = cmax;
        if (cxs > cmax) cxs = cmix;
        if (s_blue > 5) {    // 6 sd_c ccs: 6, staircase: 7,8 delay-base
          delay_CCS_bas = cxs;
          drawVar(xa + 1, ya + 2, wa - 4, ha - 3, xa + 5, ya + 16, delay_CCS_bas);
        }
        else {
          x_scale = xs_t[cxs];
          drawVar(xa + 1, ya + 2, wa - 4, ha - 3, xa + 5, ya + 16, x_scale);
        }
        oldPos1 = newPos1;

        delay(30);
      }
      if (s_isr2 == 1) s_isr1 = 0;
    }  // while
    l_ch = 7; //##
    drawEdge(efnr, 4);    // erase frame
   
  }
  if (efnr == 8) {   // y-scale
    // int8_t ys_t[] = {0,2,5,10,20,50,100};
    unsigned int pow_t[] = {0, 1, 10, 100, 1000}; // power base 10
    int8_t cmiy = 1, cmay = 6;
    //  cys  = 1;
    cmiy = 1;
    cmay = 6;
    if (s_blue > 5) cmay = 4; // 3 sd_c ccs: 6, staircase: 7,8 delay-base

    s_isr1 = 1;
    s_isr2 = 0;
    while (s_isr1 == 1) {   //?1

      if (newPos1 != oldPos1) {
        if (newPos1 > oldPos1)   cys += 1;
        else                     cys -= 1;
        if (cys < cmiy) cys = cmay;
        if (cys > cmay) cys = cmiy;
        if (s_blue > 5) {    // 3 sd_c ccs: 6, staircase: 7,8 delay-base
          if (cys == 0) cys = 1; // 1,10,100,1000
          delay_CCS_pow = pow_t[cys];  // 1,10,100
          drawVar(xa + 1, ya + 2, wa - 2, ha - 3, xa + 5, ya + 16, delay_CCS_pow); //delay_CCS_pow);
        }
        else {
          y_scale = ys_t[cys];
          drawVar(xa + 1, ya + 2, wa - 2, ha - 3, xa + 5, ya + 16, y_scale);
        }
        oldPos1 = newPos1;
        delay(30);
        fui = 240.0 / (float)y_scale; //###
   
      }
      if (s_isr2 == 1) s_isr1 = 0;
    }  // WHILE
    drawEdge(efnr, 4);   // erase frame
    rota2 = 0;      //##############
    l_ch = 8; //##
   
  }  

  goto start_0;
  
}




void ex_bluepar() {

  int8_t bl_t[] = {0, 1, 2, 3, 4, 5, 6};
  int8_t cbl = 0;
  s_isr1 = 1;
  while (s_isr1 == 1) {

    if (newPos1 != oldPos1) {
      if (newPos1 > oldPos1) cbl += 1;
      else                   cbl -= 1;
      if (cbl < 0) cbl = 6;
      if (cbl > 6) cbl = 0;
      s_blue = bl_t[cbl];
      if (s_blue == 0) {      // normal TCT/KS
        back_col = TFT_BLACK;
        text_col = TFT_WHITE;
      }
      else  { // TCT/KS with Bluetooth(==1) or CCS (==2) or Staircase==3,4,5 or ConstVolt==6 )
        back_col = TFT_WHITE;
        if (s_blue == 1 or s_blue == 3) text_col = TFT_BLUE; // TCT/KS with Bluetooth
        else             text_col = TFT_BLUE; // 2=ConstVolt.,3==CCS-fixed ,4,5,6== Staircase(4:only up,6:up/down,5:up/down+/-
      }

      drawVar(xa + 2, ya + 2, wa - 4, ha - 4, xa + 6, ya + 14, s_blue); //
      oldPos1 = newPos1;

      delay(30);
    }
  }  // while

  drawEdge(efnr, 4);     // erase frame
  text_col = TFT_WHITE;
  back_col = TFT_BLACK;
  if (s_blue > 5 and s_blue < 8) drawPar_7_8();    // 3 sd_c draw Param x_scale,y_scale or staircase-times

}


void updateParam(int8_t pnr) {  // update parameter min,inc,max und mist,inst,mast
  int8_t pnro = pnr, pnrd=pnr-4;
  int incdec = 1, parm = 150; // 150= 15V V-gate
  if      (pnr == 4) incdec = minst;
  else if (pnr == 5) incdec = incst;
  else if (pnr == 6) {
    incdec                  = maxst; // test: 10  maxst;  // 10 ################################
    if (s_blue < 4) parm =  500;  // 2 sd_c for instance ib-max/ Udmax
    else            parm = 4000;  // ccs limit threw 10k - resistor
  }


  if (pnr >= 10) {
    pnr  = pnr - 6;  // 10,11,12 same pos as 4,5,6
    pnrd = pnr - 1;  // 3,4,5
  }

  newParPos(pnr, 0); //drawEdge(pnr,1);               // only draw-pos
  DrawBox(xa - 41, ya + 2, wa + 1, ha - 4, TFT_BLACK);
  // DrawBox(xa+2,ya+2,wa-10,ha-4,TFT_UDARKGREY);
  // tft.setTextSize(2);
 // tft.setCursor(88, ya + 14); tft.setTextColor(TFT_WHITE); tft.print(txp[pnrd]);
 
  DrawStringAt(88, ya + 14, txp[pnrd], 1, TFT_WHITE);
  xo = xa; yo = ya; wo = wa - 4, ho = ha - 3;
  drawVar(xa + 1, ya + 2, wo, ho, xa + 5, ya + 16, para);

  oldPos1 = newPos1;
  s_isr1 = 1;
  s_touchi = 0;
  while (s_isr1 == 1) {
    delay(10);
    if (s_touchi == 1) {
      s_touchi = 0;
      s_isr1 = 0;
      return;
    }
    if (s_isr2 == 1) s_isr1 = 0; //##############################################
    if (s_isr1 == 0) {  //?1
      s_isr2 = 0; //########################################
      if (s_diode == 1) {  // new test
        if (pnr == 5) {
          if (para > 0 and s_zen == 99) 
            s_zen = para;
          else s_zen = 0; 
        }   
     // Serial.print("s_zen: ");Serial.println(s_zen);
      }  
      
  
      return;
      
      if (pnro < 10) return;                              // ?? 10 ###############################################  <<<<<<<<<<<<<<<
      //?????
      DrawBox(xa - 41, ya + 2, wa + 1, ha - 4, TFT_UDARKGREY);
      DrawBox(xa + 2, ya + 2, wa - 10, ha - 4, TFT_UDARKGREY);
      tft.setCursor(88, ya + 14); tft.setTextColor(TFT_WHITE); tft.print(txp[pnrd - 3]);
      // DrawStringAt(88, ya + 14, txp[pnrd - 3], 1, TFT_WHITE); // min.icr,max
      drawVar(xo + 1, yo + 2, wo, ho, xo + 5, yo + 16, paro);
      delay(30);
     
      // s_isr = 0;  //??
      return;
    }
 
    if (s_diode == 1) {  // new test
      if (pnr == 5) {
        incdec = 2;
        parm   = 12; 
        s_zen  = 99;
      }
         
    }
    else s_zen = 0;  
   // Serial.println(s_zen);
    if (newPos1 != oldPos1) {
      if (newPos1 > oldPos1)  para = para + incdec; // +
      else                    para = para - incdec; // -
      if (para > parm)        para = parm;
      if (para <= 0)  {
        para = 0;           // 0
        if (pnro >= 10)  para = 1;
      
      }
   
      drawVar(xa + 1, ya + 2, wo, ho, xa + 5, ya + 16, para);
      oldPos1 = newPos1;
      delay(30);    // 15
    }
   // Serial.print("para: ");Serial.println(para);
  }

}


void batVolt() {    // bat-voltage on tft-display
  
  tft.setFreeFont(NULL); // NULL: use (old/)5tandard font GLCD.h
  tft.setTextSize(1);
  tft.setCursor(50, 2);
  tft.setTextColor(TFT_GREEN);
  tft.print(txt[7]);
  //DrawStringAt(50, 9, txt[7], 1, TFT_GREEN); // " batt. [V]:"  //85
  DrawBox(79, 1, 35, 9, TFT_BLACKALMOST);  //12
  tft.setTextColor(TFT_BRIGHTBLUE);//tft.setCursor(50, 12);
  tft.print(ubat, 2);
  tft.print(" V");
  //  DrawFloatAt(86, 9, ubat, 2, 2, TFT_BRIGHTBLUE);  //19
  //  DrawCharAt(114, 9, 'V', 1, TFT_GREEN); // " batt. [V]:"  //85
  if (ubat < 3.2) ubat = 3.2;      // test   //#######################################

  if (ubat >= 3.20) return;              // 4.24V = 850 adc-counts battery ok

  tft.fillScreen(TFT_BLACK);
  // tft.fillScreen(TFT_BLACK); //ClearDisplay(TFT_BLACK);  // black display: minimize current
  tft.setCursor(50, 2);
  tft.setTextColor(TFT_MAGENTA);
  tft.print(ubat, 2);
  tft.print(" V");
  //  DrawFloatAt(86, 9, ubat, 2, 1, TFT_CYAN);    // 19
  //  DrawCharAt(114, 9, 'V', 1, TFT_GREEN); // " batt. [V]:"  //85
  efnr = 0;
  efnr = blinkFrame(255);          // blinking frame wait long for 2. press push-button


}
