// Board specific configuration for Teensy

// There are currently two boards, one designed by the WDT and one by me (G8BPQ). Mine is 
// a Raspberry PI form factor board. The WDT board is intended to be a standalone unit.

// Currently there are two applications using these boards, ARDOP and a port of Thomas Sailer's
// Packet Soundmodem. The port supports 1200 and 9600 modems. 

// WDTBOARD is the WDT version, with 4 LEDS and Switches, an Adafruit TFT display.
// a 10 segment LED bar driven by a CAT4016 and two digital pots using i2c. The standard
// setup is with the Host Port on USB and a monitor/debug port on Serial1, though a 
// Bluetooth version is planned.

// WDTV2 is Version 2 of the  WDT board, with 4 LEDS and Switches, an Adafruit TFT display.
// a 10 segment LED bar driven by a CAT4016 and two digital pots using i2c. The standard
// setup is with the Host Port on USB and a monitor/debug port on Serial1 or with Bluetooth on Serial 5
// CAT (CI-V or RS232) on Serial4

// PIBOARD is the Raspberry PI i2c Board. It has 4 LEDS and the Host port on 
// Serial1 Serial3 or i2c. There is no TFT,  CAT4016 Display or switches.
// Level setting pots are on SPI

// This file is for ARDOP

//#define PIBOARD
//#define WDTBOARD

#ifndef PIBOARD
#ifndef WDTBOARD

// Default to WDT V2

#define WDTV2
#endif
#endif

// define PLOTCONSTELLATION if you have a display attached and want a constellation display

#define PLOTCONSTELLATION

// define the type of display 
// OLED is i2c 128 x 64 Display
// WDTTFT is the TFT on the WDT version of the board.
// KMR_1.8 is 1.8" TFT 128 x 160 SPI with ST7735 chip

//#define OLED

// define the type of display 
// OLED is i2c 128 x 64 Display
// WDTTFT is the TFT on the WDT version of the board.
// KMR_1.8 is 1.8" TFT 128 x 160 SPI with ST7735 chip

#define WDTTFT
//#define KMR_18

// if using KMR_18 display with Black or Green TAB uncomment relevant line
// for Red Tab leave both out

//#define KMR_BLACKTAB
//#define KMR_GREENTAB

#ifdef PIBOARD

// Can use Serial or I2C for Host Interface

// if HOSTPORT is not defined, i2c will be used

// Define LOGTOHOST for logging over Host Port (Serial or i2c)
// Define MONPORT for logging to Teensy Serial Port

// Serial3 connects to ESP01 Module

// To use a Serial port for host link, define here
// Serial for USB Port
// Serial1 for PI Header
// Serial3 for ESP01 Header
// If using Serial1 or Serial3 also define SERIAL1SIZE or SERIAL3SIZE to
// increase size of serial port buffers

#define HOSTPORT Serial1
#define HOSTSPEED 115200
#define SERIAL1SIZE 512

#define MONPORT Serial
//#define CPULOAD

#define CATPORT Serial5
#define CATSPEED 19200

// Shouldn't need to change anything below here

#ifndef MONPORT
#define LOGTOHOST
#endif

#ifndef HOSTPORT
#define I2CHOST
#define I2CSLAVEADDR 0x1F
#endif


#define HASPOTS
#define SPIPOTS
#define SPIPOTCS 10

#define pttPin 6

#define LED0 2
#define LED1 3
#define LED2 4
#define LED3 5

#endif


#ifdef WDTBOARD

// Original WDT BOard

#define HOSTPORT Serial1
#define HOSTSPEED 115200
#define MONPORT Serial
#define CATPORT Serial5
#define CATSPEED 19200

#ifndef MONPORT
#define LOGTOHOST
#endif

#define TFT
#define BARLEDS

//#define HASPOTS
//#define I2CPOTS
//#define I2CPOTADDR

#define pttPin 6

#define LED0 24
#define LED1 25
#define LED2 26
#define LED3 31

#define SW1 27
#define SW2 28
#define SW3 29
#define SW4 30

// CAT4016 10 LED display

#define CLK 2
#define BLANK 3
#define LATCH 4
#define SIN 5

#endif


#ifdef WDTV2

// New WDT Board

#define HOSTPORT Serial1
#define HOSTSPEED 115200
#define MONPORT Serial
#define SERIAL1SIZE 512
#define CATPORT Serial4
#define CATSPEED 19200
#define BTPORT Serial5

// You can log to both a serial port and the host by defining both MONPORT and LOGTOHOST

//#ifndef MONPORT
#define LOGTOHOST
//#endif


#ifdef BTPORT
#define SERIAL5SIZE 512
//#define HOSTPORT2 Serial5
#endif


#define TFT
#define BARLEDS

#define HASPOTS
#define I2CPOTS
#define I2CPOTADDR 0x28

#define pttPin 6

#define LED1 8
#define LED2 26
#define LED3 25

#define SW1 27
#define SW2 28

// CAT4016 10 LED display

#define CLK 2
#define BLANK 3
#define LATCH 4
#define SIN 5

/* Bluetooth Defines for RN4678 */
//                          Pin   BT-Function
#define BT_WakeUp           A9    // In  H = Module On
#define BT_SoftwareButton   7     // In  H    L = Put Module in Standby
#define BT_Status1          A7    // Out      See table 2.3 in datasheet
#define BT_Status2          A8    // Out      See table 2.3 in datasheet
#define BT_CTS              24    // In   
#define BT_RTS              30    // Out
#define BT_PairingKey       35    // In  H    L = Force Standby
#define BT_UartRxInd        A17   // In  H    L = UART I/O In stanby mode
#define BT_LinkDropCtl      A18   // In  H    L = Drop BLE link
#define BT_InquiryCtl       A19   // In  H    L = Force BT-Classic mode
#define BT_ResetN           A20   // In  H    L = Module reset
#define BT_EAN              A6    // In  L    See table 2.1 in datasheet
#define BT_P2_4             A1    // In  H    See table 2.1 in datasheet
#define BT_P2_0             A3    // In  H    See table 2.1 in datasheet
#define BT_P3_7             29    // Out




#endif

#ifdef OLED
#define ConstellationHeight 64
#define ConstellationWidth 64
#define PLOTRADIUS 30
#define WHITE 0xffff
#define Tomato 0xffff					// Only have black or white
#define Gold 0xffff
#define Lime 0xffff
#define Yellow 0xffff
#define Goldenrod 0xffff
#define Fuchsia 0xffff
#endif

#ifdef KMR_18

//	Using KMR-1.8 128 * 160 TFT

#define ConstellationHeight 91
#define ConstellationWidth 91
#define PLOTRADIUS 42
// Set position of constellation on display
#define ConsXoffset 69
#define ConsYoffset 0
#define WHITE 0xffff
#define Tomato 0xFD20	// ILI9341_ORANGE
#define Gold Yellow
#define Lime 0x07E0 	// ILI9341_GREEN
#define Yellow 0xFFE0	// ILI9341_YELLOW 
#define BLACK 0 
#define Goldenrod Yellow
#define Fuchsia 0xFD20
    
#define Green 0x07E0 
#define Red 0xF800 


#endif

#ifdef WDTTFT

//	Using the display on the WDT board (ILI9341)

#define ConstellationHeight 101
#define ConstellationWidth 101
#define PLOTRADIUS 49
// Set position of constellation on display
#define ConsXoffset 200
#define ConsYoffset 0
#define WHITE 0xffff
#define Tomato 0xFD20	// ILI9341_ORANGE
#define Gold Yellow
#define Lime 0x07E0 	// ILI9341_GREEN
#define Yellow 0xFFE0	// ILI9341_YELLOW  
#define Goldenrod 0xffff
#define Fuchsia 0xffff
    

#if 0
#define ILI9341_BLACK       0x0000      /*   0,   0,   0 */
#define ILI9341_NAVY        0x000F      /*   0,   0, 128 */
#define ILI9341_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define ILI9341_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define ILI9341_MAROON      0x7800      /* 128,   0,   0 */
#define ILI9341_PURPLE      0x780F      /* 128,   0, 128 */
#define ILI9341_OLIVE       0x7BE0      /* 128, 128,   0 */
#define ILI9341_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define ILI9341_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define ILI9341_BLUE        0x001F      /*   0,   0, 255 */
#define ILI9341_GREEN       0x07E0      /*   0, 255,   0 */
#define ILI9341_CYAN        0x07FF      /*   0, 255, 255 */
#define ILI9341_RED         0xF800      /* 255,   0,   0 */
#define ILI9341_MAGENTA     0xF81F      /* 255,   0, 255 */
#define ILI9341_YELLOW      0xFFE0      /* 255, 255,   0 */
#define ILI9341_WHITE       0xFFFF      /* 255, 255, 255 */
#define ILI9341_ORANGE      0xFD20      /* 255, 165,   0 */
#define ILI9341_GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define ILI9341_PINK        0xF81F
#endif

#endif

#define Statsprintf MONprintf
#define WriteExceptionLog MONprintf
