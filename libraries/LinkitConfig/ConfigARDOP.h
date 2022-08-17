// Board specific configuration for Linkit 7697

// define PLOTCONSTELLATION if you have a display attached and want a constellation display

//#define PLOTCONSTELLATION

// define the type of display 
// OLED is i2c 128 x 64 Display
// WDTTFT is the TFT on the WDT version of the board.
// KMR_1.8 is 1.8" TFT 128 x 160 SPI with ST7735 chip

//#define OLED

// define the type of display 
// OLED is i2c 128 x 64 Display
// WDTTFT is the TFT on the WDT version of the board.
// KMR_1.8 is 1.8" TFT 128 x 160 SPI with ST7735 chip

//#define WDTTFT
//#define KMR_18


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

#define HOSTPORT Serial
#define HOSTSPEED 115200


#define MONPORT Serial
#define CPULOAD

// Shouldn't need to change anything below here

#ifndef MONPORT
#define LOGTOHOST
#endif

#ifndef HOSTPORT
#define I2CHOST
#define I2CSLAVEADDR 0x1F
#endif


/*
#define HASPOTS
#define SPIPOTS
#define SPIPOTCS 10

#define pttPin 6

#define LED0 2
#define LED1 3
#define LED2 4
#define LED3 5

*/

#define Statsprintf MONprintf
#define WriteExceptionLog MONprintf
