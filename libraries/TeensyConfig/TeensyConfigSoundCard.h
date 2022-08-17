// Board specific configuration for Teensy

// There are currently two boards, one designed by the WDT and one by me (G8BPQ). Mine is 
// a Raspberry PI form factor board. The WDT board is intended to be a standalone unit.

// Currently there are two applications using these boards, ARDOP and a port of Thomas Sailer's
// Packet Soundmodem. The port supports 1200 and 9600 modems. 

// WDTBOARD is the WDT version, with 4 LEDS and Switches, an Adafruit TFT display.
// a 10 segment LED bar driven by a CAT4016 and two digital pots using i2c. The standard
// setup is with the Host Port on USB and a monitor/debug port on Serial1, though a 
// Bluetooth version is planned.

// PIBOARD is the Raspberry PI i2c Board. It has 4 LEDS and the Host port on 
// Serial1 Serial3 or i2c. There is no TFT,  CAT4016 Display or switches.
// Level setting pots are on SPI

// This file is for Soundmodem

// Standard definitions

// Board Defines

#define PIBOARD

#ifdef PIBOARD

// Can use Serial or I2C for Host Interface

// if HOSTPORT is not defined, i2c will be used

// Define LOGTOHOST for logging over Host Port (Serial or i2c)
// Define MONPORT for logging to Teensy Serial Port

// Serial3 connects to ESP01 Header and can be used with level converters as a CAT port


// If using Serial1 or Serial3 also define SERIAL1SIZE or SERIAL3SIZE to
// increase size of serial port buffers

#define HOSTPORT Serial
#define SERIAL1SIZE 512
#define HOSTSPEED 115200

// if using USB port for CAT can't use it for Monitoring

#define MONPORT Serial

//#define HIDCAT			// CAT over HID
//#define SERIALCAT			// CAT over USB Serial

#ifdef SERIALCAT
#undef MONPORT
#endif

#define MONPORT Serial


#define CATPORT Serial3
#define CATSPEED 19200

// Shouldn't need to change anything below here

#define HASPOTS
#define SPIPOTS
#define SPIPOTCS 10

#define pttPin 6
 
#define LED0 2
#define LED1 3
#define LED2 4
#define LED3 5

#else

#define WDTV2

// New WDT Board

#define HOSTPORT Serial1
#define HOSTSPEED 115200
#define MONPORT Serial
#define SERIAL1SIZE 512

#define TFT
#define WDTTFT

#define BARLEDS

#define HASPOTS
#define I2CPOTS
#define I2CPOTADDR 0x28

#define pttPin 6

#define LED0 8
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
    

#endif
