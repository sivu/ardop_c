// Project specific configuration for Teensy

// At the moment only Packet, ARDOP or SOUNDCARD

// These tests assume a modified platform.txt
// If you haven't done that you will need to define ARDOP PACKET or SOUNDCARD yourself

#define ARDOP

#ifdef PROJECT_SoundCard  
#define SOUNDCARD
#define ProgramName "SoundCard"
#endif

#ifdef PROJECT_SoundCardPkt  
#define SOUNDCARD
#define PACKET
#define SOUNDCARDPACKET
#define  ProgramName "SoundCard + Packet"
#endif


#ifdef PROJECT_ARDOP_Teensy
#define ARDOP
#define  ProgramName "ARDOP 1"
#endif

#ifdef PROJECT_ARDOP2_Teensy
#define ARDOP
#define  ProgramName "ARDOP2"
#endif

#ifdef PROJECT_ARDOP3_Teensy
#define ARDOP
#define  ProgramName "ARDOP3"
#endif

#ifdef PROJECT_ARDOP3K_Teensy
#define ARDOP
#define  ProgramName "ARDOP3"
#endif

#ifdef PROJECT_ARDOPOFDM_Teensy
#define ARDOP
#define _OFDM
#define ProgramName "ARDOPOFDM"
#endif

#ifdef PROJECT_ARDOP1OFDM_Teensy
#define ARDOP
#define _OFDM
#define  ProgramName "ARDOP1OFDM"
#endif

#ifdef PROJECT_SM_Teensy
#define PACKET
#define  ProgramName "Packet"
#endif

#ifdef PROJECT_ARDOPWithPacket
#define ARDOP
#define PACKET
#define  ProgramName "ARDOP+Packet"
#endif

// Standard definitions


#define TEENSY

// Define to use i2s input or output, eg for Teensy 4

#define Usei2sOut			// Needed for T4, may be better for T3
#define Usei2sIn			// T4 has ADC but i2c may be better

#ifdef ARDOP
#include "TeensyConfigARDOP.h"
#endif

#ifdef PACKET
#include "TeensyConfigPacket.h"
#endif

#ifdef SOUNDCARD
#ifndef PACKET
#include "TeensyConfigSoundCard.h"
#endif
#endif


