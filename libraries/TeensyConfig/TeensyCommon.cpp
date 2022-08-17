// Common Arduino interface code for modems running on a Teensy 3.6

// Currently used by ARDOP and Packet TNC

#include <EEPROM.h>
#include "TeensyConfig.h"
#include "TeensyCommon.h"

extern "C"  void debugprintf(const char * format, ...);

// This seems to be the only way to change the serial port buffer size
// without editing the IDE core file. We set the equates then include the
// core library source

// We need bigger buffers if we want to use a hardware serial port for the host
// interface (to avoid buffering delays)

// We need to be able to queue a full KISS or Hostmode frame to the host without
// waiting

#ifdef SERIAL1SIZE
#define SERIAL1_TX_BUFFER_SIZE	SERIAL1SIZE // number of outgoing bytes to buffer
#define SERIAL1_RX_BUFFER_SIZE	SERIAL1SIZE
//#include "serial1.c"
#include "HardwareSerial1.cpp"
#endif

#ifdef SERIAL3SIZE
#define SERIAL3_TX_BUFFER_SIZE	SERIAL3SIZE // number of outgoing bytes to buffer
#define SERIAL3_RX_BUFFER_SIZE	SERIAL3SIZE
//#include "serial3.c"
#include "HardwareSerial3.cpp"
#endif

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

void CAT4016(int value);
void setupTFT();

// extern "C" {#include "..\..\ARDOPC.h"}

#include "SPI.h"

#if defined I2CHOST || defined I2CKISS || defined I2CMONITOR || defined I2CPOTS



#endif
#include <i2c_t3.h>
void StartADC();
void setupDAC();
void setupADC();
void i2csetup();

extern "C" void ProcessKISSMessage(unsigned char * Packet, int Length);
extern "C" int PutChar(unsigned char c);
extern "C" int i2cPutChar(unsigned char c);
extern "C" void SendMonUI(char * Mess);

#include <DMAChannel.h>

extern int VesionNo;
extern int ActivePort;
extern int SerialMode;
  
extern BOOL SerialHost;

extern int centreFreq;

DMAChannel dma1(true);
DMAChannel dma2(true);

int VRef = 32768;				// ADC and ADC reference (ideal is 32678)

unsigned int PKTLEDTimer = 0;

#define PKTLED LED3				// flash when packet received

extern int KISSCHECKSUM;


extern volatile unsigned short * DMABuffer;
extern volatile int dmaints;
extern volatile int samplessent;

extern volatile int lastperiod, lastint;

void CommonSetup()
{
#ifdef HOSTPORT
  HOSTPORT.begin(HOSTSPEED);
#endif
#ifdef HOSTPORT2
  HOSTPORT2.begin(HOSTSPEED);
#endif
#ifdef HOSTPORT3
  HOSTPORT3.begin(HOSTSPEED);
#endif
#ifdef MONPORT
  MONPORT.begin(115200);
#endif
#ifdef CATPORT
  CATPORT.begin(CATSPEED);				// CAT Port
#endif

  // Set 10 second watchdog

  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  WDOG_TOVALL = 15000; // The next 2 lines sets the time-out value. This is the value that the watchdog timer compare itself to.
  WDOG_TOVALH = 0;
  WDOG_STCTRLH = (WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN |
                  WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_STOPEN); // Enable WDG

  WDOG_PRESC = 0; // This sets prescale clock so that the watchdog timer ticks at 1kHZ instead of the default 1kHZ/4 = 200 HZ

#ifdef BTPORT

 // Set up BT

  Serial5.begin(115200);        // default for the module requires CTS/RTS


  pinMode (BT_WakeUp, OUTPUT);
  pinMode (BT_SoftwareButton, OUTPUT);
  pinMode (BT_Status1, INPUT);
  pinMode (BT_Status2, INPUT);
  pinMode (BT_CTS, OUTPUT);
  pinMode (BT_RTS, INPUT);
  pinMode (BT_PairingKey, OUTPUT);
  pinMode (BT_UartRxInd, OUTPUT);
  pinMode (BT_LinkDropCtl, OUTPUT);
  pinMode (BT_InquiryCtl, OUTPUT);
  pinMode (BT_ResetN, OUTPUT);
  pinMode (BT_EAN, OUTPUT);
  pinMode (BT_P2_4, OUTPUT);
  pinMode (BT_P2_0, OUTPUT);
  pinMode (BT_P3_7, INPUT);


  digitalWriteFast(BT_WakeUp, 1);         // 1 = normal
  digitalWriteFast(BT_SoftwareButton, 1); // 1 = Power on
  digitalWriteFast(BT_CTS, 0);            // Force CTS for now
  digitalWriteFast(BT_PairingKey, 1);     // 1 = normal
  digitalWriteFast(BT_UartRxInd, 1);      // 1 = normal
  digitalWriteFast(BT_LinkDropCtl, 1);    // 0 = disconnect BLE session    
  digitalWriteFast(BT_InquiryCtl, 1);     // 0 = Force Classic Mode
  digitalWriteFast(BT_ResetN, 1);         // 1 = normal
  digitalWriteFast(BT_EAN, 0);            // 0 = normal
  digitalWriteFast(BT_P2_4, 1);           // 1 = normal
  digitalWriteFast(BT_P2_0, 1);           // 1 = normal

// lets reset BT module
  digitalWriteFast(BT_ResetN, 0);
  delay(100);
  digitalWriteFast(BT_ResetN, 1);
  delay(100);

#endif
  
#ifdef MONPORT
  MONPORT.printf("Monitor Buffer Space %d\r\n", MONPORT.availableForWrite());
#endif
  
#if defined HOSTPORT
  WriteDebugLog(0, "Host Buffer Space %d", HOSTPORT.availableForWrite());
  #if defined HOSTPORT2
    WriteDebugLog(0, "Host Port 2 Buffer Space %d", HOSTPORT2.availableForWrite());
  #endif
  #if defined HOSTPORT3
    WriteDebugLog(0, "Host Port 3 Buffer Space %d", HOSTPORT3.availableForWrite());
   #endif
#elif defined I2CHOST
  WriteDebugLog(0, "Host Connection is i2c on address %x Hex", I2CSLAVEADDR);
#elif defined I2CKISS
  WriteDebugLog(0, "Host Connection is i2cKISS on address %x Hex", I2CSLAVEADDR);
#endif

  if (RCM_SRS0 & 0X20)		// Watchdog Reset
    WriteDebugLog(LOGCRIT, "\n**** Reset by Watchdog ++++");

  pinMode(pttPin, OUTPUT);
#ifdef LED0
  pinMode(LED0, OUTPUT);
#endif
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  // Flash Leds to show starting

#ifdef LED0
  SetLED(LED0, 1);
  SetLED(LED1, 1);
#endif
  SetLED(LED2, 1);
  SetLED(LED3, 1);
  delay(200);
#ifdef LED0
  SetLED(LED0, 0);
  delay(200);
#endif
  SetLED(LED1, 0);
  delay(200);
  SetLED(LED2, 0);
  delay(200);
  SetLED(LED3, 0);

#ifdef WDTBOARD

  pinMode (SW1, INPUT_PULLUP);
  pinMode (SW2, INPUT_PULLUP);
//  pinMode (SW3, INPUT_PULLUP);
//  pinMode (SW4, INPUT_PULLUP);

#endif
#ifdef BARLEDS

  pinMode(CLK, OUTPUT);
  pinMode(BLANK, OUTPUT);
  pinMode(LATCH, OUTPUT);
  pinMode(SIN, OUTPUT);

  // Clear CAT4016

  digitalWriteFast(BLANK, 0);	// Enable display
  digitalWriteFast(LATCH, 0);	// Strobe High to copy data from shift reg to display
  digitalWriteFast(CLK, 0);		// Strobe High to enter data to shift reg

  CAT4016(0);				// All off
#endif

#if defined I2CKISS
	KISSCHECKSUM = 1;
#endif

#if defined I2CHOST || defined I2CKISS || defined I2CMONITOR
	i2csetup();
#endif

#if defined I2CPOTS
	Wire.begin();
#endif

#ifdef SPIPOTS
	pinMode (SPIPOTCS, OUTPUT);	// SPI Pot
	digitalWriteFast (SPIPOTCS, HIGH);
#endif

#if defined(SPIPOTS) || defined (TFT)
	SPI.begin();
#endif

#ifdef TFT
	setupTFT();
#endif

  // Set intial TX and RX Levels

  if (RXLevel)			// Zero means auto set level
    AdjustRXLevel(RXLevel);
  else
    AdjustRXLevel(autoRXLevel);

  AdjustTXLevel(TXLevel);
}

extern "C"
{
  int LastNow;
  


  unsigned int getTicks()
  {
    return millis();
  }
  
   void printtick(char * msg)
  {
#ifdef MONPORT
    MONPORT.printf("%s %i\r\n", msg, Now - LastNow);
    LastNow = Now;
#endif	
  }

  int loadCounter = 0;		// for performance monitor
  int lastLoadTicks = 0;
  unsigned int lastRTCTick = 0;	// Make sure this is set to ticks mod 1000 so RTC incrememtns at mS 0
  unsigned int RTC = 0;		// set to seconds since 01.01.2000 by DATE and TIME Commands (Dragon log time epoch)
  
  void Sleep(int mS)
  {
#if defined MONPORT && defined CPULOAD

    int loadtime = millis() - lastLoadTicks;

    loadCounter += mS;

    if (loadtime > 999)
    {
      MONPORT.printf("Load = %d\r\n" , 100 - (100 * loadCounter / loadtime));
      lastLoadTicks = millis();
      loadCounter = 0;
    }
#endif
    delay(mS);
  }
  void PlatformSleep(int mS)
  {
    noInterrupts();
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;
    interrupts();

	// update clock time
	
	while ((millis() - lastRTCTick) > 999)
	{
		lastRTCTick += 1000;
		RTC++;
	}
	if (PKTLEDTimer && Now > PKTLEDTimer)
    {
      PKTLEDTimer = 0;
      SetLED(PKTLED, 0);				// turn off packet rxed led
    }
	Sleep(mS);
  }

  void txSleep(int mS)
  {
    // called while waiting for next TX buffer. Run background processes

    PollReceivedSamples();			// discard any received samples
    HostPoll();
    PlatformSleep(0);
    Sleep(mS);
  }



// Code to access Digital Pots (used for setting input and output levels)

// PI Board uses SPI, WDT Board uses i2c

#ifdef HASPOTS
  void AdjustRXLevel(int Level)
  {
 #ifdef PIBOARD
	int Pot = (Level * 256) / 3000;
#else
	
	// Tom's board. Res = 3.9k , pot is in feedback loop
	// Gain 3000 / Level
	// Pot Res = Gain * 3.9
	// pot val = 256 * Res / 50
	
	int Pot = (3000 * 39 * 256) / (500 * Level);	//39 instead of 3.9, 500 instead of 50 to avoid loss of precsision
#endif

    WriteDebugLog(LOGINFO, "Adjusting RX Level %d mV Pot %d", Level, Pot);

	if (Pot > 256)
		Pot = 256;

    if (Level > 0)
		SetPot(0, Pot);		// Write to live and nv regs
  }

  void AdjustTXLevel(int Level)
  {
    int Pot = (Level * 256) / 3000;
    WriteDebugLog(LOGINFO, "Adjusting TX Level %d mV Pot %d", Level, Pot);
    SetPot(1, Pot);		// Write to live
  }
#else
  void AdjustRXLevel(int Level)
  {}
  void AdjustTXLevel(int Level)
  {}
#endif

#ifdef SPIPOTS

  void SetPot(int address, unsigned int value)
  {
    int Command = address << 4;  // Reg addr to top 4 bits, write opcode is zero

    if (GetPot(address) == value)
      return;											// Don't write if same

    if (value & 0x100)						// 9th bit set?
      Command |= 1;								// goes in bottom bit of command

    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWriteFast(SPIPOTCS, LOW);

    SPI.transfer(Command);
    SPI.transfer(value);

    digitalWriteFast(SPIPOTCS, HIGH);
    // release control of the SPI port
    SPI.endTransaction();
  }

  unsigned int GetPot(int i)
  {
    byte thisRegister;
    unsigned int result = 0, result1 = 0;   // result to return

    thisRegister = (i << 4) | 0x0c;					// Read Command

    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

    digitalWrite(SPIPOTCS, LOW);

    // Device returns the top two bits of the value when address is sent

    result1 = SPI.transfer(thisRegister);
    // send a value of 0 to read the low 8 bits
    result = SPI.transfer(0x00);

    digitalWrite(SPIPOTCS, HIGH);
    SPI.endTransaction();

    return (result | (result1 << 8));
  }

#endif
#ifdef I2CPOTS

size_t idx;
#define MEM_LEN 256
char databuf[MEM_LEN];


  void SetPot(int address, unsigned int value)
  {
	Wire.beginTransmission(I2CPOTADDR);       // Slave address
	Wire.write((address << 4) | (value >> 8));      // opcode + top bit of value
	Wire.write(value & 0xff);    // value
	Wire.endTransmission();               // Transmit to Slave

    // Check if error occured
    if (Wire.getError())
      WriteDebugLog(LOGINFO, "i2c Pot write FAIL");

    return ; 
  }

  
  unsigned int GetPot(int i)
  {
	int n;
	
    Wire.beginTransmission(I2CPOTADDR);         // Slave address
    Wire.write((i << 4) | 0x0c);      // opcode
    Wire.endTransmission();           // Transmit to Slave

    // Check if error occured
    if (Wire.getError())
      WriteDebugLog(LOGINFO, "i2c Pot selest FAIL ");
 
    // Read from Slave

    n = Wire.requestFrom(I2CPOTADDR, 2); // Read from Slave (len 2)

    // Check if error occured
    if (Wire.getError())
      WriteDebugLog(LOGINFO, "i2c Pot read FAIL ");
    else
    {
		idx = 0;
		while (Wire.available())
			databuf[idx++] = Wire.readByte();
    }
  
    return ((databuf[0] << 8) + databuf[1]); 
}

#endif

#ifndef SPIPOTS
#ifndef I2CPOTS

  // Dummys for platforms without digital pots

  void SetPot(int address, unsigned int value)
  {}

  unsigned int GetPot(int i)
  {
    return 0;
  }
#endif
#endif
}


// LED Bargrath Code

void CAT4016(int value)
{
#ifdef BARLEDS
  // writes value to the 12 LED display
  int i;

  for (i = 0; i < 16; i++)			// must send all 16 to maintain sync
  {
    // Send each bit to display

    // We probably don't need a microsecond delay, but I doubt if it will
    // cause timing problems anywhere else

    // looks like we need to send data backwards (hi order bit first)

    digitalWriteFast(SIN, (value >> 15) & 1);
    delayMicroseconds(1);		// Setup is around 20 nS
    digitalWriteFast(CLK, 1);		// Copy SR to Outputs
    delayMicroseconds(1);		// Setup is around 20 nS
    digitalWriteFast(CLK, 0);		// Strobe High to copy data from shift reg to display
    value = value << 1;			// ready for next bit
    delayMicroseconds(1);		// Setup is around 20 nS
  }
  // copy Shift Reg to display

  digitalWriteFast(LATCH, 1);		//  Strobe High to copy data from shift reg to display
  delayMicroseconds(1);					// Setup is around 20 nS
  digitalWriteFast(LATCH, 0);
#endif
}
extern "C" void SaveEEPROM(int Reg, unsigned char Val)
{
  if (EEPROM.read(Reg) != Val)
    EEPROM.write(Reg, Val);
}

extern "C" int GetEEPROM(int Reg)
{
  return EEPROM.read(Reg);
}

// Teensy 3.x DAC/ADC Code

// We use two buffers, but DMA treats as one long one, and we process as
// two halves

volatile uint16_t dac1_buffer[DAC_SAMPLES_PER_BLOCK * 2];
extern volatile uint16_t ADC_Buffer[ADC_SAMPLES_PER_BLOCK * 2];

#define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_CONT | PDB_SC_PDBIE | PDB_SC_DMAEN)

//	PDB count of 4999 should give 12K clock with 60 MHz bus clock

void setupPDB(int SampleRate)
{
  // Get PDB running to provide clock to DAC and ADC

  // DAC flow PDB->DMA->DAC
  // ADC flow PDB->DAC->DMA

  SIM_SCGC6 |= SIM_SCGC6_PDB;		// Enable PDB clock

  PDB0_IDLY = 1;
  PDB0_MOD = F_BUS / SampleRate - 1;

  WriteDebugLog(LOGINFO, "Sample Rate %d PDB Divider %d", SampleRate, F_BUS / SampleRate);

  PDB0_SC = 0;
  PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;
  PDB0_SC = PDB_CONFIG | PDB_SC_SWTRIG;
  PDB0_CH0C1 = 0x0101;				// ?? PDB triggers ADC
}

void isr(void)
{
	dmaints++;
	samplessent += 1200;
	lastperiod = millis() - lastint;
	lastint = millis();
	dma1.clearInterrupt();
}

// i2s code based on Teensy Audio Library

/* Audio Library for Teensy 3.X
 * Copyright (c) 2014, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

//	Need to recalculate for 12000

// i2c clock is a bit clock, and I think we need 16 bit stereo samples, so 12000 * 16 * 2 = 384k

// With Clock at 60 MHz I think we need 250 (-1)

// But input may be cpu clock (180 MHz on 3.6) so need 750

#define MCLK_SRC  0		// system clock
#define MCLK_MULT 1	
#define MCLK_DIV  750

// 1088 = 48M / 44.1 K 
// 4000 = 48M / 12K

// I think we send 16 bit samples, with /4 that gives / 128, so 4000 / 128 = 31.25  ???????????

/*
// MCLK needs to be 48e6 / 1088 * 256 = 11.29411765 MHz -> 44.117647 kHz sample rate
// why???? - Seems to have 32 bit samples (so 64 bits stereo) and divide clk by 4, giving 256 above

#if F_CPU == 96000000 || F_CPU == 48000000 || F_CPU == 24000000
  // PLL is at 96 MHz in these modes
  #define MCLK_MULT 2
  #define MCLK_DIV  17
#elif F_CPU == 72000000
  #define MCLK_MULT 8
  #define MCLK_DIV  51
#elif F_CPU == 120000000
  #define MCLK_MULT 8
  #define MCLK_DIV  85
#elif F_CPU == 144000000
  #define MCLK_MULT 4
  #define MCLK_DIV  51
#elif F_CPU == 168000000
  #define MCLK_MULT 8
  #define MCLK_DIV  119
#elif F_CPU == 180000000
  #define MCLK_MULT 16
  #define MCLK_DIV  255
  #define MCLK_SRC  0
#elif F_CPU == 192000000
  #define MCLK_MULT 1
  #define MCLK_DIV  17
#elif F_CPU == 216000000
  #define MCLK_MULT 8
  #define MCLK_DIV  153
  #define MCLK_SRC  0
#elif F_CPU == 240000000
  #define MCLK_MULT 4
  #define MCLK_DIV  85
#elif F_CPU == 16000000
  #define MCLK_MULT 12
  #define MCLK_DIV  17
#else
  #error "This CPU Clock Speed is not supported by the Audio library";
#endif


#ifndef MCLK_SRC
#if F_CPU >= 20000000
  #define MCLK_SRC  3  // the PLL
#else
  #define MCLK_SRC  0  // system clock
#endif
#endif
*/
	
void config_i2s(void)
{
	SIM_SCGC6 |= SIM_SCGC6_I2S;
	SIM_SCGC7 |= SIM_SCGC7_DMA;
	SIM_SCGC6 |= SIM_SCGC6_DMAMUX;

	// if either transmitter or receiver is enabled, do nothing
	if (I2S0_TCSR & I2S_TCSR_TE) return;
	if (I2S0_RCSR & I2S_RCSR_RE) return;

	// enable MCLK output
	I2S0_MCR = I2S_MCR_MICS(MCLK_SRC) | I2S_MCR_MOE;
	while (I2S0_MCR & I2S_MCR_DUF) ;
	I2S0_MDR = I2S_MDR_FRACT((MCLK_MULT-1)) | I2S_MDR_DIVIDE((MCLK_DIV-1));

	// configure transmitter
	I2S0_TMR = 0;
	I2S0_TCR1 = I2S_TCR1_TFW(1);  // watermark at half fifo size
	I2S0_TCR2 = I2S_TCR2_SYNC(0) | I2S_TCR2_BCP | I2S_TCR2_MSEL(1)
		| I2S_TCR2_BCD | I2S_TCR2_DIV(1);									// /4 ?? (DIV + 1) * 2
	I2S0_TCR3 = I2S_TCR3_TCE;
	I2S0_TCR4 = I2S_TCR4_FRSZ(1) | I2S_TCR4_SYWD(31) | I2S_TCR4_MF
		| I2S_TCR4_FSE | I2S_TCR4_FSP | I2S_TCR4_FSD;
	I2S0_TCR5 = I2S_TCR5_WNW(31) | I2S_TCR5_W0W(31) | I2S_TCR5_FBT(31);		// 32 bits per word

	// configure receiver (sync'd to transmitter clocks)
	I2S0_RMR = 0;
	I2S0_RCR1 = I2S_RCR1_RFW(1);
	I2S0_RCR2 = I2S_RCR2_SYNC(1) | I2S_TCR2_BCP | I2S_RCR2_MSEL(1)
		| I2S_RCR2_BCD | I2S_RCR2_DIV(1);
	I2S0_RCR3 = I2S_RCR3_RCE;
	I2S0_RCR4 = I2S_RCR4_FRSZ(1) | I2S_RCR4_SYWD(31) | I2S_RCR4_MF
		| I2S_RCR4_FSE | I2S_RCR4_FSP | I2S_RCR4_FSD;
	I2S0_RCR5 = I2S_RCR5_WNW(31) | I2S_RCR5_W0W(31) | I2S_RCR5_FBT(31);

	// configure pin mux for 3 clock signals
	CORE_PIN23_CONFIG = PORT_PCR_MUX(6); // pin 23, PTC2, I2S0_TX_FS (LRCLK)
	CORE_PIN9_CONFIG  = PORT_PCR_MUX(6); // pin  9, PTC3, I2S0_TX_BCLK
	CORE_PIN11_CONFIG = PORT_PCR_MUX(6); // pin 11, PTC6, I2S0_MCLK
}


 
void setupi2sDAC()
{
	// Configure to use external i2s interface instead of internal DAC. Eg for Teensy4 
	
	// I think once configured i2s will run continuously under DMA,
	
	// I want to use 12000 but could use 48000 if necessary
	
	// Should we also allow i2s input??
	
	// Copied from 
	
	dma1.begin(true); // Allocate the DMA channel first

//	block_left_1st = NULL;
//	block_right_1st = NULL;

	// TODO: should we set & clear the I2S_TCSR_SR bit here?

	config_i2s();

	CORE_PIN22_CONFIG = PORT_PCR_MUX(6); // pin 22, PTC1, I2S0_TXD0

#if defined(KINETISK)
	dma1.TCD->SADDR = dac1_buffer;
	dma1.TCD->SOFF = 2;
	dma1.TCD->ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
	dma1.TCD->NBYTES_MLNO = 2;
	dma1.TCD->SLAST = -sizeof(dac1_buffer);
	dma1.TCD->DADDR = (void *)((uint32_t)&I2S0_TDR0 + 2);
	dma1.TCD->DOFF = 0;
	dma1.TCD->CITER_ELINKNO = sizeof(dac1_buffer) / 2;
	dma1.TCD->DLASTSGA = 0;
	dma1.TCD->BITER_ELINKNO = sizeof(dac1_buffer) / 2;
	dma1.TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;
#endif
	dma1.triggerAtHardwareEvent(DMAMUX_SOURCE_I2S0_TX);

	dma1.enable();

	I2S0_TCSR = I2S_TCSR_SR;
	I2S0_TCSR = I2S_TCSR_TE | I2S_TCSR_BCE | I2S_TCSR_FRDE;
	dma1.attachInterrupt(isr);
}

extern "C" void Starti2sDAC()
{
}

extern "C" void Stopi2sDAC()
{
}

void setupDAC()
{
  // Configure DAC

  SIM_SCGC2 |= SIM_SCGC2_DAC0; // enable DAC clock
  DAC0_C0 = DAC_C0_DACEN | DAC_C0_DACRFS; // enable the DAC module, 3.3V reference

  DAC0_DAT0L = 0;
  DAC0_DATH = 8;		// Set output to mid value (should ramp up??)
  
  DMABuffer = &dac1_buffer[0];
}
extern "C" void StartDAC()
{
  // Start sending a frame under DMA

  // We don't need to do all this each time, but probably not worth
  // working out which is needed

  dma1.disable();

  dma1.TCD->SADDR = dac1_buffer;
  dma1.TCD->SOFF = 2;
  dma1.TCD->ATTR = DMA_TCD_ATTR_SSIZE(DMA_TCD_ATTR_SIZE_16BIT) | DMA_TCD_ATTR_DSIZE(DMA_TCD_ATTR_SIZE_16BIT);
  dma1.TCD->NBYTES_MLNO = 2;	// Bytes per minor loop
  dma1.TCD->SLAST = -sizeof(dac1_buffer);	// Reinit to start
  dma1.TCD->DADDR = &DAC0_DAT0L;					// Write to DAC data register
  dma1.TCD->DOFF = 0;
  dma1.TCD->CITER_ELINKNO = sizeof(dac1_buffer) / 2;
  dma1.TCD->DLASTSGA = 0;
  dma1.TCD->BITER_ELINKNO = sizeof(dac1_buffer) / 2;
  dma1.TCD->CSR = DMA_TCD_CSR_INTMAJOR | DMA_TCD_CSR_INTHALF; 
  dma1.triggerAtHardwareEvent(DMAMUX_SOURCE_PDB);
  
//  debugprintf("dma1.TCD->CITER_ELINKNO %d", dma1.TCD->CITER_ELINKNO);
  
  
  dma1.enable();
  dma1.attachInterrupt(isr);
}


extern "C" void stopDAC()
{
  dma1.disable();
  StartADC();
  DAC0_DAT0L = 0;
  DAC0_DATH = 8;		// Set output to mid value
}

uint16_t dc_average;

void setupADC(int pin)
{
  // pin must be 0 to 13 (for A0 to A13)
  // or 14 to 23 for digital pin numbers A0-A9
  // or 34 to 37 corresponding to A10-A13
  if (pin > 23 && !(pin >= 34 && pin <= 37)) return;

  // Configure the ADC and run at least one software-triggered
  // conversion.  This completes the self calibration stuff and
  // leaves the ADC in a state that's mostly ready to use

  analogRead(pin);

  ADC0_SC2 |= ADC_SC2_ADTRG | ADC_SC2_DMAEN;	// Hardware triggered

  // NVIC_ENABLE_IRQ(IRQ_PDB);
}

// not used, but leave for possible use for timing test

void pdb_isr()
{
  PDB0_SC &= ~PDB_SC_PDBIF; // clear interrupt flag
}

void StartADC()
{

  // The ADC runs all the time. This resets the buffer pointers
  // after a transmit completes.

  dma2.disable();

  dma2.TCD->SADDR = &ADC0_RA;
  dma2.TCD->SOFF = 0;
  dma2.TCD->ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
  dma2.TCD->NBYTES_MLNO = 2;
  dma2.TCD->SLAST = 0;
  dma2.TCD->DADDR = ADC_Buffer;
  dma2.TCD->DOFF = 2;
  dma2.TCD->CITER_ELINKNO = sizeof(ADC_Buffer) / 2;
  dma2.TCD->DLASTSGA = -sizeof(ADC_Buffer);
  dma2.TCD->BITER_ELINKNO = sizeof(ADC_Buffer) / 2;
  dma2.TCD->CSR = 0; //DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;

  dma2.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC0);

  inIndex = 0;
  dma2.enable();
}
extern "C"
{
  int GetDMAPointer()
  {
    return dma1.TCD->CITER_ELINKNO;
  }
  int GetADCDMAPointer()
  {
    return dma2.TCD->CITER_ELINKNO;
  }
  void SetLED(int LED, int state)
  {
#ifdef PIBOARD
    state = !state;			// LEDS inverted on PI Board
#endif
    digitalWriteFast(LED, state);
  }
}

// Code to drive a Adafruit/PRJC or 3.5" 480*320 PI TFT display

#ifdef TFT

#ifdef PI35TFT

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9340.h>

// PI35TFT on Teensy

#define _sclk 13
#define _miso 12
#define _mosi 11
#define _cs 10
#define _dc 9
#define _rst 8

// Color definitions
#define	ILI9341_BLACK   0x0000
#define	ILI9341_BLUE    0x001F
#define	ILI9341_RED     0xF800
#define	ILI9341_GREEN   0x07E0
#define ILI9341_CYAN    0x07FF
#define ILI9341_MAGENTA 0xF81F
#define ILI9341_YELLOW  0xFFE0
#define ILI9341_WHITE   0xFFFF

Adafruit_ILI9340 tft = Adafruit_ILI9340(_cs, _dc, _rst);

#else

// PRJC/Adafruit Display

#include "ILI9341_t3.h"

#define TFT_DC  9
#define TFT_CS 10
#define _RST 255		// Not Used
#define _MOSI 11
#define _SCLK 14		// Clock moved to ALT pin as LED is on A13
#define _MISO 12

// Use hardware SPI

ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, _RST, _MOSI, _SCLK, _MISO);
#endif

static Print * tftptr = NULL;


void setupTFT()
{
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
  tft.setRotation(0);
  tft.setTextSize(2);
  tft.setCursor(0, 220);
  
  // Put Program name on botton row of display
  
 // tft.print(ProgramName);
  tft.setCursor(0, 0);
 
  tftptr = &tft;
  
  //	Indicate PTT Location
  
  tft.fillRect(300, 220, 20, 20, ILI9341_GREEN);

}

 // Write to tft. Gets round problem of different TFT Hardware
 
 void TFTprintf(const char * format, ...)
  {
    char Mess[256];
    va_list(arglist);
    va_start(arglist, format);
    vsnprintf(Mess, sizeof(Mess), format, arglist);
    tft.print(Mess);
    return;
  }




extern "C"
{
  void displayCall(int dirn, char * Call)
  {
    char paddedcall[12] = "           ";

    paddedcall[0] = dirn;
    memcpy(paddedcall + 1, Call, strlen(Call));

    if (tftptr)
    {
      tft.setCursor(0, 140);
      tft.print(paddedcall);
    }
  }

  void displayState(const char * State)
  {
    if (tftptr)
    {
      tft.setCursor(0, 120);
      tft.print("          ");
      tft.setCursor(0, 120);
      tft.print(State);
    }
  }
}
#else

// Dummy routines

extern "C" void displayCall(int dirn, char * Call)
{}
extern "C" void displayState(const char * State)
{}

#endif

// Signal levels for each bar

const int barlevels[12] = {
  1000, 2000, 5000, 8000, 11000,
  16000, 18000, 20000, 24000, 28000, 30000, 32000
};
#ifdef TFT
const int barcolours[12] = {
  ILI9341_YELLOW, ILI9341_YELLOW, ILI9341_YELLOW,
  ILI9341_YELLOW, ILI9341_GREEN, ILI9341_GREEN,
  ILI9341_GREEN, ILI9341_GREEN, ILI9341_GREEN,
  ILI9341_YELLOW, ILI9341_RED, ILI9341_RED
};
#endif
const int CAT4016Levels[13] = {
  0, 1, 3, 7, 0xf, 0b11111,
  0b111111, 0b1111111, 0b11111111, 0x1ff, 0x3ff, 0x7ff, 0xfff
};


extern "C"
{
  void displayLevel(int level)
  {
    int i;
#ifdef TFT
    for (i = 0; i < 12; i++)
    {
      if (level > barlevels[i])
        tft.fillRect(15 * i, 200, 14, 16, barcolours[i]);
      else
        tft.fillRect(15 * i, 200, 14, 16, ILI9341_BLACK);
    }
#endif
    for (i = 0; i < 12; i++)
    {
      if (level < barlevels[i])
        break;
    }
    CAT4016(CAT4016Levels[i]);
  }
}

// Routine to adjust the RX gain pot to keep input level in range

extern "C"
{
  void CheckandAdjustRXLevel(int maxlevel, int minlevel, bool Force)
  {
    int pktopk = (maxlevel - minlevel) * 3300 / 65536;
	
	if (RXLevel)		// Only auto adjust if level = 0)
	{
		WriteDebugLog(LOGDEBUG, "RX Level fixed at %d ", RXLevel);
		return;	
	}
	
    if (!OKtoAdjustLevel() && !Force)			// Protocol specific test
      return;

    // Try adjusting pot to get about 2000 mV

    if (pktopk > 3200)
    {
      // Overloaded so can't get accurate level. Set Gain to minimum

      autoRXLevel = 3000;
      AdjustRXLevel(autoRXLevel);
      return;
    }
    if (pktopk < 40)
    {
      // Assume no input
      WriteDebugLog(LOGDEBUG, "Level below threshold - assume no input %d %d %d pk", maxlevel, minlevel, pktopk);
      return;
    }

    if (pktopk < 1600 || pktopk > 2400)
    {
      // Calculate actual input voltage from current pot gain setting

      pktopk = pktopk * autoRXLevel / 3000;
	  
      WriteDebugLog(LOGDEBUG, "peak to peak input %d mV", pktopk);

      autoRXLevel = pktopk  * 3 / 2;			// try to get to 2/3rd

      if (autoRXLevel > 3000)
        autoRXLevel = 3000;
	
	  if (autoRXLevel < 100)
		  autoRXLevel = 100;

      AdjustRXLevel(autoRXLevel);
    }
  }

  void CatWrite(const uint8_t * Buffer, int Len)
  {
#ifdef CATPORT
    CATPORT.write(Buffer, Len);
#endif
  }

  unsigned char CatRXbuffer[256];
  extern int CatRXLen;

  int RadioPoll()
  {
#ifdef CATPORT
    int Length = CATPORT.available();

    // only try to read number of bytes in queue

    if (CatRXLen + Length > 256)
      CatRXLen = 0;

    Length = CATPORT.readBytes(&CatRXbuffer[CatRXLen], Length);

    if (Length == 0)
      return 0;					// Nothing doing

    CatRXLen += Length;

    Length = CatRXLen;

    //    MONprintf("CAT RX ");
    //    for (i = 0; i < Length; i++)
    //      MONprintf("%x ", CatRXbuffer[i]);
    //    MONprintf("\r\n");
#endif
    return CatRXLen;
  }
}


// i2c support

#if defined I2CHOST || defined I2CKISS || defined I2CMONITOR


void receiveEvent(size_t count);
void requestEvent(void);

#define MEM_LEN 512
uint8_t databuf[MEM_LEN];

volatile int i2cputptr = 0, i2cgetptr = 0, target = 0;
volatile int i2ctxgetptr = 0, i2ctxputptr =0;

void i2csetup()
{
  // Setup for Slave mode, address I2SLAVEADDR, pins 18/19, external pullups, 400kHz
  Wire.begin(I2C_SLAVE, I2CSLAVEADDR, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);

  // Data init

  memset(databuf, 0, sizeof(databuf));

  // register events

  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

unsigned char i2cMessage[512];
int i2cMsgPtr = 0;

#define FEND 0xC0
#define FESC 0xDB
#define TFEND 0xDC
#define TFESC 0xDD

#ifdef I2CHOST

unsigned char i2creply[512] = {0xaa, 0};

volatile int i2creplyptr = 0;
volatile int i2creplylen = 0;

#else

unsigned char i2creply[512] = {192, 8, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 3, 192};
unsigned char i2cidle[2] = {0x0e};

volatile int i2creplyptr = 0;
volatile int i2creplylen = 0;

extern int ESCFLAG;

// This is only called if I2CKISS is enabled. SCS Hostmode uses HostPoll

void i2cloop()
{
  while (i2cgetptr != i2cputptr)
  {
    unsigned char c;
    c = databuf[i2cgetptr++];
    i2cgetptr &= 0x1ff;				// 512 cyclic

    if (c == FEND)
    {
      // Start or end of message

      int i, Len;

      if (i2cMsgPtr == 0)
        continue;							// Start of message

      Len = i2cMsgPtr;
      i2cMsgPtr = 0;

      // New message

#ifdef MONPORT
//		MONPORT.printf("Slave received: ");
//		for (i = 0; i < Len; i++)
//			MONPORT.printf("%x ", i2cMessage[i]);
//		MONPORT.printf("\r\n");
#endif
      if (i2cMessage[0] == 15)
      {
        // Immediate Command

        if (i2cMessage[1] == 1)
        {
          // Get Params Command

          int Sum = 8, val;
		  
		  i2cPutChar(FEND);
		  i2cPutChar(8);				// Get Params Response
		  
          for (i = 0; i < 11; i++)
          {
            val = EEPROM.read(i);
            i2cPutChar(val);
            Sum ^= val;
          }

          // Reg 11 is actual rx level

          val = GetPot(0);
          i2cPutChar(val);
          Sum ^= val;

          val = EEPROM.read(13);	// Centre Freq
          i2cPutChar(val);
          Sum ^= val;

          i2cPutChar(Sum);
          i2cPutChar(FEND);
		  

			// Also write to log for clients that can't handle KISS response
			
			char GetResp[64] = "GetResp: ";
			char * ptr = &GetResp[8];

			for (i = 0; i < 11; i++)
			{
				val = GetEEPROM(i);
				ptr += sprintf(ptr, "%02X ", val);
			}
			val = GetPot(0);
			ptr += sprintf(ptr, "%02X ", val & 0xff);
			val = GetEEPROM(13);
			ptr += sprintf(ptr, "%02X ", val);

			WriteDebugLog(6, GetResp);
		  
          return;
        }
		
		if (i2cMessage[1] == 2)		// Reboot
		   CPU_RESTART // reset processor

        return;			// other immediate command
      }
      if (i2cMessage[0] && i2cMessage[0] != 12)
      {
        // Not Normal or Ackmode Data, so set param

        int reg = i2cMessage[0];
        int val = i2cMessage[1];

        if (reg == 9) // SPI Pots
          SetPot(0, val);
        else if (reg == 10)
          SetPot(1, val);
	  
	    WriteDebugLog(7,"Set KISS Param %d to %d", reg, val);
        SaveEEPROM(reg, val);
		
		// Drop through to process immediately
 
      }

      // Data Frame
	  
//	  WriteDebugLog(7,"I2CKiss Data Frame Opcode %d len %d", i2cMessage[0], Len);

	  ProcessKISSMessage(i2cMessage, Len);
      continue;
    }
    else
    {
      // Not FEND but could be FESC etc
	  
	  	if (ESCFLAG)
		{
			//
			//	FESC received - next should be TFESC or TFEND

			ESCFLAG = FALSE;

			if (c == TFESC)
				c = FESC;
	
			if (c == TFEND)
				c=FEND;
			
			// others leave alone
		}
		else
		{
			if (c == FESC)
			{
				ESCFLAG = TRUE;
				continue;
			}
		}
		
		//
		//	Ok, a normal char
		//

		i2cMessage[i2cMsgPtr++] = c;
    }
  }
}

#endif

//
// handle Rx Event (incoming I2C data)
//
void receiveEvent(size_t count)
{
  target = Wire.getRxAddr(); 	// Get Slave address

  while (count--)
  {
    databuf[i2cputptr++] = Wire.readByte();
    i2cputptr &= 0x1ff;		// 512 cyclic buffer
  }
}

//
// handle Tx Event (outgoing I2C data)
//

void requestEvent(void)
{
#ifdef I2CHOST

	int Len;
	unsigned char Reply[33];
	int i;
	unsigned char * ptr2 = &Reply[1];
	
// SCS Host interface reads blocks of 32 bytes + Length byte (probably - may optimise!)

	if (i2creplylen > 32)
		Len = 32;
	else
		Len = i2creplylen;
	
  Reply[0] = Len;
  
  for (i = 0; i < Len; i++)
  {
	  *(ptr2++) = i2creply[i2ctxgetptr++];
	  i2ctxgetptr &= 0x1ff;		// 512 cyclic buffer
  }
  
  Wire.write(Reply, Len + 1);
  
  i2creplylen -= Len;

#else
	
  // TNC-PI interface works a byte at a time

  if (i2creplylen == 0)			// nothing to send
    Wire.write(i2cidle, 1); 	// return idle
  else
  {
    // return the next byte of the message
    Wire.write(&i2creply[i2ctxgetptr++], 1);
	i2ctxgetptr &= 0x1ff;		// 512 cyclic buffer
    i2creplylen--;
  }
#endif
}

#endif

#include <stdarg.h>
extern "C"
{
  int HostInit()
  {
    SerialMode = 1;			// Teensy always uses Serial (PTC) Interface
	return true;
  }


  void PutString(const char * Msg)
  {
	  SerialSendData((const uint8_t *)Msg, strlen(Msg));
  }

  int PutChar(unsigned char c)
  {
#ifdef HOSTPORT
	if (ActivePort == 1)
	{
		HOSTPORT.write(&c, 1);
		return 0;
	}
#endif
#ifdef HOSTPORT2
	if (ActivePort == 2)
	{
		HOSTPORT2.write(&c, 1);
		return 0;
	}
#endif
#ifdef HOSTPORT3
	if (ActivePort == 3)
	{
		HOSTPORT3.write(&c, 1);
		return 0;
	}
#endif
#if defined I2CHOST || defined I2CKISS
    i2creply[i2creplyptr++] = c;
	i2creplyptr &= 0x1ff;					// 512 cyclic
	i2creplylen++;
#endif
    return 0;
  }
  
  int i2cPutChar(unsigned char c)
  {
  #if defined I2CHOST || defined I2CKISS || defined I2CMONITOR
    i2creply[i2creplyptr++] = c;
	i2creplyptr &= 0x1ff;					// 512 cyclic
	i2creplylen++;
#endif
    return 0;
  }
  
  void SerialSendData(const uint8_t * Msg, int Len)
  {
#ifdef HOSTPORT
	if (ActivePort == 1)
	{
		HOSTPORT.write(Msg, Len);
		return;
	}
#endif
#ifdef HOSTPORT2
	if (ActivePort == 2)
	{
		HOSTPORT2.write(Msg, Len);
		return;
	}
#endif
#ifdef HOSTPORT3
	if (ActivePort == 3)
	{
		HOSTPORT3.write(Msg, Len);
		return;
	}
#endif
#if defined I2CHOST || defined I2CKISS
	uint8_t * ptr =  (uint8_t *)Msg;
	int cnt = Len;
	
	while (cnt--)
	{
		i2creply[i2creplyptr++] = *(ptr++);
		i2creplyptr &= 0x1ff;					// 512 cyclic
	}
	i2creplylen += Len;
#endif
  }

  void WriteDebugLog(int Level, const char * format, ...)
  {
	  // Use Dragon log format (stats with 32 bit timestamp)
	  
    char Mess[256];
    va_list(arglist);
	int len;

    va_start(arglist, format);
#ifdef LOGTOHOST

	// correct RTC if needed
	
	while ((millis() - lastRTCTick) > 999)
	{
		lastRTCTick += 1000;
		RTC++;
	}
	Mess[0] = (RTC >> 24) & 0xff;
	Mess[1] = (RTC >> 16) & 0xff;
	Mess[2] = (RTC >> 8) & 0xff;
	Mess[3] = RTC & 0xff;

	sprintf(&Mess[4], "%03d", millis() % 1000);	// millisecs
    Mess[7] = '0' + Level;

    len = vsnprintf(&Mess[8], sizeof(Mess) - 8, format, arglist);
    strcat(&Mess[8], "\r\n");
    SendLogToHost(Mess, len + 10);
#endif
#ifdef MONPORT
    vsnprintf(Mess, sizeof(Mess), format, arglist);
    MONPORT.println(Mess);
#endif
#ifdef UIMON
    vsnprintf(Mess, sizeof(Mess), format, arglist);
    SendMonUI(Mess);
#endif
    return;
  }

  // Write to Log, either via host or serial port
  void MONprintf(const char * format, ...)
  {
    char Mess[256];
    va_list(arglist);
	int len;

    va_start(arglist, format);
#ifdef LOGTOHOST
 
 // correct RTC if needed

	while ((millis() - lastRTCTick) > 999)
	{
		lastRTCTick += 1000;
		RTC++;
	}
	Mess[0] = (RTC >> 24) & 0xff;
	Mess[1] = (RTC >> 16) & 0xff;
	Mess[2] = (RTC >> 8) & 0xff;
	Mess[3] = RTC & 0xff;

	sprintf(&Mess[4], "%03d", millis() % 1000);	// millisecs
    Mess[7] = '0';

    len = vsnprintf(&Mess[8], sizeof(Mess) - 8, format, arglist);

    strcat(&Mess[8], "\r\n");
    SendLogToHost(Mess, len + 10);
#endif
#ifdef MONPORT
    vsnprintf(Mess, sizeof(Mess), format, arglist);
    MONPORT.println(Mess);
#endif
#ifdef UIMON
    vsnprintf(Mess, sizeof(Mess), format, arglist);
    SendMonUI(Mess);
#endif
    return;
  }

  void CloseDebugLog()
  {
  }

  void CloseStatsLog()
  {
  }

#ifdef LOGTOHOST

#define LOGBUFFERSIZE 2048

  extern char LogToHostBuffer[LOGBUFFERSIZE];
  extern int LogToHostBufferLen;
  extern int PTCMode;

  
  void SendLogToHost(char * Cmd, int len)
  {
    // I think we need log in text mode
    //	if (HostMode & !PTCMode)	// ARDOP Native

    if (!PTCMode)	// ARDOP Native
    {
      char * ptr = &LogToHostBuffer[LogToHostBufferLen];
     
      if (LogToHostBufferLen + len >= LOGBUFFERSIZE)
        return;			// ignore if full
		
      memcpy(ptr, Cmd, len);
      LogToHostBufferLen += len;
    }
  }
#endif


// Sound Routines. These are all "C" Routines

int Index = 0;				// DMA Buffer being used 0 or 1
int inIndex = 0;			// DMA Buffer being used 0 or 1

extern int Number;

BOOL DMARunning = FALSE;		// Used to start DMA on first write
BOOL FirstTime = FALSE;

extern int totSamples;

void InitSound()
{
}

volatile unsigned short * SendtoCard(unsigned short buf, int n)
{
  // Start DMA if first call

  totSamples += n;
	
  if (DMARunning == FALSE)
  {
    StartDAC();
    DMARunning = TRUE;
    FirstTime = TRUE;
	
    // We can immediately fill second half

    Index = 1;
    return &dac1_buffer[DAC_SAMPLES_PER_BLOCK];
  }

  // wait for other DMA buffer to finish

  // First time through we must wait till we are into the second
  //	(left < DAC_SAMPLES_PER_BLOCK)

  if (FirstTime)
  {
    FirstTime = FALSE;

    while (GetDMAPointer() >= DAC_SAMPLES_PER_BLOCK)
      txSleep(1);
  }

  // 	printtick("Start Wait");

  while (1)
  {
    int Left = GetDMAPointer();

    //	WriteDebugLog(LOGDEBUG, "Index %d Left %d", Index, Left);

    if (Index == 0)
    { // Just filled first buffer. Can return when left is less than half,
      // as then we are sending buffer 2

      if (Left > (DAC_SAMPLES_PER_BLOCK) )
        break;
    }
    else
    {
      // Just filled 2nd buffer, can return as soon as pointer is above half

      if (Left < (DAC_SAMPLES_PER_BLOCK))
        break;
    }
    txSleep(1);				// Run background while waiting
  }
  Index = !Index;
  txSleep(1);				// Run background while waiting
  //  printtick("Stop Wait");

  return &dac1_buffer[Index * DAC_SAMPLES_PER_BLOCK];
}
}


#ifdef OLED

#include <i2c_t3.h>
#include <Adafruit_SSD1306.h>



// Support for 128 * 64 OLED display on i2c 


/*// If using software SPI (the default case):
  #define OLED_MOSI   9
  #define OLED_CLK   10
  #define OLED_DC    11
  #define OLED_CS    12
  #define OLED_RESET 13
  Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
*/
// Uncomment this block to use hardware SPI
//#define OLED_DC     8
//#define OLED_CS     9
#define OLED_RESET  -1

//#define _MOSI 11
//#define _SCLK 13		// Clock moved to ALT pin as LED is on A13
//#define _MISO 12

// for spi Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);

Adafruit_SSD1306 display(OLED_RESET);		// i2c

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

uint8_t * buffer;		// OLED Image buffer
char * TXType = "TX:";

int OLEDOK = 0;		// Set if display found

extern "C"
{
  void Set8Pixels(int16_t x, int16_t y, int pixels, int16_t Colour)
  {
    int offset;
    offset = (127 - y) + 128 * (x / 8);
    buffer[offset] = pixels;
  }
   
  void mySetPixel(int16_t x, int16_t y, int16_t Colour)
  {
    int offset;
    uint8_t val;

    offset = (127 - y) + 128 * (x / 8);

    if (offset < 0 || offset > 2047)
    {
      return;
    }

    // top bit is first pixel

    val = (1 << (x & 7));
    if (Colour)
      buffer[offset] |= val;
    else
      buffer[offset] &= ~val;
  }

  void DrawAxes(int Qual, const char * Mode, const char * Type)
  {
    int y;
  
    // Draw x and y axes in centre of constallation area

    int yCenter = (ConstellationHeight - 2) / 2;
    int xCenter = (ConstellationWidth - 2) / 2;

    Set8Pixels(0, xCenter, 0x55, WHITE);
    Set8Pixels(8, xCenter, 0x55, WHITE);
    Set8Pixels(16, xCenter, 0x55, WHITE);
    Set8Pixels(24, xCenter, 0x55, WHITE);
    Set8Pixels(32, xCenter, 0x55, WHITE);
    Set8Pixels(40, xCenter, 0x55, WHITE);
    Set8Pixels(48, xCenter, 0x55, WHITE);
    Set8Pixels(56, xCenter, 0x55, WHITE);

    // y is 64 pixels from 31, 0

    for (y = 0; y < 64; y += 2)
      mySetPixel(xCenter, y, WHITE);

    display.setRotation(0);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 5);
    display.print(Mode);
    display.setCursor(0, 17);
    display.printf("QUAL %d  ", Qual);
  }

  void DrawDecode(const char * Decode)
  {
    display.setCursor(0, 28);
    display.print(Decode);
	if (TXType)
	{
		display.setCursor(0, 52);
		display.print(TXType);
	}
  }

  void DrawTXMode(char * TXMode)
  {
	TXType = TXMode;
	display.setCursor(0, 52);
	display.print("            ");	//Clear old mode
	display.setCursor(0, 52);
	display.print(TXMode);
  }

  void clearDisplay()
  {
    display.clearDisplay();
  }

  void updateDisplay()
  {
    if (OLEDOK)	
      display.display();
  }
}


void setupOLED()
{
  // Make sure the device is there, or the display code will hang

  Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_37_38, I2C_PULLUP_EXT, 400000);
  Wire1.setDefaultTimeout(10000); // 10ms

  Wire1.beginTransmission(0x3C);  // slave addr
  Wire1.endTransmission();        // no data, just addr

  switch (Wire1.status())
  {
    case I2C_WAITING:
      WriteDebugLog(6, "Found OLED display at 0x3C");
      OLEDOK = 1;
      break;
    case I2C_ADDR_NAK:
      WriteDebugLog(6, " Find OLED Error Addr: 0x%02X NAK", 0x3C);
      break;
    default:
      WriteDebugLog(6, "OLED not found at address 0x3C");
      break;
  }
  
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);
 
 // init done
  // Show image buffer on the display hardware.
  // Since the buffer is internalized with an Adafruit splash screen
  // internally, this will display the splash screen.

//  Serial.printf("Start Display %d\r\n", millis());
  updateDisplay();
//  Serial.printf("End Display %d\r\n", millis());
  delay(1000);
    
  // I can't get the drawline or pixel routines to work from C,
  // so I get the display buffer and update it myself.

  // The displays I have are 128 x 64. As this is primarily for
  // constellation display, I'll use in portrait mode, with the top
  // 64 x 64 used for the constellation and the lower area for short
  // status messages

  // Pixels from a byte seem to be plotted in the 64 direction, and if we
  // mount the display so this is left to right, it works bottom to top.
  // or for conventional (L>R, Top>Bottom) we must reverse one. I'll start
  // from 127 and work down.

  // So byte origin is 127 - y + 128 * (x/8)

  buffer = display.getDisplay();
  clearDisplay();
  
  DrawAxes(99, "16Q.200.100", "");
  DrawDecode("PASS");
//  Serial.printf("Start Display %d\r\n", millis());
  updateDisplay();
//  Serial.printf("End Display %d\r\n", millis());
}

#endif

#ifdef KMR_18

// This Teensy3 native optimized version requires specific pins
//
#define sclk 13  // SCLK can also use pin 14
//#define mosi 11  // MOSI can also use pin 7
#define cs   15  // CS & DC can use pins 2, 6, 9, 10, 15, 20, 21, 22, 23
#define dc   14   //  but certain pairs must NOT be used: 2+10, 6+9, 20+23, 21+22
#define rst  23  // RST can use any pin
//#define sdcs 4   // CS for SD card, can use any pin

#define TFT_SCLK 13  // SCLK can also use pin 14
#define TFT_MOSI 11  // MOSI can also use pin 7
#define TFT_CS   15  // CS & DC can use pins 2, 6, 9, 10, 15, 20, 21, 22, 23
#define TFT_DC    14  //  but certain pairs must NOT be used: 2+10, 6+9, 20+23, 21+22
#define TFT_RST   23  // RST can use any pin

#include <Adafruit_GFX.h>    // Core graphics library
//#include <ST7735_t3.h> // Hardware-specific library
#include <Adafruit_ST7735.h> // Hardware-specific library


#include <SPI.h>

Adafruit_ST7735 tft = Adafruit_ST7735(cs, dc, rst);

//ST7735_t3 tft = ST7735_t3(cs, dc, rst);

//ST7735 tft = ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

char * TXType = "TX:";		// Save last transmitted type

extern "C"
{
  void mySetPixel(int16_t x, int16_t y, int16_t Colour)
  {
	tft.drawPixel(x + ConsXoffset, y + ConsYoffset, Colour);
  }

  void DrawAxes(int Qual, const char * Mode, const char * XX)
  {
    // Draw x and y axes in centre of constallation area

    int yCenter = ConsYoffset + (ConstellationHeight - 2) / 2;
    int xCenter = ConsXoffset + (ConstellationWidth - 2) / 2;
	
	tft.setRotation(3);

	tft.drawFastVLine(xCenter, ConsYoffset , ConstellationHeight, Yellow);
	tft.drawFastHLine(ConsXoffset, yCenter , ConstellationWidth, Yellow);

    tft.setRotation(3);
    tft.setTextSize(1);
    tft.setTextColor(WHITE, BLACK);
    tft.setCursor(0, 0);
	tft.print("           ");	//Clear old mode
    tft.setCursor(0, 0);
    tft.print(Mode);
    tft.setCursor(0, 18);
    tft.printf("QUAL %d  ", Qual);
  }

  void DrawDecode(const char * Decode)
  {
    tft.setCursor(0, 36);
    tft.printf("%s    ", Decode);
	if (TXType)
	{
		tft.setCursor(0, 54);
		tft.print("           ");	//Clear old TXTYPE
		tft.setCursor(0, 54);
		tft.print(TXType);
	}
  }

  void DrawTXMode(char * TXMode)
  {
	TXType = TXMode;
	tft.setCursor(0, 54);
	tft.print("           ");	//Clear old txtype
	tft.setCursor(0, 54);
	tft.print(TXMode);
  }

  void clearDisplay()
  {
	  // This just has to clear constellation area
	  
	  tft.fillRect(ConsXoffset, ConsYoffset, ConstellationWidth, ConstellationHeight, ST7735_BLACK);
  }

  void updateDisplay()
  {
      // Probably not needed with TFT
  }
}

void setupKMR_18(void) {

//  pinMode(sdcs, INPUT_PULLUP);  // keep SD CS high when not using SD card

  // Our supplier changed the 1.8" display slightly after Jan 10, 2012
  // so that the alignment of the TFT had to be shifted by a few pixels
  // this just means the init code is slightly different. Check the
  // color of the tab to see which init code to try. If the display is
  // cut off or has extra 'random' pixels on the top & left, try the
  // other option!
  // If you are seeing red and green color inversion, use Black Tab

  // If your TFT's plastic wrap has a Black Tab, use the following:
#ifdef KMR_BLACKTAB
 tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
#else
#ifdef KMR_GREENTAB
  tft.initR(INITR_GREENTAB); // initialize a ST7735R chip, green tab
#else
  tft.initR(INITR_REDTAB);   // initialize a ST7735R chip, red tab
#endif
#endif

 	tft.fillRect(0, 0, 128, 160, ST7735_BLACK);
 
  	DrawAxes(99, "16Q.200.100", "");
	DrawDecode("PASS");
}


#endif

#ifdef WDTTFT

// Constallation or Waterfall display on TFT on WDT board

char * TXType = "TX:";		// Save last transmitted type

extern "C"
{
  void mySetPixel(int16_t x, int16_t y, int16_t Colour)
  {
	tft.drawPixel(x + ConsXoffset, y + ConsYoffset, Colour);
  }

  void DrawAxes(int Qual, const char * Mode, const char * xx)
  {
    // Draw x and y axes in centre of constallation area

    int yCenter = ConsYoffset + (ConstellationHeight - 2) / 2;
    int xCenter = ConsXoffset + (ConstellationWidth - 2) / 2;
	
	tft.setRotation(0);

	tft.drawFastVLine(xCenter, ConsYoffset , ConstellationHeight, WHITE);
	tft.drawFastHLine(ConsXoffset, yCenter , ConstellationWidth, WHITE);

    tft.setRotation(0);
    tft.setTextSize(2);
    tft.setTextColor(WHITE, 0);
	tft.setCursor(0, 0);
	tft.print("           ");	//Clear old mode
    tft.setCursor(0, 0);
    tft.print(Mode);
    tft.setCursor(0, 18);
    tft.printf("QUAL %d  ", Qual);
  }

  void DrawDecode(const char * Decode)
  {
   	tft.setCursor(0, 36);
	tft.print("           ");	//Clear old decode

	tft.setCursor(0, 36);
    tft.print(Decode);
	if (TXType)
	{
		tft.setCursor(0, 75);
		tft.print("           ");	//Clear old TXTYPE
		tft.setCursor(0, 75);

		tft.print(TXType);
	}
  }

  void DrawTXMode(char * TXMode)
  {
	TXType = TXMode;
	tft.setCursor(0, 75);
	tft.print("           ");	//Clear old TXMODE
	tft.setCursor(0, 75);
	tft.print(TXMode);
  }

  void clearDisplay()
  {
	  // This just has to clear constellation area
	  
	  tft.fillRect(ConsXoffset, ConsYoffset, ConstellationWidth, ConstellationHeight, ILI9341_BLACK);
  }

  void updateDisplay()
  {
      // Probably not needed with TFT
  }
}


void setupWDTTFT()
{
	// Board has already been initialised
	
	clearDisplay();  
//	Serial.printf("Start Display %d\r\n", millis());
  	DrawAxes(99, "16Q.200.100", "");
	DrawDecode("PASS");
//	Serial.printf("End Display %d\r\n", millis());
}

#endif //WDTTFT


extern "C" void DrawRXFrame(int State, char * Frame)
{
#ifdef KMR_18
    if (State == 0)
		tft.setTextColor(Yellow, BLACK);
    else if (State == 1)
		tft.setTextColor(Green, BLACK);
    else if (State == 2)
		tft.setTextColor(Red, BLACK);

    tft.setCursor(0, 110);
    tft.print("Rx:                       ");	//Clear old mode
    tft.setCursor(20, 110);
    tft.print(Frame);
#endif
}

extern "C" BOOL KeyPTT(BOOL blnPTT)
  {
    digitalWriteFast(pttPin, blnPTT);
	
#ifdef WDTTFT

	// Set PTT indicator on bottom row of FTFif
	
	if (blnPTT)
		 tft.fillRect(300, 220, 20, 20, ILI9341_RED);
	else
		tft.fillRect(300, 220, 20, 20,  ILI9341_GREEN);
	 
#endif
		
    return TRUE;
  }














