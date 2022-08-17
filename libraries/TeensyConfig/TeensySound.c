//
//	Passes audio samples to and from the sound interface

//	This is the Arduino/Teensy Version, using DMA to DAC/ADC

//	Also has some platform specific routines

#include "TeensyConfig.h"
#include "TeensyCommon.h"

//#include "..\..\ARDOPC.h"
//#include <math.h>

extern int SoundIsPlaying;

extern int VRef;

extern BOOL blnDISCRepeating;

// the ADC DMA saves the incoming ADC samples into these 2 buffers

extern volatile unsigned short dac1_buffer[DAC_SAMPLES_PER_BLOCK * 2];

extern int ADCInterrupts;


// Windows and Linux work with signed samples +- 32767
// STM32 DAC uses unsigned 0 - 4095
// TEENSY DAC uses unsigned 0 - 4095
// TEENSY ADC uses unsigned 0 - 65535

// the ADC DMA saves the incoming ADC samples into these 2 buffers

unsigned short ADC_Buffer[2][ADC_SAMPLES_PER_BLOCK] = {0};	// Two Transfer/DMA buffers of 0.1 Sec
unsigned short work;

int maxlevel, minlevel, tot;

BOOL Loopback = FALSE;
//BOOL Loopback = TRUE;

char CaptureDevice[] = "ADC";
char PlaybackDevice[] = "DAC";

char * CaptureDevices = CaptureDevice;
char * PlaybackDevices = PlaybackDevice;

int ARDOPMode = 1;

int LastNow;

int Numbertosend = 0;				// Number waiting to be sent
int totSamples = 0;
extern int Capturing ;

unsigned short * DMABuffer = &dac1_buffer[0];
volatile int dmaints = 0;
volatile int samplessent = 0;
volatile int samplesqueued = 0;

extern int Index;				// DMA Buffer being used 0 or 1
extern int inIndex;			// DMA Buffer being used 0 or 1

extern int Number;

extern BOOL DMARunning;		// Used to start DMA on first write
extern BOOL FirstTime;


//		// This generates a nice musical pattern for sound interface testing
//    for (t = 0; t < sizeof(buffer); ++t)
//        buffer[t] =((((t * (t >> 8 | t >> 9) & 46 & t >> 8)) ^ (t & t >> 13 | t >> 6)) & 0xFF);

int lastmin = 0, lastmax = 0;
int Samples, levelticks = 0;

void PollReceivedSamples()
{
  int Pointer = GetADCDMAPointer();
   
  if (SoundIsPlaying)
    return;

  if (inIndex == 0)
  {
    if (Pointer > ADC_SAMPLES_PER_BLOCK)
      return;								// Still reading into first half
  }
  else
  {
    if (Pointer < ADC_SAMPLES_PER_BLOCK)
      return;

  }
  // convert the saved ADC 16-bit unsigned samples into 16-bit signed samples
  {
    unsigned short  *src = (unsigned short *)&ADC_Buffer[inIndex];	// point to the DMA buffer where the ADC samples were saved
    short  *dst = (unsigned short *)src;				// reuse input buffer

    int i;

    for (i = 0; i < ADC_SAMPLES_PER_BLOCK; i++)
    {
      register int s1 = (unsigned short)(*src++);
      s1 -= VRef;
      *dst++ = s1;
      tot += s1;
      if (s1 > maxlevel)
        maxlevel = s1;
      if (s1 < minlevel)
        minlevel = s1;
    }

    Samples += ADC_SAMPLES_PER_BLOCK;

    //  printtick("Process Sample Start");
	
	#ifdef SOUNDCARDPACKET
	
		// Pass samples to both
		
		ProcessNewSamples(&ADC_Buffer[inIndex], ADC_SAMPLES_PER_BLOCK);
		pktProcessNewSamples(&ADC_Buffer[inIndex], ADC_SAMPLES_PER_BLOCK);
		
	#else

	if (ARDOPMode)
		ProcessNewSamples(&ADC_Buffer[inIndex], ADC_SAMPLES_PER_BLOCK);
	else
		pktProcessNewSamples(&ADC_Buffer[inIndex], ADC_SAMPLES_PER_BLOCK);
	
	#endif
	
    //  printtick("Process Sample End");

    // We save Max and Min every block so we can adjust level
    // when we see a valid packet. We display every 10 seconds

    lastmin = minlevel;
    lastmax = maxlevel;

    if (Now - levelticks > 9999)
    {
      	char HostCmd[64];
		
		levelticks = Now;
		WriteDebugLog(LOGDEBUG, "Input peaks %d %d average %d", maxlevel, minlevel, tot / Samples);
		sprintf(HostCmd, "INPUTPEAKS %d %d", minlevel, maxlevel);
#ifdef ARDOP
		QueueCommandToHost(HostCmd);
#endif
		displayLevel(maxlevel);

		// Adjust VRef

//      VRef += tot / Samples;
      
      Samples = tot = 0;

      CheckandAdjustRXLevel(maxlevel, minlevel, FALSE);
    }
    minlevel = maxlevel = 0;
  }
  inIndex = !inIndex;
}

void StartCodec(char * strFault)
{
  strFault[0] = 0;
}

void StopCodec(char * strFault)
{
  strFault[0] = 0;
}


void CloseSound()
{
}

unsigned short * SoundInit()
{
  Index = 0;
  return &dac1_buffer[0];
}

//	Called at end of transmission

extern int Number;				// Number of samples waiting to be sent

void SoundFlush()
{
  int FlushEnd;

  // Append Trailer then send remaining samples

  AddTrailer();			// add the trailer.
  
  // Index= 0, Number = 0 causes a nasty boundary condition. Simplest is to make sure it isn't
  
  // Actually, as there is a 1ms sleep between tests, anything less than about 50 (at 48K) could cause
  // a problem
  
  if (Index == 0 && Number < 50)
	Number = 50;

//  printtick("Start flush");

  //  WriteDebugLog (LOGDEBUG, "Flush Index = %d Number = %d Left = %d", Index, Number, GetDMAPointer());

  if (Index == 0)

    //	Sending from first half of buffer. Stop when DMA Pointer gets to
    //	(2 * DAC_SAMPLES_PER_BLOCK) - Number

    // Remember DMA pointer goes downwards

    FlushEnd = (2 * DAC_SAMPLES_PER_BLOCK) - Number;

  else

    // Second Half. Stop when pointer gets to DAC_SAMPLES_PER_BLOCK - Number)
    FlushEnd = DAC_SAMPLES_PER_BLOCK - Number;

  // WriteDebugLog (LOGDEBUG, "Flush Index = %d Number = %d Left = %d Flushend %d", Index, Number, GetDMAPointer(), FlushEnd);

  // Wait if necessary for the other half to complete

  while (GetDMAPointer() < FlushEnd)
    txSleep(1);

 // printtick("mid flush");

  // Wait for this (partial) half to complete

  while (GetDMAPointer() > FlushEnd)
    txSleep(1);

  // printtick("end flush");

  stopDAC();
  DMARunning = FALSE;
  SoundIsPlaying = FALSE;

	TurnroundLink();					// Mode Specific Code
	
  KeyPTT(FALSE);		 // Unkey the Transmitter

  StartCapture();

  WriteDebugLog(7, "totSamples %d", totSamples);
  totSamples = 0;
  Number = 0;
  DMABuffer = &dac1_buffer[0];

  return;
}





