//	Teensy Soundcard emulator. Presents ADC and DAC to host as a SoundCard.

//	I don't need the overhead of the Audio Library, so I'll copy the interupt routines from
//	usb_audio.cpp and add my own wrapper
//	I see two uses for this. One combined with ARDOP (or possibly Packet) so host can run a 
//  Soundcard mode (eg WINMOR) in parallel with ARDOP/

//	Or a simple passthough (a "Signalink emulator").

//	Sample rates may be a problem. WINMOR uses 48000, but ARDOP 12000. Will PC tell us what
//  it wants???

//	What happens if we try to run ARDOP at 48000???

/* Teensyduino Core Library
   http://www.pjrc.com/teensy/
   Copyright (c) 2016 PJRC.COM, LLC.

   Permission is hereby granted, free of charge, to any person obtaining
   a copy of this software and associated documentation files (the
   "Software"), to deal in the Software without restriction, including
   without limitation the rights to use, copy, modify, merge, publish,
   distribute, sublicense, and/or sell copies of the Software, and to
   permit persons to whom the Software is furnished to do so, subject to
   the following conditions:

   1. The above copyright notice and this permission notice shall be
   included in all copies or substantial portions of the Software.

   2. If the Software is incorporated into a build system that allows
   selection among a list of target devices, then similar target
   devices manufactured by PJRC.COM must be included in the list of
   target devices and selectable in the same manner.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
   EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
   MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
   BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
   ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
*/

#include "SoundCard.h"
#include "TeensyConfig.h"
#include "TeensyCommon.h"

#include "usb_dev.h"
#include <string.h> // for memcpy()


#define __disable_irq() __asm__ volatile("CPSID i":::"memory");
#define __enable_irq()	__asm__ volatile("CPSIE i":::"memory");

#define WDOG_REFRESH		(*(volatile uint16_t *)0x4005200C) // Watchdog Refresh register


my_audio_block_t * q_rem(my_audio_block_t *Q);
my_audio_block_t * allocate(void);

#ifdef AUDIO_INTERFACE // defined by usb_dev.h -> usb_desc.h
#else
#error ("USB Audio Not Enabled") 
#endif // AUDIO_INTERFACE

int SampleRate = 48000;

int SoundIsPlaying = 0;

volatile int pttActive = 0;				// Transmitting Flag/Timer
volatile int dacActive = 0;				// Transmitting Flag/Timer

volatile int timestarted = 0;
extern volatile int samplesqueued;
extern volatile int samplessent;

extern unsigned short * DMABuffer ;

volatile int Number = 0;


struct setup_struct {
  union {
    struct {
	uint8_t bmRequestType;
	uint8_t bRequest;
	union {
		struct {
			uint8_t bChannel;  // 0=main, 1=left, 2=right
			uint8_t bCS;       // Control Selector
		};
		uint16_t wValue;
	};
	union {
		struct {
			uint8_t bIfEp;     // type of entity
			uint8_t bEntityId; // UnitID, TerminalID, etc.
		};
		uint16_t wIndex;
	};
	uint16_t wLength;
    };
  };
};


// audio features supported

struct usb_audio_features_struct
{
  int change;  // set to 1 when any value is changed
  int mute;    // 1=mute, 0=unmute
  int volume;  // volume from 0 to FEATURE_MAX_VOLUME, maybe should be float from 0.0 to 1.0
};

struct usb_audio_features_struct features = {0, 0, 25};		// Defualt level about 300 mV

int lastTicks = 0;

void  SoundCardBG()
{
	int Ticks = getTicks();

	if ((Ticks - lastTicks) > 999)
	{
		lastTicks = Ticks;
		debugprintf("in Ints %d Out ints %d Q %d tot rx %d tot tx %d queued %d", inInts, outInts, q_count, rxtot / 4, txtot / 4, samplesqueued - samplessent);
		inInts = outInts = rxtot = txtot = 0;
		__disable_irq();
		WDOG_REFRESH = 0xA602;
		WDOG_REFRESH = 0xB480;
		__enable_irq();
	}

	PollReceivedSamples();
	
	if (features.change == 1)		// TX Level (or Mute) changed
	{
		features.change = 0;
		SetPot(1, features.volume);				// Write to live pt (should we save in NVRAM??)
	}
		
	if (dacActive)
	{
		if (samplessent >= samplesqueued)		// Not really fast enough - maybe count millisecs since start
		{
			 KeyPTT(0);		// Drop PTT
			 dacActive = 0;
			 pttActive = 0;
			 stopDAC();
		}
	}
}


// Called from the USB interrupt when an isochronous packet arrives
// we must completely remove it from the receive buffer before returning

// We need to detect start and end of transmission so we can key ptt (could derive from samples but dont see the need)
// We can also start/stop DAC (same as for ARDOP etc) at the same time.
// To provide a bit if buffering don't start TX until we have 10 ms worth of data, but key ptt immediately
// At end must delay dropping PTT until buffer is empty (sent > queued)


void usb_audio_receive_callback(unsigned int len)
{
  int32_t value;
  int i = 0;
  #ifdef MONO
   int count = len /2;		//Len is in bytes, 1 bytes per samples (1 chan 16 bit)
  #else
 int count = len /4;		//Len is in bytes, 4 bytes per samples (2 chan 16 bit)
  #endif
  
  // Check for silence. Some programs send silence instead of stopping sending
  // But there may be low level noise (+= 3)
  int silent = TRUE;
  
  
  for (i = 0; i < count; i++)
  {
	value = usb_audio_receive_buffer[i * CHANNELS];
	if (value > 3 && value < 65534)
		silent = FALSE;
	
	dac1_buffer[Number++] = (value + 32768) >> 4;
  }
  
  if (silent)
  {
	  Number -= count;
	  return;
  }
	  
  
  if (pttActive == 0)
  {
	  // Starting TX
	  
	    if (features.mute == 0)
			KeyPTT(1);		// Raise PTT	unless muted
		pttActive =1;
		samplessent = 0;
		samplesqueued = 0;
		Number = 0;
  }
  
  inInts++;
  rxtot += len;
  samplesqueued += count;
  
/*
  if (Number == 1200)
  {
	  debugprintf("%d %d %d %d %d %d %d %d ", 
	  usb_audio_receive_buffer[0],
	  usb_audio_receive_buffer[2],
	  usb_audio_receive_buffer[4],
	  usb_audio_receive_buffer[6],
	  usb_audio_receive_buffer[8],
	  usb_audio_receive_buffer[10],
	  usb_audio_receive_buffer[12],
	  usb_audio_receive_buffer[14]);
  }
*/  
  if (Number == 2400)
  {
	// Just loop. DMA sends buffer continuously. Will need sync updates
		
	Number = 0;
  }	
 
  if (dacActive == 0)
  {
	  // Start dac when we've queued enough to give a bit of buffering
	  
	  if (Number > 960) 	// 20 mS
	  {
	      StartDAC();
		  dacActive = 1;
		 // timestarted = millis();
	  }
  }
  
  return;

}


// Called from the USB interrupt when ready to transmit another
// isochronous packet.  If we place data into the transmit buffer,
// the return is the number of bytes.  Otherwise, return 0 means
// no data to transmit

// Called every mS

unsigned int usb_audio_transmit_callback(void)
{
  static uint32_t count = 5;
  uint32_t avail, num, target, offset, len = 0;
  my_audio_block_t *buff;

	outInts++;

	target = 2 * CHANNELS * SampleRate / 1000;		// Called every Millisecond
	
	buff = q_rem(&tohost_q);	// nothing on free_q so get first from tohost_q
	if (buff == NULL)
		return 192;

	memcpy(usb_audio_transmit_buffer, &buff->data[0], target);
	
	release(buff);
	
	txtot += target;

	return target;
}

int usb_audio_set_feature(void *stp, uint8_t *buf)
{
	struct setup_struct setup = *((struct setup_struct *)stp);

	debugprintf("set feature %x %x %x %x %x %x %x %x %x", setup.bmRequestType, setup.bCS,
			setup.bRequest, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

	if (setup.bmRequestType == 0x21)
	{ // should check bRequest, bChannel and UnitID
		if (setup.bCS == 0x01)
		{ // mute
			if (setup.bRequest == 0x01)
			{ // SET_CUR
        features.mute = buf[0]; // 1=mute,0=unmute
        features.change = 1;
        return 1;
      }
    }
    else if (setup.bCS == 0x02) { // volume
      if (setup.bRequest == 0x01) { // SET_CUR
        features.volume = buf[0] + (buf[1] << 8);
        features.change = 1;
        return 1;
      }
    }
  }
  return 0;
}

int usb_audio_get_feature(void *stp, uint8_t *data, uint32_t *datalen)
{
  struct setup_struct setup = *((struct setup_struct *)stp);

  debugprintf("get feature %x %x %x", setup.bmRequestType, setup.bCS, setup.bRequest);

  if (setup.bmRequestType == 0xA1)
  { // should check bRequest, bChannel, and UnitID
    if (setup.bCS == 0x01) 
	{ // mute
      data[0] = features.mute;  // 1=mute, 0=unmute
      *datalen = 1;
      return 1;
    }
    else if (setup.bCS == 0x02)
	{ // volume
		if (setup.bRequest == 0x81)
		{ // GET_CURR
			data[0] = features.volume & 0xFF;
			data[1] = 0;
		}
		else if (setup.bRequest == 0x82) 
		{ // GET_MIN
			//serial_print("vol get_min\n");
			data[0] = 0;     // min level is 0
			data[1] = 0;
		}
		else if (setup.bRequest == 0x83) 
		{ // GET_MAX
			data[0] = 0xFF;  // max level, for range of 0 to 255
			data[1] = 0;
		}
		else if (setup.bRequest == 0x84) 
		{ // GET_RES
			data[0] = 1; // increment vol by by 1
			data[1] = 0;
		}
		else 
		{ // pass over SET_MEM, etc.
			return 0;
		}
		*datalen = 2;
		return 1;
    }
  }
  return 0;
}






my_audio_block_t * allocate(void)
{
  __disable_irq();

  my_audio_block_t * block = free_q.chain;

  if (block == NULL)
  {
    __enable_irq();
    return NULL;
  }

  free_q.chain = block->chain;
  q_count--;

  __enable_irq();
  return block;
}

void release(my_audio_block_t *block)
{
  __disable_irq();
  block->chain = free_q.chain;
  free_q.chain = block;
  q_count++;
  __enable_irq();
}

my_audio_block_t * q_rem(my_audio_block_t *Q)
{
	my_audio_block_t * first;
	my_audio_block_t * next;

  __disable_irq();
	first = Q->chain;

	if (first == 0)
	{
		__enable_irq();
		return (0);			// Empty
	}
	next = first->chain;					// Address of next buffer

	Q->chain = next;
	
	__enable_irq();


	return (first);
}

int q_add(my_audio_block_t * Q, my_audio_block_t * BUFF)
{
	my_audio_block_t * next;

	BUFF->chain = 0;					// Clear chain in new buffer

	__disable_irq();

	if (Q->chain == NULL)				// Empty
	{
		Q->chain = BUFF;				// New one on front
		__enable_irq();
		return(0);
	}

	next = Q->chain;

	while (next->chain != 0)
	{
		next=next->chain;				// Chain to end of queue
	}
	next->chain = BUFF;					// New one on end
	__enable_irq();

	return(0);
}






// ADC/DAC drivers work in blocks of 1200 samples = 25ms at 48000 sample rate.

// USB requests a block every millisecond and needs 48 samples (192 bytes, 16 but L/R)

// I think I'll use buffers of 48 samples, so will need 25 by ADC block

//	I don't know yet how to sync if pc is faster or slower than Teensy Clock.
//	Can I return a double block or return 0? Will try....




void ProcessNewSamples(short * Samples, int nSamples)
{
	// copy to 25 buffers
	
	int i, j, k = 0;;
	short sample;
	short * ptr;
	
	my_audio_block_t * buff;
	
	if (q_count < 25)
	{
//		debugprintf("Insufficient buffers\n");
		return;
	}
	
	for (i = 0; i< 25; i++)
	{
		buff = allocate();				// get buffer from free_q
			
		if (buff == NULL)
		{
			buff = q_rem(&tohost_q);	// nothing on free_q so get first from tohost_q
			if (buff == NULL)
				return;
		}
		
		ptr = &buff->data[0];
				
		for (j = 0; j< 48; j++)
		{
			*(ptr++) = Samples[k];
#ifndef MONO
			*(ptr++) = Samples[k++];			// Two channels
#endif
		}
		q_add(&tohost_q, buff);
	}
}

extern int totSamples;
int Capturing = 0;


#define DCDLED LED0
#define PKTLED LED3		// flash when packet received


 void SampleSink(short Sample)
  {
    int work = (short)Sample;

    DMABuffer[Number++] = (work + 32768) >> 4; // 12 bit left justify

    if (Number == SendSize)
    {
      // send this buffer to sound interface

      //  printtick("Enter SendtoCard");
      DMABuffer = SendtoCard(DMABuffer, SendSize);
      //  printtick("Leave SendtoCard");
      Number = 0;
    }
    totSamples++;
  }


  int OKtoAdjustLevel()
  {
    // Only auto adjust level when disconnected.
    // Level is set at end of each received packet when connected

    return 1;
  }

  void TurnroundLink()
  {
  }

  int AddTrailer()
  {
    return 0;
  }

  int displayDCD(int state)
  {
    SetLED(DCDLED, state);
    return 1;
  }

  void StopCapture()
  {
    Capturing = FALSE;
  }

  void StartCapture()
  {
    Capturing = TRUE;

    //	printf("Start Capture\n");
  }
  
  
  void _getpid()		// Seem to be needed to satiSfy linker
  {
  }

  void _kill()
  {
  }



