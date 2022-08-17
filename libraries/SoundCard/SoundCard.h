#include "usb_dev.h"

//  At 48000 sample rate USB requests 48 samples of 4 bytes per interrupt (1000 interupts/sec)
//	We need 25 buffers for each 1200 samples from ADC. 32 gives a bit of marging for slow pc

#undef AUDIO_BLOCK_SAMPLES
#define AUDIO_BLOCK_SAMPLES 192

#define POOLCOUNT 32

#define DAC_SAMPLES_PER_BLOCK 1200

//#define MONO

#ifdef MONO
#define CHANNELS 1
#else
#define CHANNELS 2
#endif


typedef struct my_audio_block_struct
{
	struct my_audio_block_struct * chain;
	short data[AUDIO_BLOCK_SAMPLES];
} my_audio_block_t;



extern volatile int rxtot;
extern volatile int txtot;
extern volatile int inInts;
extern volatile int outInts; 

extern volatile int q_count;

extern my_audio_block_t pool[POOLCOUNT];
extern my_audio_block_t free_q;
extern my_audio_block_t tohost_q;


extern volatile unsigned short dac1_buffer[DAC_SAMPLES_PER_BLOCK * 2];


struct my_audio_block_struct;



