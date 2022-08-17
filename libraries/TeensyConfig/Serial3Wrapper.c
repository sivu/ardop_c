#include "TeensyConfig.h"

// This seems to be the only way to change the serial port buffer size
// without editing the IDE core file. We set the equates then include the
// core library source

// We need bigger buffers if we want to use a hardware serial port for the host
// interface (to avoid buffering delays)

// We need to be able to queue a full KISS or Hostmode frame to the host without
// waiting


#ifdef SERIAL3SIZE
#define SERIAL3_TX_BUFFER_SIZE	SERIAL3SIZE // number of outgoing bytes to buffer
#define SERIAL3_RX_BUFFER_SIZE	SERIAL3SIZE
#include "serial3.c"
#endif

