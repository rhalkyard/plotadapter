#pragma once

// Size of GPIB transmit buffer
#define TXBUF_SZ 256

// Timeout for GPIB operations. This has to be relatively long to account for
// time-consuming operations such as writing labels or slow pen movements. If a
// timeout is reached, the GPIB transmit buffer is flushed and the bus reset.
#define GPIB_TIMEOUT 30000

// Length of argument buffer for escape sequences. 80 bytes is probably overkill
#define ARGBUF_LEN 80

// GPIB address of this adapter
#define MY_ADDR 0

// GPIB address of plotter. Must match DIP switches on plotter
#define PLOTTER_ADDR 5

// Pin to use for RS232 DTR line. Only useful if using a separate serial adapter
//#define DTR_PIN 4

// If true, invert DTR to be active-low
//#define DTR_INVERT false

// Define HANDSHAKE_DEBUG to add an ESC . P escape sequence that prints out the
// current handshake configuration
//#define HANDSHAKE_DEBUG

// GPIB pin definitions
#define DIO1    50  // Pin 1  on GPIB connector
#define DIO2    48  // Pin 2  on GPIB connector
#define DIO3    46  // Pin 3  on GPIB connector
#define DIO4    44  // Pin 4  on GPIB connector
#define DIO5    42  // Pin 13 on GPIB connector
#define DIO6    40  // Pin 14 on GPIB connector
#define DIO7    38  // Pin 15 on GPIB connector
#define DIO8    36  // Pin 16 on GPIB connector

#define EOI     47  // Pin 5  on GPIB connector
#define DAV     49  // Pin 6  on GPIB connector
#define NRFD    51  // Pin 7  on GPIB connector
#define NDAC    53  // Pin 8  on GPIB connector
#define IFC     35  // Pin 9  on GPIB connector
#define SRQ     41  // Pin 10 on GPIB connector - not used
#define ATN     39  // Pin 11 on GPIB connector
#define REN     37  // Pin 17 on GPIB connector - not used


#if defined(DTR_PIN)
#if DTR_INVERT
#define DTR_TRUE LOW
#define DTR_FALSE HIGH
#else
#define DTR_TRUE HIGH
#define DTR_FALSE LOW
#endif
#endif