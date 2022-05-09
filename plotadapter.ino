#include <Arduino.h>

/*** FreeRTOS headers - requires FreeRTOS library from Library Manager ***/
#include <Arduino_FreeRTOS.h>
#include <stream_buffer.h>

#include "config.h"
#include "gpib.h"

#define CH_ETX '\x03'
#define CH_ESC '\x1b'

enum esc_state_t { ESC_IDLE, ESC_EXPECT_DOT, ESC_EXPECT_CMD, ESC_EXPECT_ARGS };

enum echo_mode_t { ECHO_OFF, ECHO_IMMEDIATE, ECHO_SENT };

/*** Serial interface parameters, configured with escape sequences ***/
/* ESC . @ */
bool dtr_mode = false;
enum echo_mode_t echo_mode = ECHO_OFF;
/* ESC . H, ESC . I */
int handshake_mode = 1;
int block_size = 80;
char enq_char = '\0';
char ack_xon[11] = "";
/* ESC . M */
int turnaround_delay = 0;     // Not used
char output_trigger = '\0';   // Not used
char echo_terminator = '\0';  // Not used
char output_terminator[3] = "\r";
char output_initiator = '\0';
/* ESC . N */
int interchar_delay = 0;  // Not used
char imm_xoff[11] = "";
/* ESC . (, ESC . ) */
bool plotter_enable = true;

/* Buffer for escape-sequence arguments */
char argbuf[ARGBUF_LEN];

/* Terminator character for LB command. Set with DT command. Default is ETX */
char label_terminator = CH_ETX;

/* Handle for GPIB task */
TaskHandle_t gpib_taskhandle = NULL;

/* GPIB transmit buffer */
StreamBufferHandle_t txBuf = NULL;

/* GPIB interface task.

   Reads bytes from GPIB TX buffer and outputs them over GPIB. Does some
   rudimentary command-stream parsing to keep track of commands that might
   generate output.
*/
void gpibTask(void *pvParams) {
  char cmd[3] = {'\0', '\0', '\0'}; /* Command currently being processed */
  bool in_label = false;            /* Are we currently writing a label? */
  bool talking = false;             /* Are we currently talking? */

restart:
  /* We restart from here if a timeout is reached */
  cmd[0] = '\0';
  cmd[1] = '\0';
  cmd[2] = '\0';
  in_label = false;
  talking = false;
  xStreamBufferReset(txBuf);
  gpib_init();

  while (1) {
    uint8_t ch = '\0';
    while (xStreamBufferBytesAvailable(txBuf)) {
      /* Get a byte from the GPIB TX buffer */
      xStreamBufferReceive(txBuf, &ch, 1, 0);

      /* Become talker if necessary */
      if (!talking) {
        /* Stop any currently-active bus transaction */
        if (gpib_stop(GPIB_TIMEOUT)) {
          goto restart;
        }

        /* Start new bus transaction with adapter as talker and plotter
         * as listener */
        if (gpib_start(MY_ADDR, PLOTTER_ADDR, GPIB_TIMEOUT) == -1) {
          goto restart;
        }

        talking = true;
      }

      /* Handle argument to previously-received DT command (set new
       * label-terminator character). We have to do this now because we
       * need to get the argument, and DT does not obey normal
       * command-termination rules. */
      if (!strcmp(cmd, "DT")) {
        label_terminator = ch;
      }

      /* Check if we are beginning a new command */
      if (!strcmp(cmd, "LB")) {
        /* Inside a label, commands are only terminated with the
         * label_terminator character set by the DT command (ETX by
         * default) */
        if (ch == label_terminator) {
          cmd[0] = '\0';
          cmd[1] = '\0';
        }
      } else {
        /* Outside of labels, many commands are terminated by anything
         * that isn't a comma, digit or whitespace */
        if (cmd[0] && cmd[1]) {
          if (!(isdigit(ch) || ch == ',' || ch == ' ')) {
            cmd[0] = '\0';
            cmd[1] = '\0';
          }
        }
      }

      /* Get command characters */
      if (cmd[0] == '\0') {
        if (isalpha(ch)) {
          cmd[0] = toupper(ch);
        }
      } else if (cmd[1] == '\0') {
        if (isalpha(ch)) {
          cmd[1] = toupper(ch);
        }
      }

      /* IN and DF commands reset label terminator to ETX */
      if (!strcmp(cmd, "IN") || !strcmp(cmd, "DF")) {
        label_terminator = CH_ETX;
      }

      /* Write byte to GPIB. Don't bother using EOI, the 7470 doesn't seem
       * to care about it */
      if (gpib_write(ch, false, false, GPIB_TIMEOUT) == -1) {
        goto restart;
      }

      /* If we are in "delay echo until character processed" echo mode,
       * echo the character back now that we have sent it */
      if (echo_mode == ECHO_SENT) {
        Serial.write(ch);
      }

      /* If we just sent an output command, break out of the
       * transmit loop and wait for a response */
      if (cmd[1] && toupper(cmd[0] == 'O')) {
        /* Ask plotter to talk */
        talking = false;

        /* Stop any currently-active bus transaction */
        if (gpib_stop(GPIB_TIMEOUT) == -1) {
          goto restart;
        }

        /* Lock control lines so that plotter doesn't start talking early */
        SET(NRFD);
        CLEAR(NDAC);

        /* Start new bus transaction with adapter as talker and plotter
         * as listener */
        if (gpib_start(PLOTTER_ADDR, MY_ADDR, GPIB_TIMEOUT) == -1) {
          goto restart;
        }

        /* Send optional output-initiator string */
        if (output_initiator) {
          Serial.print(output_initiator);
        }

        /* Read response from plotter and send to serial port */
        int ret;
        do {
          ret = gpib_read(&ch, 1000);
          if (ret == -1) {
            goto restart;
          } else {
            if (ch != '\r' && ch != '\n') {
              /* Suppress CRLF output terminator; we send our own */
              Serial.write(ch);
            }
          }
        } while (ret == 0);

        /* Send output-terminator string (CR by default) */
        Serial.print(output_terminator);
        Serial.flush();
      }

      vTaskDelay(0);
    }
    vTaskDelay(0);
  }
}

/* Read a semicolon-delimited list of integers from argbuf, return them as an
 * array in outval. Blank/unparseable values are replaced with -1. */
int read_args(char *argbuf, int *outvals, int nvals) {
  int i, val;
  char *current = argbuf;
  char *tok_end;

  for (i = 0; i < nvals; i++) {
    outvals[i] = -1;
  }

  i = 0;
  while (current != NULL && i < nvals) {
    val = strtod(current, &tok_end);
    if (tok_end == current) {
      outvals[i] = -1;
    } else {
      outvals[i] = val;
    }
    i++;
    current = strchr(tok_end, ';');
    if (*current != '\0') {
      current++;
    } else {
      current = NULL;
    }
  }

  return i;
}

/* ESC . @ [<ignored>; <options>] :

  Set plotter configuration

   First argument is ignored. Second is interpreted as a bitfield as follows:

    0: 0 = hold DTR high, 1= use DTR for flow control

    1: Unused

    2: 0 = immediate echo, 1 = delayed echo

    3: 0 = disable echo, 1 = enable echo

    4-7: Unused

  "Immediate echo" mode echoes characters as soon as they are received. "Delayed
  echo" mode echoes characters only after they have been accepted by the
  plotter.
*/
bool handle_config(char ch) {
  const int max_args = 2;
  static int i = 0;
  if (ch != ':' && i < ARGBUF_LEN) {
    argbuf[i++] = ch;
    return true;
  } else {
    int args[max_args];
    int nargs;
    argbuf[i] = '\0';
    nargs = read_args(argbuf, args, max_args);

    /* First argument is ignored */

    /* Second argument controls DTR pin and echo mode */
    if (args[1] == -1) {
      args[1] = 0;
    }

    dtr_mode = args[1] & 0b0001;
#ifdef DTR_PIN
    /* If we disable DTR flow control, permanently assert DTR */
    if (!dtr_mode) {
      digitalWrite(DTR_PIN, DTR_TRUE);
    }
#endif

    if (args[1] & 0b1000) {
      if (args[1] & 0b0100) {
        echo_mode = ECHO_SENT;
      } else {
        echo_mode = ECHO_IMMEDIATE;
      }
    } else {
      echo_mode = ECHO_OFF;
    }

    i = 0;
    return false;
  }
}

/* ESC . H [<block-size>; <enq-char>; <ack-char> [; ...<ack-char>]] :

   Set Handshake Mode 1

   Argument 1: Block size

   Argument 2: Character to expect for enquiry

   Argument 3..12: Null-terminated string of up to 10 characters to send as
   acknowledgement
*/
bool handle_handshake_1(char ch) {
  const int max_args = 12;
  static int i = 0;
  if (ch != ':' && i < ARGBUF_LEN) {
    argbuf[i++] = ch;
    return true;
  } else {
    int args[max_args];
    int nargs;
    argbuf[i] = '\0';
    nargs = read_args(argbuf, args, max_args);

    handshake_mode = 1;

    if (args[0] == -1) {
      block_size = 80;
    } else {
      block_size = args[0];
    }

    if (args[1] == -1) {
      enq_char = '\0';
    } else {
      enq_char = args[1];
    }

    for (int j = 2; j < max_args; j++) {
      if (args[j] == -1) {
        ack_xon[j - 2] = '\0';
      } else {
        ack_xon[j - 2] = args[j];
      }
    }
    i = 0;
    return false;
  }
}

/* ESC . I [<block-size>; <enq-char>; <ack-char> [; ...<ack-char>]] :

  Set Handshake Mode 2

  Arguments as per Handshake Mode 1
*/
bool handle_handshake_2(char ch) {
  const int max_args = 12;
  static int i = 0;
  if (ch != ':' && i < ARGBUF_LEN) {
    argbuf[i++] = ch;
    return true;
  } else {
    int args[max_args];
    int nargs;
    argbuf[i] = '\0';
    nargs = read_args(argbuf, args, max_args);

    handshake_mode = 2;

    if (args[0] == -1) {
      block_size = 80;
    } else {
      block_size = args[0];
    }

    if (args[1] == -1) {
      enq_char = '\0';
    } else {
      enq_char = args[1];
    }

    for (int j = 2; j < max_args; j++) {
      if (args[j] == -1) {
        ack_xon[j - 2] = '\0';
      } else {
        ack_xon[j - 2] = args[j];
      }
    }

    i = 0;
    return false;
  }
}

/* ESC . M [<turnaround-delay>; <trigger-char>; <echo-term-char>;
   <output-term-char1>; <output-term-char2>; <output-initiator-char>] :

   Set Output Mode

   Argument 1: turnaround delay before response = ((param * 1.1875) % 65536) /
   1.2 ms

   Argument 2: output trigger character

   Argument 3: Echo terminator character

   Argument 4,5: Output terminator characters

   Argument 6: Output initiator character
*/
bool handle_output_mode(char ch) {
  const int max_args = 6;
  static int i = 0;
  if (ch != ':' && i < ARGBUF_LEN) {
    argbuf[i++] = ch;
    return true;
  } else {
    int args[max_args];
    int nargs;
    argbuf[i] = '\0';
    nargs = read_args(argbuf, args, max_args);

    if (args[0] == -1) {
      turnaround_delay = 0;
    } else {
      turnaround_delay = args[0];
    }

    if (args[1] == -1) {
      output_trigger = '\0';
    } else {
      output_trigger = args[1];
    }

    if (args[2] == -1) {
      echo_terminator = '\0';
    } else {
      echo_terminator = args[2];
    }

    for (int j = 3; j < 5; j++) {
      if (args[j] == -1) {
        output_terminator[j - 3] = '\0';
      } else {
        output_terminator[j - 3] = args[j];
      }
    }

    if (args[5] == -1) {
      output_initiator = '\0';
    } else {
      output_initiator = args[5];
    }

    i = 0;
    return false;
  }
}

/* ESC . N [<interchar-delay>; <immediate-response-string>... ] :

  Set extended handshake options

  Argument 1: Inter-character delay = ((param * 1.1875) % 65536) / 1.2 ms

  Argument 2-11: Immediate-response string to send when ENQ character is
  received
 */
bool handle_ext_output_mode(char ch) {
  const int max_args = 11;
  static int i = 0;
  if (ch != ':' && i < ARGBUF_LEN) {
    argbuf[i++] = ch;
    return true;
  } else {
    int args[max_args];
    int nargs;
    argbuf[i] = '\0';
    nargs = read_args(argbuf, args, max_args);

    if (args[0] == -1) {
      interchar_delay = 0;
    } else {
      interchar_delay = args[0];
    }

    for (int j = 1; j < max_args; j++) {
      if (args[j] == -1) {
        imm_xoff[j - 1] = '\0';
      } else {
        imm_xoff[j - 1] = args[j];
      }
    }

    i = 0;
    return false;
  }
}

/* ESC . R

  Reset handshake parameters to default
*/
void reset_handshake(void) {
#ifdef DTR_PIN
  pinMode(DTR_PIN, OUTPUT);
  digitalWrite(DTR_PIN, DTR_TRUE);
#endif

  /* Reset configuration options to defaults */
  dtr_mode = false;
  echo_mode = ECHO_OFF;

  handshake_mode = 0;
  block_size = 80;
  enq_char = '\0';

  for (int i = 0; i < 11; i++) {
    ack_xon[i] = '\0';
  }

  output_trigger = '\0';
  echo_terminator = '\0';
  output_terminator[0] = '\r';
  output_terminator[1] = '\0';
  output_terminator[2] = '\0';
  output_initiator = '\0';

  for (int i = 0; i < 11; i++) {
    imm_xoff[i] = '\0';
  }
}

void restartGpibTask() {
  if (gpib_taskhandle != NULL) {
    vTaskDelete(gpib_taskhandle);
  }

  xTaskCreate(gpibTask, "gpib", 128, NULL, 1, &gpib_taskhandle);
}

#ifdef HANDSHAKE_DEBUG
void dump_config() {
  Serial.println("Handshake Configuration:");
  Serial.print("DTR mode: ");
  Serial.println(dtr_mode);
  Serial.print("Echo mode: ");
  Serial.println(echo_mode);
  Serial.print("Handshake mode: ");
  Serial.println(handshake_mode);
  Serial.print("Block size: ");
  Serial.println(block_size);
  Serial.print("ENQ character: ");
  Serial.println(enq_char, HEX);
  Serial.print("ACK/XON string: ");
  for (int i = 0; i < 11; i++) {
    Serial.print(ack_xon[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
  Serial.print("Turnaround delay: ");
  Serial.println(turnaround_delay);
  Serial.print("Output trigger: ");
  Serial.println(output_trigger, HEX);
  Serial.print("Echo terminator: ");
  Serial.println(echo_term, HEX);
  Serial.print("Output terminator: ");
  Serial.print(output_term[0], HEX);
  Serial.print(' ');
  Serial.println(output_term[1], HEX);
  Serial.print("Output initiator: ");
  Serial.println(output_initiator, HEX);
  Serial.print("Inter-character delay: ");
  Serial.println(interchar_delay);
  Serial.print("Immediate-response/XOFF string: ");
  for (int i = 0; i < 11; i++) {
    Serial.print(imm_xoff[i], HEX);
    Serial.print(' ');
  }
}
#endif

/* Intercept characters and handle escape sequences
 * Returns true if character was 'consumed' by an escape sequence, false if
 * character should be processed further */
bool handle_esc(char ch) {
  static enum esc_state_t state = ESC_IDLE;

  /* Function-pointer to command argument handler.
   * Must return true while consuming characters, false once argument list is
   * complete. If NULL, command has no arguments */
  static bool (*handler)(char) = NULL;

  if (ch == CH_ESC) {
    state = ESC_EXPECT_DOT;
    return true;
  }

  if (state == ESC_EXPECT_DOT && ch == '.') {
    state = ESC_EXPECT_CMD;
    return true;
  }

  if (state == ESC_EXPECT_CMD) {
    if (plotter_enable) {
      switch (toupper(ch)) {
        case '@':  // Plotter Configuration
          handler = handle_config;
          break;
        case 'B':  // Output available buffer space
          handler = NULL;
          if (output_initiator) {
            Serial.print(output_initiator);
          }
          Serial.print(xStreamBufferSpacesAvailable(txBuf));
          Serial.print(output_terminator);
          Serial.flush();
          break;
        case 'E':  // Output extended error state
          handler = NULL;
          if (output_initiator) {
            Serial.print(output_initiator);
          }
          Serial.print(0);
          Serial.print(output_terminator);
          Serial.flush();
          break;
        case 'H':  // Set handshake mode 1
          handler = handle_handshake_1;
          break;
        case 'I':  // Set handshake mode 2
          handler = handle_handshake_2;
          break;
        case 'J':  // Abort device control string
          handler = NULL;
          break;
        case 'K':  // Abort graphic
          handler = NULL;
          restartGpibTask();
          break;
        case 'L':  // Output buffer size
          handler = NULL;
          if (output_initiator) {
            Serial.print(output_initiator);
          }
          Serial.print(TXBUF_SZ);
          Serial.print(output_terminator);
          Serial.flush();
          break;
        case 'M':  // Set output mode
          handler = handle_output_mode;
          break;
        case 'N':  // Set extended output and handshake mode
          handler = handle_ext_output_mode;
          break;
        case 'O':  // Output extended status
          handler = NULL;
          if (output_initiator) {
            Serial.print(output_initiator);
          }
          Serial.print(0);
          Serial.print(output_terminator);
          Serial.flush();
          break;
#ifdef HANDSHAKE_DEBUG
        case 'P':
          handler = NULL;
          if (output_initiator) {
            Serial.print(output_initiator);
          }
          dump_config();
          Serial.print(output_term);
          break;
#endif
        case 'R':  // Reset handshake parameters
          reset_handshake();
          break;
        case '(':
        case 'Y':  // Plotter on
          handler = NULL;
          plotter_enable = true;
          break;
        case ')':
        case 'Z':          // Plotter off
          handler = NULL;  // Ignore
          plotter_enable = false;
          break;
        default:
          handler = NULL;
          break;
      }
    } else {
      handler = NULL;
      if (ch == '(' || ch == 'Y') {
        plotter_enable = true;
      }
    }

    if (handler == NULL) {
      state = ESC_IDLE;
      return true;
    } else {
      state = ESC_EXPECT_ARGS;
      return true;
    }
  }

  if (state == ESC_EXPECT_ARGS) {
    if (handler(ch)) {
      return true;
    } else {
      state = ESC_IDLE;
      return true;
    }
  }
  return false;
}

void serialTask(void *pvParams) {
  bool in_xoff = false;
  char ch = '\0';
  reset_handshake();
  while (1) {
    /* Space has freed up for another block */
    if (in_xoff && xStreamBufferSpacesAvailable(txBuf) > block_size) {
      if (in_xoff) {
#ifdef DTR_PIN
        if (dtr_mode) {
          digitalWrite(DTR_PIN, DTR_TRUE);
        }
#endif
        /* Send XON string if we're not in ENQ/ACK mode */
        if (!enq_char) {
          Serial.print(ack_xon);
          if (handshake_mode == 1) {
            Serial.print(output_terminator);
          }
          Serial.flush();
        }
        in_xoff = false;
      }
    }

    /* Read bytes from serial port and insert into GPIB TX buffer */
    while (Serial.available()) {
      ch = Serial.read();

      if (!handle_esc(ch)) {
        if (enq_char && ch == enq_char) {
          /* We're in ENQ/ACK mode and Incoming character is enquiry */

          /* Send immediate response */
          Serial.print(imm_xoff);

          /* In handshake mode 1, we send output terminators after
           * handshake strings */
          if (handshake_mode == 1) {
            Serial.print(output_terminator);
          }
          Serial.flush();

          /* Wait for buffer to have enough space for a block */
          while (xStreamBufferSpacesAvailable(txBuf) < block_size) {
            vTaskDelay(1);
          }

          /* Send acknowledgement */
          Serial.print(ack_xon);

          /* In handshake mode 1, we send output terminators after
           * handshake strings */
          if (handshake_mode == 1) {
            Serial.print(output_terminator);
          }
          Serial.flush();
        } else if (plotter_enable) {
          /* In immediate-echo mode, echo character back when we
           * receive it */
          if (echo_mode == ECHO_IMMEDIATE) {
            Serial.write(ch);
          }
          xStreamBufferSend(txBuf, &ch, 1, 0);
        }
      }

      if (xStreamBufferSpacesAvailable(txBuf) < block_size) {
        /* No room for another block */
        in_xoff = true;
#ifdef DTR_PIN
        if (dtr_mode) {
          digitalWrite(DTR_PIN, DTR_FALSE);
        }
#endif
        /* Send XOFF string if we're not in ENQ/ACK mode */
        if (!enq_char) {
          Serial.print(imm_xoff);
          if (handshake_mode == 1) {
            Serial.print(output_terminator);
          }
          Serial.flush();
        }
      }
    }
    vTaskDelay(1);
  }
}

void setup() {
  /* put your setup code here, to run once: */
  Serial.begin(9600);

  txBuf = xStreamBufferCreate(TXBUF_SZ, 1);

  xTaskCreate(gpibTask, "gpib", 128, NULL, 1, &gpib_taskhandle);
  xTaskCreate(serialTask, "serial", 128, NULL, 2, NULL);
}

void loop() { /* Nothing to do here, it all happens in tasks */
}
