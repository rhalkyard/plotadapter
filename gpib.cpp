#include <Arduino.h>

/*** FreeRTOS headers - requires FreeRTOS library from Library Manager ***/
#include <Arduino_FreeRTOS.h>

#include "config.h"
#include "gpib.h"

/* Wait for pin to be set (pulled low) */
int wait_set(int pin, int timeout) {
  unsigned long startTime = millis();
  CLEAR(pin);
  while (IS_CLEAR(pin)) {
    if (timeout >= 0 && millis() > startTime + timeout) {
      return -1;
    }
    vTaskDelay(0);
  }
  return 0;
}

/* Wait for pin to be cleared (floating high) */
int wait_clear(int pin, int timeout) {
  unsigned long startTime = millis();
  CLEAR(pin);
  while (IS_SET(pin)) {
    if (timeout >= 0 && millis() > startTime + timeout) {
      return -1;
    }
    vTaskDelay(0);
  }
  return 0;
}

/* Clear data pins */
void data_clear() {
  CLEAR(DIO1);
  CLEAR(DIO2);
  CLEAR(DIO3);
  CLEAR(DIO4);
  CLEAR(DIO5);
  CLEAR(DIO6);
  CLEAR(DIO7);
  CLEAR(DIO8);
}

/* Write byte to DIO1..8 */
void data_write(uint8_t b) {
  OUT(DIO1, bitRead(b, 0));
  OUT(DIO2, bitRead(b, 1));
  OUT(DIO3, bitRead(b, 2));
  OUT(DIO4, bitRead(b, 3));
  OUT(DIO5, bitRead(b, 4));
  OUT(DIO6, bitRead(b, 5));
  OUT(DIO7, bitRead(b, 6));
  OUT(DIO8, bitRead(b, 7));
}

/* Read byte form DIO1..8 */
uint8_t data_read() {
  uint8_t b = 0;
  data_clear();
  bitWrite(b, 0, IS_SET(DIO1));
  bitWrite(b, 1, IS_SET(DIO2));
  bitWrite(b, 2, IS_SET(DIO3));
  bitWrite(b, 3, IS_SET(DIO4));
  bitWrite(b, 4, IS_SET(DIO5));
  bitWrite(b, 5, IS_SET(DIO6));
  bitWrite(b, 6, IS_SET(DIO7));
  bitWrite(b, 7, IS_SET(DIO8));
  return b;
}

/* Initialize bus */
void gpib_init() {
  /* Clear all data and signal lines */
  data_clear();
  CLEAR(EOI);
  CLEAR(DAV);
  CLEAR(IFC);
  CLEAR(ATN);
  CLEAR(NDAC);
  CLEAR(NRFD);
#ifdef SRQ
  CLEAR(SRQ);
#endif
#ifdef REN
  CLEAR(REN);
#endif

  /* Assert IFC to reset the bus */
  SET(IFC);
  delay(100);
  CLEAR(IFC);
}

/* Write a byte */
int gpib_write(uint8_t b, bool command, bool eoi, int timeout) {
  int ret = 0;
  if (command) {
    SET(ATN);
  } else {
    CLEAR(ATN);
  }

  CLEAR(EOI);
  CLEAR(DAV);
  CLEAR(NRFD);
  CLEAR(NDAC);
  digitalWrite(LED_BUILTIN, HIGH);

  /* Wait for listeners to be ready */
  if (wait_set(NDAC, timeout)) {
    ret = -1;
    goto cleanup;
  }
  if (wait_clear(NRFD, timeout)) {
    ret = -1;
    goto cleanup;
  }

  data_write(b);  // Put data byte on the bus
  if (!command && eoi) {
    SET(EOI);
  }
  SET(DAV);  // Signal data available
  /* Wait for listeners to accept data */
  if (wait_clear(NDAC, timeout)) {
    ret = -1;
    goto cleanup;
  }

cleanup:
  digitalWrite(LED_BUILTIN, LOW);
  CLEAR(DAV);  // Clear data available, reset lines
  SET(NDAC);
  SET(NRFD);
  data_clear();
  if (command) {
    CLEAR(ATN);
  } else if (eoi) {
    CLEAR(EOI);
  }

  return ret;
}

/* Read a byte from GPIB */
int gpib_read(uint8_t* b, int timeout) {
  int ret = 0;
  bool eoi;

  SET(NDAC);
  CLEAR(NRFD);  // Signal ready for data
  digitalWrite(LED_BUILTIN, HIGH);
  /* Wait for data available */
  if (wait_set(DAV, timeout)) {
    ret = -1;
    goto cleanup;
  }
  SET(NRFD);
  *b = data_read();
  eoi = !digitalRead(EOI);
  if (eoi) {
    ret = 1;
  };

  CLEAR(NDAC);  // Accept data
  /* Wait for talker to finish */
  if (wait_clear(DAV, -1)) {
    ret = -1;
    goto cleanup;
  }
cleanup:
  digitalWrite(LED_BUILTIN, LOW);
  SET(NDAC);

  return ret;
}

/* Send a GPIB command byte */
int gpib_cmd(uint8_t cmd, int timeout) {
  return gpib_write(cmd, true, false, timeout);
}

/* Start a TALK/LISTEN transaction */
int gpib_start(uint8_t talk_addr, uint8_t listen_addr, int timeout) {
  if (gpib_cmd(G_TAD | talk_addr, timeout) == -1) {
    return -1;
  };
  if (gpib_cmd(G_LAD | listen_addr, timeout) == -1) {
    return -1;
  }
  return 0;
}

/* End a transaction */
int gpib_stop(int timeout) {
  if (gpib_cmd(G_UNT, timeout) == -1) {
    return -1;
  }
  if (gpib_cmd(G_UNL, timeout) == -1) {
    return -1;
  }
  return 0;
}
