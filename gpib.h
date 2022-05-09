#pragma once

#include <Arduino.h>
#include <stdint.h>

/**** GPIB Commands ****/
/* Addressing */
#define G_TAD 0x40 // Talk address 
#define G_LAD 0x20 // Listen address 
/* Universal Commands */
#define G_UNL 0x3f // Unlisten 
#define G_UNT 0x5f // Untalk 
#define G_LLO 0x11 // Local Lockout 
#define G_DCL 0x14 // Device Clear 
#define G_PPU 0x15 // Parallel Poll Unconfigure 
#define G_SPE 0x18 // Serial Poll Enable 
#define G_SPD 0x19 // Serial Poll Disable 
/* Addressed Commands */
#define G_GTL 0x01 // Go To Local 
#define G_SDC 0x04 // Selected Device Clear 
#define G_PPC 0x05 // Parallel Poll Configure 
#define G_GET 0x08 // Group Execute Trigger 
#define G_TCT 0x09 // Take Control

/* Utility macros for GPIB pin control
 *
 * GPIB is an open-collector bus - a line is asserted by pulling it low, and
 * de-asserted by leaving it floating
 */
#define SET(pin)        \
  pinMode(pin, OUTPUT); \
  digitalWrite(pin, LOW);
#define CLEAR(pin) pinMode(pin, INPUT_PULLUP);
#define OUT(pin, level) \
  if (level) {          \
    SET(pin);           \
  } else {              \
    CLEAR(pin);         \
  }
#define IS_SET(pin) (digitalRead(pin) == LOW)
#define IS_CLEAR(pin) (digitalRead(pin) == HIGH)

/* Wait for a pin to be asserted, yielding task for 1 timer tick between checks
 *
 * pin: Arduino pin number
 *
 * timeout: timeout value in ms, -1 to block indefinitely
 *
 * returns: 0 if pin has been asserted, -1 if timeout has been reached
 */
int wait_set(int pin, int timeout);

/* Wait for a pin to be cleared, yielding task for 1 timer tick between checks
 *
 * pin: Arduino pin number
 *
 * timeout: timeout value in ms, -1 to block indefinitely
 *
 * returns: 0 if pin has been cleared, -1 if timeout has been reached
 */
int wait_clear(int pin, int timeout);

/* Initialize GPIB bus
 *
 * Clear all pins, and assert IFC for 100ms to reset the bus
 */
void gpib_init();

/* Output byte b on pins DIO1..DIO8 */
void data_write(uint8_t b);

/* Read a byte from pins DOI1..DIO8 */
uint8_t data_read();

/* Write a byte on the GPIB bus
 *
 * b: byte to send
 *
 * command: if true, send as command (with ATN asserted)
 *
 * eoi: if true, send with EOI asserted
 *
 * timeout: timeout for GPIB handshake, -1 to block indefinitely
 *
 * returns: 0 if byte transmitted and accepted, -1 if timeout reached
 */
int gpib_write(uint8_t b, bool command, bool eoi, int timeout);

/* Read a byte on the GPIB bus
 *
 * *b (out-parameter): pointer to location to output byte to
 *
 * timeout: timeout for GPIB handshake, -1 to block indefinitely
 *
 * returns: 0 if byte received successfully, 1 if byte received with EOI, -1 if
 * timeout reached
 */
int gpib_read(uint8_t* b, int timeout);

/* Write a command byte on the GPIB bus
 *
 * Shorthand for gpib_write(b, true, false, timeout)
 */
int gpib_cmd(uint8_t cmd, int timeout);

/* Start a GPIB TALK/LISTEN transaction
 *
 * talk_addr: address of talking device
 *
 * listen_addr: address of listening device
 *
 * timeout: timeout for GPIB handshake, -1 to block indefinitely
 *
 * returns: 0 if transaction initiated successfully, -1 if timeout reached
 */
int gpib_start(uint8_t talk_addr, uint8_t listen_addr, int timeout);

/* End a GPIB TALK/LISTEN transaction (i.e. send UNTALK, UNLISTEN)
 *
 * timeout: timeout for GPIB handshake, -1 to block indefinitely
 *
 * returns: 0 if transaction ended successfully, -1 if timeout reached
 */
int gpib_stop(int timeout);
