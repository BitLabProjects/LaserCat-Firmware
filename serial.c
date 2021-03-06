/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl v0.9

  Copyright (c) 2012-2014 Sungeun K. Jeon

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/
/* 
  This file is based on work from Grbl v0.8, distributed under the 
  terms of the MIT-license. See COPYING for more details.  
    Copyright (c) 2009-2011 Simen Svale Skogsrud
    Copyright (c) 2011-2012 Sungeun K. Jeon
*/ 

//#include <avr/interrupt.h>
#include "system.h"
#include "serial.h"
//#include "motion_control.h"
#include "protocol.h"


uint8_t serial_rx_buffer[RX_BUFFER_SIZE];
uint8_t serial_rx_buffer_head;
volatile uint8_t serial_rx_buffer_tail;

uint8_t serial_tx_buffer[TX_BUFFER_SIZE];
uint8_t serial_tx_buffer_head;
volatile uint8_t serial_tx_buffer_tail;

// Returns the number of bytes used in the RX serial buffer.
uint8_t serial_get_rx_buffer_count()
{
  uint8_t rtail;
  rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_rx_buffer_head >= rtail) { return(serial_rx_buffer_head-rtail); }
  return (RX_BUFFER_SIZE - (rtail-serial_rx_buffer_head));
}

uint8_t serial_has_bytes() {
  //!= 0 if there are bytes to read
  return serial_rx_buffer_tail - serial_rx_buffer_head;
}


// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t serial_get_tx_buffer_count()
{
  uint8_t ttail;
  ttail = serial_tx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_tx_buffer_head >= ttail) { return(serial_tx_buffer_head-ttail); }
  return (TX_BUFFER_SIZE - (ttail-serial_tx_buffer_head));
}

#define USE_SERIAL_INTERRUPTS

void serial_init()
{
//TODO
/*
  // Set baud rate
  #if BAUD_RATE < 57600
    uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1)/2 ;
    UCSR0A &= ~(1 << U2X0); // baud doubler off  - Only needed on Uno XXX
  #else
    uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
    UCSR0A |= (1 << U2X0);  // baud doubler on for high baud rates, i.e. 115200
  #endif
  UBRR0H = UBRR0_value >> 8;
  UBRR0L = UBRR0_value;
            
  // enable rx and tx
  UCSR0B |= 1<<RXEN0;
  UCSR0B |= 1<<TXEN0;
        
  // enable interrupt on complete reception of a byte
  UCSR0B |= 1<<RXCIE0;
          
  // defaults to 8-bit, no parity, 1 stop bit
*/

  BRGH_bit = 1; //BRG ad alta velocit�
  BRG16_bit = 1;  //16bit per il SPBRG
  //SPBRG = 103; // 1200 baud/sec a 4MHz
  
  SPBRG = 103; // 115000 baud/sec a 48MHz
  
  // 28800 baud/sec a 48MHz
  //SPBRGH = 1;
  //SPBRG = 156; 

  
  //SPBRG = 13; // 115000 baud/sec a 8MHz

  SYNC_bit = 0;
  TXEN_bit = 1;

  SPEN_bit = 1;
  CREN_bit = 1;

#ifdef USE_SERIAL_INTERRUPTS
  GIE_bit = 1;
  PEIE_bit = 1;
  RCIF_bit = 0;
  RCIE_bit = 1;
#endif

  serial_rx_buffer_head = 0;
  serial_rx_buffer_tail = 0;
  serial_tx_buffer_head = 0;
  serial_tx_buffer_tail = 0;
}

// Writes one byte to the TX serial buffer. Called by main program.
// TODO: Check if we can speed this up for writing strings, rather than single bytes.
void serial_write(uint8_t value) {
#ifndef USE_SERIAL_INTERRUPTS
  while(!TXIF_bit)
    continue;

  TXREG = value;
#else
  // Calculate next head
  uint8_t next_head;
  next_head = serial_tx_buffer_head + 1;
  if (next_head == TX_BUFFER_SIZE) { next_head = 0; }

  // Wait until there is space in the buffer
  while (next_head == serial_tx_buffer_tail) { 
    // TODO: Restructure st_prep_buffer() calls to be executed here during a long print.    
    if (sys.execute & EXEC_RESET) { return; } // Only check for abort to avoid an endless loop.
  }

  // Store data and advance head
  serial_tx_buffer[serial_tx_buffer_head] = value;
  serial_tx_buffer_head = next_head;
  
  // Enable Data Register Empty Interrupt to make sure tx-streaming is running
  TXIE_bit = 1;
#endif
}

// Data Register Empty Interrupt handler
void serial_tx_interrupt()
{
  uint8_t tail;
  tail = serial_tx_buffer_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile)
  
  // Send a byte from the buffer        
  TXREG = serial_tx_buffer[tail];

  // Update tail position
  tail++;
  if (tail == TX_BUFFER_SIZE) { tail = 0; }

  serial_tx_buffer_tail = tail;
  
  // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
  if (tail == serial_tx_buffer_head) { TXIE_bit = 0; }
}

// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial_read()
{
#ifndef USE_SERIAL_INTERRUPTS
  while(!RCIF_bit)
    continue;
  RCIF_bit=0;
  return RCREG;
#else
  uint8_t tail;
  uint8_t value;
  
  tail = serial_rx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
  if (serial_rx_buffer_head == tail) {
    return SERIAL_NO_DATA;
  } else {
    value = serial_rx_buffer[tail];
    
    tail++;
    if (tail == RX_BUFFER_SIZE) { tail = 0; }
    serial_rx_buffer_tail = tail;
    
    return value;
  }
#endif
}

void serial_rx_interrupt()
{
  uint8_t value;
  uint8_t next_head;
  value = RCREG;
  
  next_head = serial_rx_buffer_head + 1;
  if (next_head == RX_BUFFER_SIZE) { next_head = 0; }

  // Write data to buffer unless it is full.
  if (next_head != serial_rx_buffer_tail) {
    serial_rx_buffer[serial_rx_buffer_head] = value;
    serial_rx_buffer_head = next_head;        
  } else {
    //Huston we have a problem!
    //TODO: else alarm on overflow?
    value = value + 1;
    return;
  }
}

void serial_reset_read_buffer() 
{
  serial_rx_buffer_tail = serial_rx_buffer_head;
}