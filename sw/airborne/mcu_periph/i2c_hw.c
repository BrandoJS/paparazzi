/*
 * $Id: i2c_hw.c 2985 2009-02-04 18:45:36Z mmm $
 *  
 * Copyright (C) 2008  Pascal Brisset, Antoine Drouin
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA. 
 *
 */

#include "i2c.h"

#include "led.h"

#include "std.h"

#include "interrupt_hw.h"
#include "i2c_payload.h"

/* default clock speed 37.5KHz with our 15MHz PCLK 
   I2C0_CLOCK = PCLK / (I2C0_SCLL + I2C0_SCLH)     */
#ifndef I2C0_SCLL
#define I2C0_SCLL 200
#endif

#ifndef I2C0_SCLH
#define I2C0_SCLH 200
#endif

/* default clock speed 37.5KHz with our 15MHz PCLK 
   I2C1_CLOCK = PCLK / (I2C1_SCLL + I2C1_SCLH)     */
#ifndef I2C1_SCLL
#define I2C1_SCLL 200
#endif

#ifndef I2C1_SCLH
#define I2C1_SCLH 200
#endif

/* adjust for other PCLKs */

#if (PCLK == 15000000)
#define I2C0_SCLL_D I2C0_SCLL
#define I2C0_SCLH_D I2C0_SCLH
#define I2C1_SCLL_D I2C1_SCLL
#define I2C1_SCLH_D I2C1_SCLH
#else

#if (PCLK == 30000000)
#define I2C0_SCLL_D (2*I2C0_SCLL)
#define I2C0_SCLH_D (2*I2C0_SCLH)
#define I2C1_SCLL_D (2*I2C1_SCLL)
#define I2C1_SCLH_D (2*I2C1_SCLH)
#else

#if (PCLK == 60000000)
#define I2C0_SCLL_D (4*I2C0_SCLL)
#define I2C0_SCLH_D (4*I2C0_SCLH)
#define I2C1_SCLL_D (4*I2C1_SCLL)
#define I2C1_SCLH_D (4*I2C1_SCLH)
#else

#error unknown PCLK frequency
#endif
#endif
#endif

#ifndef I2C0_VIC_SLOT
#define I2C0_VIC_SLOT 9
#endif

// test if slave mode -> slave address has to be defined
#if USE_I2C_SLAVE
#ifndef I2C_SLAVE_ADDR
#error I2C slave address (I2C_SLAVE_ADDR) not defined
#endif
#endif

void i2c0_ISR(void) __attribute__((naked));

void i2c0_slave_ISR(void) __attribute__ ((interrupt("IRQ"))); // determine that this function is an ISR
void i2c0_payload_ISR(void) __attribute__ ((interrupt("IRQ"))); // determine that this function is an ISR

/* SDA0 on P0.3 */
/* SCL0 on P0.2 */
void i2c0_hw_init ( void ) {

  /* set P0.2 and P0.3 to I2C0 */
  PINSEL0 |= 1 << 4 | 1 << 6;
  /* clear all flags */
  I2C0CONCLR = _BV(AAC) | _BV(SIC) | _BV(STAC) | _BV(I2ENC);
  /* enable I2C */
  I2C0CONSET = _BV(I2EN);
  /* set bitrate */
  I2C0SCLL = I2C0_SCLL_D;  
  I2C0SCLH = I2C0_SCLH_D;  
  
  // initialize the interrupt vector
  VICIntSelect &= ~VIC_BIT(VIC_I2C0);              // I2C0 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_I2C0);                // I2C0 interrupt enabled
  _VIC_CNTL(I2C0_VIC_SLOT) = VIC_ENABLE | VIC_I2C0;
  _VIC_ADDR(I2C0_VIC_SLOT) = (uint32_t)i2c0_ISR;    // address of the ISR
}

void i2c0_slave_init(void) {

	PINSEL0    |= 0x50;	// P0.3 = SDA, P0.2 = SCL
	I2C0ADR     = I2C_SLAVE_ADDR << 1; // set I2C slave address - this value must be shifted one bit left from the original address because of bit 0 determining whether to react on General Calls
	I2C0CONCLR = _BV(AAC) | _BV(SIC) | _BV(STAC) | _BV(I2ENC);	// clear all flags
	I2C0CONSET = _BV(I2EN) | _BV(AA);	// enable I2C, set AA flag
	// initialize the interrupt vector
	VICIntSelect &= ~VIC_BIT(VIC_I2C0);              // I2C0 selected as IRQ
	VICIntEnable = VIC_BIT(VIC_I2C0);                // I2C0 interrupt enabled
	_VIC_CNTL(I2C0_VIC_SLOT) = VIC_ENABLE | VIC_I2C0;
	_VIC_ADDR(I2C0_VIC_SLOT) = (uint32_t)i2c0_slave_ISR;    // address of the ISR

}

void i2c0_payload_init(void) {

	PINSEL0    |= 0x50;	// P0.3 = SDA, P0.2 = SCL
	I2C0ADR     = I2C_SLAVE_ADDR << 1; // set I2C slave address - this value must be shifted one bit left from the original address because of bit 0 determining whether to react on General Calls
	I2C0CONCLR = _BV(AAC) | _BV(SIC) | _BV(STAC) | _BV(I2ENC);	// clear all flags
	I2C0CONSET = _BV(I2EN) | _BV(AA);	// enable I2C, set AA flag
	// initialize the interrupt vector
	VICIntSelect &= ~VIC_BIT(VIC_I2C0);              // I2C0 selected as IRQ
	VICIntEnable = VIC_BIT(VIC_I2C0);                // I2C0 interrupt enabled
	_VIC_CNTL(I2C0_VIC_SLOT) = VIC_ENABLE | VIC_I2C0;
	_VIC_ADDR(I2C0_VIC_SLOT) = (uint32_t)i2c0_payload_ISR;    // address of the ISR

}

#define I2C_DATA_REG I2C0DAT
#define I2C_STATUS_REG I2C0STAT

void i2c0_ISR(void)
{
  ISR_ENTRY();

  uint32_t state = I2C_STATUS_REG;
  I2cAutomaton(state);
  I2cClearIT();
  
  VICVectAddr = 0x00000000;             // clear this interrupt from the VIC
  ISR_EXIT();                           // recover registers and return
}

void i2c0_slave_ISR(void)
{

	uint32_t state = I2C_STATUS_REG;	// read current status
	I2cClearStart();					// clear start bit
	I2cClearAA();						// clear AA bit
	switch(state)
	{
		case I2C_SR_ARBIT_LOST:			// Arbitration was lost
			I2cSendStart();		// set Start, proceed as if slave address would have been received normally
		case I2C_SR_SLA_ACK:			// own SLA+W received, ACK returned
			I2cSendAck();		// send ACK on first byte
			i2cReceiveBufferIndex = 0;
			break;
		case I2C_SR_DATA_ACK:			// Data received, ACK returned
			// first data byte determines payload length
			if (i2cReceiveBufferIndex == 0){
				i2c_receive_payload_length = I2C_DATA_REG;
				i2cReceiveBufferIndex++;
			} else {
				if (i2cReceiveBufferIndex < I2C_BUF_LEN) {	// if buffer is not full, continue
					i2cReceiveBuffer[i2cReceiveBufferIndex-1] = I2C_DATA_REG;	// read and store data
					i2cReceiveBufferIndex++;
				} else {
					// handle error somehow
				}
				// if last byte, set validity on "true" so that data can be read by application
				if (i2cReceiveBufferIndex >= i2c_receive_payload_length + 1){			
					i2c_slave_data_valid = TRUE;
				}
			}
			I2cSendAck();		// send ACK to receive more data
			break;
		case I2C_SR_DATA_NACK:			// Data received, NACK returned
		case I2C_SR_RESTART:			// STOP or REP.START received while addressed as slave -> this was the last byte to read
			I2cSendAck();				// send ACK, switch to not addressed slave mode
			break;

		case I2C_ST_ARBIT_LOST:			// Arbitration was lost
			I2cSendStart();		// set Start, proceed as if slave address would have been received normally
		case I2C_ST_SLA_ACK:			// own SLA+R received, ACK returned -> first byte to send
			// first byte to read, so reset index
			i2cSendBufferIndex = 0;
			// transmit number of bytes to transmit first so that Master knows how many bytes to request
			// payload length was already computed before
			I2C_DATA_REG = i2c_send_payload_length;
			i2cSendBufferIndex++;
			I2cSendAck();				// send ACK to commit data
			break;
		case I2C_ST_DATA_ACK:			// Data transmitted, ACK received -> all following bytes to send
				if (i2cSendBufferIndex < i2c_send_payload_length+1) {	// if not all bytes sent yet, continue
					I2C_DATA_REG = i2cSendBuffer[i2cSendBufferIndex-1];	// store data
					i2cSendBufferIndex++;
				} else {
					// more bytes requested than actually present, handle error somehow
				}
		
			I2cSendAck();				// send ACK to commit data
			break;
		case I2C_ST_DATA_NACK:			// Data transmitted, NACK received
		case I2C_ST_LAST_DATA_ACK:		// last Data transmitted, ACK received
				// last byte, clear buffer and set validity
				i2c_slave_data_valid = TRUE;
				i2cSendBufferIndex = 0;
			I2cSendAck();				// send ACK, switch to not addressed slave mode
			break;
		default:
			break;
	}
	
	VICVectAddr = 0;		// reset VIC
	I2cClearIT();	// clear interrupt flag

}

uint16_t temp = 0;
void i2c0_payload_ISR(void)
{
	uint32_t state = I2C_STATUS_REG;	// read current status
	I2cClearStart();					// clear start bit
	I2cClearAA();						// clear AA bit
	switch(state)
	{
		case I2C_SR_ARBIT_LOST:			// Arbitration was lost
			I2cSendStart();		// set Start, proceed as if slave address would have been received normally
		case I2C_SR_SLA_ACK:			// own SLA+W received, ACK returned
			I2cSendAck();		// send ACK on first byte
			i2cReceiveBufferIndex = 0;
			temp = 0;
			break;
		case I2C_SR_DATA_ACK:			// Data received, ACK returned
			// first data byte determines type of data
			//payloadstatus = I2C_DATA_REG;

			if (i2cReceiveBufferIndex == 0)
			{
				i2c_code = I2C_DATA_REG;
				i2cReceiveBufferIndex++;
				temp = 0;
			} 
                        else 
			{
				if(i2c_code == 1)	//Warm up and Cool down Byte
					payloadstatus = I2C_DATA_REG;
				else if(i2c_code == 2)	//Capture Number
				{
					if(i2cReceiveBufferIndex <= 1)
					{
						temp = I2C_DATA_REG;
						i2cReceiveBufferIndex++;
					}
					else
					{

						temp |= (((uint16_t)I2C_DATA_REG) << 8);
						capturenum = temp;
					}
				}
				else if(i2c_code == 3)	//Error Code
				{
					errorcode = I2C_DATA_REG;
					gotpayloaddata = TRUE;
				}
				else
					payloadstatus = 10;
			}
			I2cSendAck();		// send ACK to receive more data
			break;
		case I2C_SR_DATA_NACK:			// Data received, NACK returned
		case I2C_SR_RESTART:			// STOP or REP.START received while addressed as slave -> this was the last byte to read
			I2cSendAck();				// send ACK, switch to not addressed slave mode
			break;

		case I2C_ST_ARBIT_LOST:			// Arbitration was lost
			I2cSendStart();		// set Start, proceed as if slave address would have been received normally
		case I2C_ST_SLA_ACK:			// own SLA+R received, ACK returned -> first byte to send
			// first byte to read, so reset index
			i2cSendBufferIndex = 0;
			// transmit number of bytes to transmit first so that Master knows how many bytes to request
			// payload length was already computed before
			I2C_DATA_REG = i2c_send_payload_length;
			i2cSendBufferIndex++;
			I2cSendAck();				// send ACK to commit data
			break;
		case I2C_ST_DATA_ACK:			// Data transmitted, ACK received -> all following bytes to send
				if (i2cSendBufferIndex < i2c_send_payload_length+1) {	// if not all bytes sent yet, continue
					I2C_DATA_REG = i2cSendBuffer[i2cSendBufferIndex-1];	// store data
					i2cSendBufferIndex++;
				} else {
					// more bytes requested than actually present, handle error somehow
				}
		
			I2cSendAck();				// send ACK to commit data
			break;
		case I2C_ST_DATA_NACK:			// Data transmitted, NACK received
		case I2C_ST_LAST_DATA_ACK:		// last Data transmitted, ACK received
				// last byte, clear buffer and set validity
				i2c_slave_data_valid = TRUE;
				i2cSendBufferIndex = 0;
			I2cSendAck();				// send ACK, switch to not addressed slave mode
			break;
		default:
			break;
	}
	
	VICVectAddr = 0;		// reset VIC
	I2cClearIT();	// clear interrupt flag

}

#ifdef USE_I2C1

#define I2C1_DATA_REG   I2C1DAT
#define I2C1_STATUS_REG I2C1STAT

void i2c1_ISR(void) __attribute__((naked));

/* SDA1 on P0.14 */
/* SCL1 on P0.11 */
void i2c1_hw_init ( void ) {

  /* set P0.11 and P0.14 to I2C1 */
  PINSEL0 |= 3 << 22 | 3 << 28;
  /* clear all flags */
  I2C1CONCLR = _BV(AAC) | _BV(SIC) | _BV(STAC) | _BV(I2ENC);
  /* enable I2C */
  I2C1CONSET = _BV(I2EN);
  /* set bitrate */
  I2C1SCLL = I2C1_SCLL_D;  
  I2C1SCLH = I2C1_SCLH_D;  
  
  // initialize the interrupt vector
  VICIntSelect &= ~VIC_BIT(VIC_I2C1);              // I2C0 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_I2C1);                // I2C0 interrupt enabled
  _VIC_CNTL(I2C1_VIC_SLOT) = VIC_ENABLE | VIC_I2C1;
  _VIC_ADDR(I2C1_VIC_SLOT) = (uint32_t)i2c1_ISR;    // address of the ISR
}

void i2c1_ISR(void)
{
  ISR_ENTRY();

  uint32_t state = I2C1_STATUS_REG;
  I2c1Automaton(state);
  I2c1ClearIT();
  
  VICVectAddr = 0x00000000;             // clear this interrupt from the VIC
  ISR_EXIT();                           // recover registers and return
}


#endif /* USE_I2C1 */

