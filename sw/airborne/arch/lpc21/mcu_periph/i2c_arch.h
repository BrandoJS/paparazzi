#ifndef I2C_HW_H
#define I2C_HW_H


#include "LPC21xx.h"


#define I2C_START        0x08
#define I2C_RESTART      0x10
#define I2C_MT_SLA_ACK   0x18
#define I2C_MT_SLA_NACK  0x20
#define I2C_MT_DATA_ACK  0x28
#define I2C_MR_SLA_ACK   0x40
#define I2C_MR_SLA_NACK  0x48
#define I2C_MR_DATA_ACK  0x50
#define I2C_MR_DATA_NACK 0x58

#define I2C_SR_SLA_ACK        0x60
#define I2C_SR_ARBIT_LOST     0x68
#define I2C_SR_DATA_ACK       0x80
#define I2C_SR_DATA_NACK      0x88
#define I2C_SR_RESTART        0xA0
#define I2C_ST_SLA_ACK        0xA8
#define I2C_ST_ARBIT_LOST     0xB0
#define I2C_ST_DATA_ACK       0xB8
#define I2C_ST_DATA_NACK      0xC0
#define I2C_ST_LAST_DATA_ACK  0xC8


#ifdef USE_I2C0

extern void i2c0_hw_init(void);

#endif /* USE_I2C0 */

#ifdef USE_I2C0_SLAVE

#define I2cSendAck_slv()   { I2C0CONSET = _BV(AA); }
#define I2cSendStop_slv()  {						\
    I2C0CONSET = _BV(STO);						\
    if (i2c_finished) *i2c_finished = TRUE;				\
    i2c_status = I2C_IDLE;						\
    I2cStopHandler();							\
  }
#define I2cSendStart_slv() { I2C0CONSET = _BV(STA); }
#define I2cSendByte_slv(b) { I2C_DATA_REG = b; }

#define I2cReceive_slv(_ack) {	    \
    if (_ack) I2C0CONSET = _BV(AA); \
    else I2C0CONCLR = _BV(AAC);	    \
  }

#define I2cClearStart_slv() { I2C0CONCLR = _BV(STAC); }
#define I2cClearIT_slv() { I2C0CONCLR = _BV(SIC); }
#define I2cClearAA_slv() { I2C0CONCLR = _BV(AAC); }

#define I2C_DATA_REG I2C0DAT
#define I2C_STATUS_REG I2C0STAT

extern volatile uint8_t i2cSendBuffer[I2C_BUF_LEN];
extern volatile uint8_t i2cReceiveBuffer[I2C_BUF_LEN];
extern volatile uint8_t i2cSendBufferIndex;
extern volatile uint8_t i2cReceiveBufferIndex;
extern volatile uint8_t i2c_send_payload_length;
extern volatile uint8_t i2c_receive_payload_length;
extern volatile bool_t i2c_slave_data_valid;
extern void i2c0_hw_init_slave(void);

extern uint8_t i2c0_circ_read(uint8_t *circdata);
extern uint16_t i2c0_circ_buff_size(void);

#endif

#ifdef USE_I2C1

extern void i2c1_hw_init(void);

#endif /* USE_I2C1 */


#endif /* I2C_HW_H */
