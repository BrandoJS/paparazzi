#ifndef I2C_PAYLOAD_H_
#define I2C_PAYLOAD_H_

#include "std.h"
#include "mcu_periph/i2c.h"
#include "led.h"

extern uint8_t payloadstatus;
extern uint16_t capturenum;
extern uint8_t errorcode;

bool_t gotpayloaddata;

extern void i2cpayloadinit(void);
extern void checkI2CBuffer(void);
extern void sendPayloadData(void);

#endif
