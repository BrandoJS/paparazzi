#ifndef I2C_PAYLOAD_H_
#define I2C_PAYLOAD_H_

#include "std.h"
#include "i2c.h"
#include "led.h"

extern uint8_t payloadstatus;
extern uint16_t capturenum;
extern uint8_t errorcode;

bool_t gotpayloaddata;

void i2cpayloadinit();
extern bool_t GotPayloadData();

#endif
