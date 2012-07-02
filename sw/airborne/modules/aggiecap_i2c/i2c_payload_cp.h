#ifndef I2C_PAYLOAD_H_
#define I2C_PAYLOAD_H_

#include "std.h"
#ifndef SITL
#include "mcu_periph/i2c.h"
#endif

#include "led.h"


#define AGCP_LINK_DATA_SIZE 64
#define AGCP_LINK_PKT_SIZE (AGCP_LINK_DATA_SIZE + 5)

extern uint8_t payloadstatus;
extern uint16_t capturenum;
extern uint8_t errorcode;

bool_t gotpayloaddata;

extern void i2cpayloadinit_cp(void);
extern void checkI2CBuffer_cp(void);
extern void sendPayloadData_cp(void);

extern uint8_t i2c0_circ_read(uint8_t *circdata);
extern uint16_t i2c0_circ_buff_size(void);

#endif
