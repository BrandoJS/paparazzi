/*
 * Copyright (C) 2009  ENAC, Pascal Brisset, Michel Gorraz,Gautier Hattenberger
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

#ifndef PAYLOAD_I2C
#define PAYLOAD_I2C

#include "std.h"

#define PAYLOAD_I2C_SLAVE_ADDR (0x42 << 1)

#define payload_I2C_BUF_SIZE 256
extern uint8_t payload_i2c_rx_buf[payload_I2C_BUF_SIZE];
extern uint8_t payload_i2c_rx_insert_idx, payload_i2c_rx_extract_idx;
extern uint8_t payload_i2c_tx_buf[payload_I2C_BUF_SIZE];
extern uint8_t payload_i2c_tx_insert_idx, payload_i2c_tx_extract_idx;

extern bool_t payload_i2c_done, payload_i2c_data_ready_to_transmit;

void payload_i2c_init(void);
void payload_i2c_event(void);
void payload_i2c_periodic(void);

#define payload_i2cEvent() { if (payload_i2c_done) payload_i2c_event(); }
#define payload_i2cChAvailable() (payload_i2c_rx_insert_idx != payload_i2c_rx_extract_idx)
#define payload_i2cGetch() (payload_i2c_rx_buf[payload_i2c_rx_extract_idx++])
#define payload_i2cTransmit(_char) {             \
  if (! payload_i2c_data_ready_to_transmit)  /* Else transmitting, overrun*/     \
    payload_i2c_tx_buf[payload_i2c_tx_insert_idx++] = _char; \
}
#define payload_i2cSendMessage() {           \
  payload_i2c_data_ready_to_transmit = TRUE; \
  payload_i2c_tx_extract_idx = 0;            \
}
// #define payload_i2cTxRunning (payload_i2c_data_ready_to_transmit)
// #define payload_i2cInitParam(_baud, _uart_prm1, _uart_prm2) {}

#endif // payload_I2C
