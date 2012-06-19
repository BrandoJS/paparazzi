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

#include "payload_i2c.h"
#include "mcu_periph/i2c.h"
#include "subsystems/payload.h"

struct i2c_transaction i2c_payload_trans;


uint8_t payload_i2c_rx_buf[PAYLOAD_I2C_BUF_SIZE];
uint8_t payload_i2c_rx_insert_idx, payload_i2c_rx_extract_idx;
uint8_t payload_i2c_tx_buf[PAYLOAD_I2C_BUF_SIZE];
uint8_t payload_i2c_tx_insert_idx, payload_i2c_tx_extract_idx;
bool_t payload_i2c_done, payload_i2c_data_ready_to_transmit;

/* u-blox5 protocole, page 4 */
#define PAYLOAD_I2C_ADDR_NB_AVAIL_BYTES 0xFD
#define PAYLOAD_I2C_ADDR_DATA 0xFF

#define PAYLOAD_I2C_STATUS_IDLE                   0
#define PAYLOAD_I2C_STATUS_ASKING_DATA            1
#define PAYLOAD_I2C_STATUS_ASKING_NB_AVAIL_BYTES  2
#define PAYLOAD_I2C_STATUS_READING_NB_AVAIL_BYTES 3
#define PAYLOAD_I2C_STATUS_READING_DATA           4

#define payload_i2c_AddCharToRxBuf(_x) { \
  payload_i2c_rx_buf[payload_i2c_rx_insert_idx] = _x; \
  payload_i2c_rx_insert_idx++; /* size=256, No check for buf overflow */ \
}

static uint8_t payload_i2c_status;
//static uint16_t payload_i2c_nb_avail_bytes; /* size buffer =~ 12k */
//static uint8_t data_buf_len;

void payload_i2c_init(void) {
  payload_i2c_status = PAYLOAD_I2C_STATUS_IDLE;
  payload_i2c_done = TRUE;
  payload_i2c_data_ready_to_transmit = FALSE;
  payload_i2c_rx_insert_idx = 0;
  payload_i2c_rx_extract_idx = 0;
  payload_i2c_tx_insert_idx = 0;
#ifdef PAYLOAD_CONFIGURE
  /* The call in main_ap.c is made before the modules init (too early) */
  payload_configure_uart();
#endif
}

void payload_i2c_periodic(void) {
/*
  if (payload_i2c_done && payload_i2c_status == PAYLOAD_I2C_STATUS_IDLE) {
    i2c0_buf[0] = PAYLOAD_I2C_ADDR_NB_AVAIL_BYTES;
    i2c0_transmit_no_stop(PAYLOAD_I2C_SLAVE_ADDR, 1, &payload_i2c_done);
    payload_i2c_done = FALSE;
    payload_i2c_status = PAYLOAD_I2C_STATUS_ASKING_NB_AVAIL_BYTES;
  }
*/

}

void payload_i2c_event(void) {
/*
 *  switch (payload_i2c_status) {
  case PAYLOAD_I2C_STATUS_IDLE:
    if (payload_i2c_data_ready_to_transmit) {
      // Copy data from our buffer to the i2c buffer
      uint8_t data_size = Min(payload_i2c_tx_insert_idx-payload_i2c_tx_extract_idx, I2C0_BUF_LEN);
      uint8_t i;
      for(i = 0; i < data_size; i++, payload_i2c_tx_extract_idx++)
        i2c0_buf[i] = payload_i2c_tx_buf[payload_i2c_tx_extract_idx];

      // Start i2c transmit
      i2c0_transmit(PAYLOAD_I2C_SLAVE_ADDR, data_size, &payload_i2c_done);
      payload_i2c_done = FALSE;

      // Reset flag if finished
      if (payload_i2c_tx_extract_idx >= payload_i2c_tx_insert_idx) {
        payload_i2c_data_ready_to_transmit = FALSE;
        payload_i2c_tx_insert_idx = 0;
      }
    }
    break;

  case PAYLOAD_I2C_STATUS_ASKING_NB_AVAIL_BYTES:
    i2c0_receive(PAYLOAD_I2C_SLAVE_ADDR, 2, &payload_i2c_done);
    payload_i2c_done = FALSE;
    payload_i2c_status = PAYLOAD_I2C_STATUS_READING_NB_AVAIL_BYTES;
    break;

  case PAYLOAD_I2C_STATUS_READING_NB_AVAIL_BYTES:
    payload_i2c_nb_avail_bytes = (i2c0_buf[0]<<8) | i2c0_buf[1];

    if (payload_i2c_nb_avail_bytes)
      goto continue_reading;
    else
      payload_i2c_status = PAYLOAD_I2C_STATUS_IDLE;
    break;

  continue_reading:

  case PAYLOAD_I2C_STATUS_ASKING_DATA:
    data_buf_len = Min(payload_i2c_nb_avail_bytes, I2C0_BUF_LEN);
    payload_i2c_nb_avail_bytes -= data_buf_len;

    i2c0_receive(PAYLOAD_I2C_SLAVE_ADDR, data_buf_len, &payload_i2c_done);
    payload_i2c_done = FALSE;
    payload_i2c_status = PAYLOAD_I2C_STATUS_READING_DATA;
    break;

  case PAYLOAD_I2C_STATUS_READING_DATA: {
    uint8_t i;
    for(i = 0; i < data_buf_len; i++) {
      payload_i2c_AddCharToRxBuf(i2c0_buf[i]);
    }

    if (payload_i2c_nb_avail_bytes)
     goto continue_reading;
    else
      payload_i2c_status = PAYLOAD_I2C_STATUS_IDLE;
    break;
  }

  default:
    return;
  }
*/

}


