/*
 * Copyright (C) 2010 The Paparazzi Team
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

#include "modules/sensors/airspeed_adc.h"
#include "mcu_periph/adc.h"
#include BOARD_CONFIG
#include "generated/airframe.h"
#include "estimator.h"
#include "messages.h"
#include "downlink.h"
#include "mcu_periph/uart.h"

uint16_t adc_airspeed_val;
uint16_t airspeed_ets_offset;

#ifndef SITL // Use ADC if not in simulation

#ifndef ADC_CHANNEL_AIRSPEED
#error "ADC_CHANNEL_AIRSPEED needs to be defined to use airspeed_adc module"
#endif

#ifndef ADC_CHANNEL_AIRSPEED_NB_SAMPLES
#define ADC_CHANNEL_AIRSPEED_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif


struct adc_buf buf_airspeed;

#endif

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

float airspeed_scale;
uint16_t airspeed_bias;

void airspeed_adc_init( void ) {
  airspeed_scale = AIRSPEED_SCALE;
  airspeed_bias = AIRSPEED_BIAS;
#ifndef SITL
  adc_buf_channel(ADC_CHANNEL_AIRSPEED, &buf_airspeed, ADC_CHANNEL_AIRSPEED_NB_SAMPLES);
#endif
}

void airspeed_adc_update( void ) {
#ifndef SITL
  adc_airspeed_val = buf_airspeed.sum / buf_airspeed.av_nb_sample;
#ifdef AIRSPEED_QUADRATIC_SCALE
  float airspeed = (adc_airspeed_val - AIRSPEED_BIAS);
  if (airspeed <= 0.0f)
    airspeed = 0.0f;
  airspeed = sqrtf(airspeed) * AIRSPEED_QUADRATIC_SCALE;
#else
  float airspeed = AIRSPEED_SCALE * (adc_airspeed_val - AIRSPEED_BIAS);
#endif
  EstimatorSetAirspeed(airspeed);
#ifdef SENSOR_SYNC_SEND
  DOWNLINK_SEND_AIRSPEED_ETS(DefaultChannel, &adc_airspeed_val, &airspeed_ets_offset, &airspeed);
#endif
#else // SITL
  extern float sim_air_speed;
  EstimatorSetAirspeed(sim_air_speed);
  adc_airspeed_val = 0;
#endif //SITL
}
