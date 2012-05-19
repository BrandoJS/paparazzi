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

#include "modules/sensors/airspeed_adc_adv.h"
#include "modules/sensors/baro_bmp.h"
#include "mcu_periph/adc.h"
#include BOARD_CONFIG
#include "generated/airframe.h"
#include "estimator.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
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



float airspeed_scale;
uint16_t airspeed_bias;
float rho;
float true_airspeed;

void airspeed_adc_adv_init( void ) {
  airspeed_scale = AIRSPEED_SCALE;
  airspeed_bias = AIRSPEED_BIAS;
  true_airspeed = 0;
#ifndef SITL
  adc_buf_channel(ADC_CHANNEL_AIRSPEED, &buf_airspeed, ADC_CHANNEL_AIRSPEED_NB_SAMPLES);
#endif
}

void airspeed_adc_adv_update( void ) {
#ifndef SITL
  adc_airspeed_val = buf_airspeed.sum / buf_airspeed.av_nb_sample;
  float pas = airspeed_scale * (adc_airspeed_val - airspeed_bias);
  
  /* True Airspeed Measurement with baro */
  #ifdef USE_BARO
    rho = baro_bmp_pressure/(287.058*(baro_bmp_temperature/10.+273.15)); 
  #else
    rho = 1.225;
  #endif
  
  if (pas < 0) 
    true_airspeed = 0;
  else
    true_airspeed = true_airspeed + 0.5*(sqrtf(2*pas/rho)-true_airspeed); //LPF at 1.59Hz

  EstimatorSetAirspeed(true_airspeed);
#else // SITL
  extern float sim_air_speed;
  EstimatorSetAirspeed(sim_air_speed);
  adc_airspeed_val = 0;
#endif //SITL
}
