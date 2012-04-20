/*
 * Copyright (C) 2010 The Paparazzi Team
 *
 * Autor: Bruzzlee
 * Angle of Attack ADC Sensor
 * US DIGITAL MA3-A10-236-N
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

#include "modules/sensors/AOA_SS_adc.h"
#include "mcu_periph/adc.h"
#include BOARD_CONFIG
#include "generated/airframe.h"
#include "estimator.h"
#include "std.h"
//Messages
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

uint16_t adc_AOA1_val;
uint16_t adc_AOA2_val;
uint16_t adc_SS_val;

//Downlink
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

#ifndef SITL // Use ADC if not in simulation

#ifndef ADC_CHANNEL_AOA1
#error "ADC_CHANNEL_AOA1 needs to be defined to use AOA_adc module"
#endif
#ifndef ADC_CHANNEL_AOA2
#error "ADC_CHANNEL_AOA2 needs to be defined to use AOA_adc module"
#endif
#ifndef ADC_CHANNEL_SS
#error "ADC_CHANNEL_SS needs to be defined to use AOA_adc module"
#endif

#ifndef ADC_CHANNEL_AOA1_NB_SAMPLES
#define ADC_CHANNEL_AOA1_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif
#ifndef ADC_CHANNEL_AOA2_NB_SAMPLES
#define ADC_CHANNEL_AOA2_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif
#ifndef ADC_CHANNEL_SS_NB_SAMPLES
#define ADC_CHANNEL_SS_NB_SAMPLES DEFAULT_AV_NB_SAMPLE
#endif

#endif
struct adc_buf buf_AOA1;
float AOA1_offset, AOA1_filter;
float AOA1, AOA1_old;
struct adc_buf buf_AOA2;
float AOA2_offset, AOA2_filter;
float AOA2, AOA2_old;
struct adc_buf buf_SS;
float SS_offset, SS_filter;
float SS, SS_old;



void AOA_SS_adc_init( void ) {

	AOA1_offset = AOA1_OFFSET;
	AOA1_filter = AOA1_FILTER;
	AOA1_old = 0;
	AOA2_offset = AOA2_OFFSET;
	AOA2_filter = AOA2_FILTER;
	AOA2_old = 0;
	SS_offset = SS_OFFSET;
	SS_filter = SS_FILTER;
	SS_old = 0;
#ifndef SITL
	adc_buf_channel(ADC_CHANNEL_AOA1, &buf_AOA1, ADC_CHANNEL_AOA1_NB_SAMPLES);
	adc_buf_channel(ADC_CHANNEL_AOA2, &buf_AOA2, ADC_CHANNEL_AOA2_NB_SAMPLES);
	adc_buf_channel(ADC_CHANNEL_SS, &buf_SS, ADC_CHANNEL_SS_NB_SAMPLES);
#endif
}

void AOA_SS_adc_update( void ) {  //10 Hz
#ifndef SITL
	adc_AOA1_val = buf_AOA1.sum / buf_AOA1.av_nb_sample;
	adc_AOA2_val = buf_AOA2.sum / buf_AOA2.av_nb_sample;
	adc_SS_val = buf_SS.sum / buf_SS.av_nb_sample;

// 	PT1 filter and convert to rad
	AOA1 = AOA1_filter * AOA1_old + (1 - AOA1_filter) * (adc_AOA1_val*(2*M_PI)/1024-M_PI+AOA1_offset);
	AOA1_old = AOA1;
	AOA2 = AOA2_filter * AOA2_old + (1 - AOA2_filter) * (adc_AOA2_val*(2*M_PI)/1024-M_PI+AOA2_offset);
	AOA2_old = AOA2;
	SS = SS_filter * SS_old + (1 - SS_filter) * (adc_SS_val*(2*M_PI)/1024-M_PI+SS_offset);
	SS_old = SS;

	RunOnceEvery(4, DOWNLINK_SEND_AOA_SS_adc(DefaultChannel, DefaultDevice, &adc_AOA1_val, &AOA1, &adc_AOA2_val, &AOA2, &adc_SS_val, &SS)); // .4s
#endif
#ifdef USE_AOA
	//EstimatorSetAOA(AOA);
#endif
}
