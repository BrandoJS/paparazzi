/*
 * $Id: demo_module.h 3079 2009-03-11 16:55:42Z gautier $
 *
 * Copyright (C) 2009  Gautier Hattenberger
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

#include "sdcard.h"

#include "subsystems/ahrs.h"
#include "subsystems/gps.h"
#include "actuators.h"
#include <string.h>
#include "estimator.h"


uint8_t STATE_buf[STATE_MESSAGE_SIZE];

void init_sdcard(void){
	STATE_buf[STATE_MESSAGE_SIZE-2] = 0x0D; //CR
	STATE_buf[STATE_MESSAGE_SIZE-1] = 0x0A; //LF
}

void periodic_sdcard(void) {
	
 	int16_t sd_phi = (int16_t)( estimator_phi * 10000);
	int16_t sd_the = (int16_t)( estimator_theta * 10000 );
	int16_t sd_psi = (int16_t)( estimator_psi * 10000 );	
	STATE_buf[0] = 0xFF&sd_phi;
	STATE_buf[1] = 0xFF&(sd_phi>>8);
	STATE_buf[2] = 0xFF&sd_the;
	STATE_buf[3] = 0xFF&(sd_the>>8);
	STATE_buf[4] = 0xFF&sd_psi;
	STATE_buf[5] = 0xFF&(sd_psi>>8);
	for (uint8_t i = 0;i<SERVOS_NB;i++) {
		STATE_buf[2*i+6] = 0xFF&actuators[i];
		STATE_buf[2*i+7] = 0xFF&(actuators[i]>>8);
	}
    
	send_buf(STATE_MESSAGE_SIZE,STATE_buf);
		
}

void send_buf(uint8_t size, uint8_t *_buf){
  for (uint8_t i = 0;i<size;i++){
  	SDCLink(Transmit(_buf[i]));
  }
}
