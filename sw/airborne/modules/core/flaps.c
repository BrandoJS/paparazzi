/*
 * $Id$
 *
 * Copyright (C) 2010 Martin Mueller
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

#include "flaps.h"
#include "inter_mcu.h"
#include "generated/airframe.h"


int32_t flaps_min;
int32_t flaps_max;
bool_t flaps_engaged;

void flaps_init ( void ){
	flaps_min = FLAPS_MIN;
	flaps_max = FLAPS_MAX;
	flaps_engaged = 0;
}

void periodic_flaps(void) {
	if (flaps_engaged){
		ap_state->commands[COMMAND_FLAP] = flaps_max;	
	} else {
		ap_state->commands[COMMAND_FLAP] = flaps_min;		
	}
}

