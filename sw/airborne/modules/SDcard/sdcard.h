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
#ifndef SDCARD_MODULE_H
#define SDCARD_MODULE_H

#include <inttypes.h>
#include "mcu_periph/uart.h"
#include "actuators.h"

#define __SDCLink(dev, _x) dev##_x
#define _SDCLink(dev, _x)  __SDCLink(dev, _x)
#define SDCLink(_x) _SDCLink(SDC_LINK, _x)

#define SDCBuffer() SDCLink(ChAvailable())


#define STATE_MESSAGE_SIZE SERVOS_NB*2+6+2

void send_buf(uint8_t size,uint8_t *_buf);
void init_sdcard(void);
void periodic_sdcard(void);

#endif

