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

/** aggie_cap.h
 *  Aggie Cap Interface for Lisa plus Camera Track Control
 *
 * 
 */

#ifndef AGGIE_CAP_MODULE_H
#define AGGIE_CAP_MODULE_H

#include <inttypes.h>
#include "mcu_periph/uart.h"

#define __AGCLink(dev, _x) dev##_x
#define _AGCLink(dev, _x)  __AGCLink(dev, _x)
#define AGCLink(_x) _AGCLink(AGC_LINK, _x)

#define AGCBuffer() AGCLink(ChAvailable())


#ifndef CAMERA_MODE_DEFAULT
#define CAMERA_MODE_DEFAULT 2
#endif


#define MSG0 0x93
#define MSG1 0xE0
#define GPS_HEADER 0x00
#define IMU_HEADER 0x01
#define COM_HEADER 0x02

#define IMU_HEADER_SIZE 0x06
#define GPS_HEADER_SIZE 0x28
#define COM_HEADER_SIZE 0x01

#define IMU_MESSAGE_SIZE 12
#define GPS_MESSAGE_SIZE 46
#define COM_MESSAGE_SIZE 7

#define TURN_CAM_ON	0x01
#define TURN_CAM_OFF	0x04
#define CAM_TRIGGER	0x02
#define RESTART		0x08


extern uint8_t camera_mode;

extern uint8_t cam_on;

extern int16_t image_phi;
extern int16_t image_the;
extern int16_t image_psi;

extern int32_t image_count;
void init_aggie_cap(void);
void periodic_aggie_cap(void);

void ugearIMU(void);
void ugearGPS(void);
void ugearCOM(uint8_t command);
void ugear_cksum( uint8_t hdr1, uint8_t hdr2, uint8_t *buf, uint8_t *cksum0, uint8_t *cksum1 );
void send_buf(uint8_t size,uint8_t *_buf);


#endif
