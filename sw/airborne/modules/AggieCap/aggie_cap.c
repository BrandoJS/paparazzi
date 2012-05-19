/*
 * $Id: demo_module.c 3079 2009-03-11 16:55:42Z gautier $
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

#include "aggie_cap.h"

#include "subsystems/ahrs.h"
#include "subsystems/ins.h"
#include "subsystems/gps.h"


uint8_t camera_mode;

uint8_t IMU_buf[IMU_MESSAGE_SIZE];
uint8_t GPS_buf[GPS_MESSAGE_SIZE];
uint8_t COM_buf[COM_MESSAGE_SIZE];

int16_t image_phi;
int16_t image_the;
int16_t image_psi;

int32_t image_lon;
int32_t image_lat;
int32_t image_alt;
int32_t image_vsp;
int32_t image_hea;
uint32_t image_pac;
uint32_t image_sac;
uint32_t image_gsp;
uint32_t image_tow;
uint16_t image_pdp;
uint8_t image_nSV;
uint8_t image_bbb;
int32_t image_count;
uint8_t cam_on;



void init_aggie_cap(void) {
	camera_mode = 4;//CAMERA_MODE_DEFAULT;
	image_count = 0;
	cam_on = 0;

	/* Message Buffer Headers never change */
	IMU_buf[0] = MSG0;
	GPS_buf[0] = MSG0;
	COM_buf[0] = MSG0; 
	IMU_buf[1] = MSG1;
	GPS_buf[1] = MSG1;	
	COM_buf[1] = MSG1;
	IMU_buf[2] = IMU_HEADER;
	GPS_buf[2] = GPS_HEADER;
	COM_buf[2] = COM_HEADER;
	IMU_buf[3] = IMU_HEADER_SIZE;	
	GPS_buf[3] = GPS_HEADER_SIZE;	
	COM_buf[3] = COM_HEADER_SIZE;	

}

void periodic_aggie_cap(void)
{
  static uint16_t counter_sw2 = 0;
  
  
  /* Always stream IMU data*/
  if ((counter_sw2 % 2)==0) { //every 20 ms = 50Hz
	ugearIMU();
  }
  if ((counter_sw2 % 12)==1) { //every 120 ms = 8.3Hz 
	ugearGPS();
  }
  
  
  switch (camera_mode)
  {
    default:	// Idle
	
	break;
    case 1:	// Turn Cam On
	ugearCOM(TURN_CAM_ON);
        camera_mode = 0;
	cam_on = 1;
        break;
    case 2:	//Turn Cam Off
	ugearCOM(TURN_CAM_OFF);
	cam_on = 0;
        camera_mode = 0;
	break;
    case 3:	//Manual Trigger
        if (cam_on) {
		ugearCOM(CAM_TRIGGER);	
                image_count++;
        }
        camera_mode = 0;
	break; 
    case 4: //Auto
	if (cam_on) {
           if (counter_sw2 > CAM_DELAY) {
                counter_sw2 = 0;
                ugearCOM(CAM_TRIGGER);
		image_count++;
           }
 	}  else { camera_mode = 0;}
        break;
     case 5: //Stop Auto
        camera_mode = 0;
        break;
     case 6: //Restart
	ugearCOM(RESTART);
        cam_on = 0;
        camera_mode = 0;
        break;
	
  
  
  } //End time modulation
  counter_sw2++;
   
}


void ugearCOM(uint8_t command) {
  uint8_t cksum0, cksum1;
  COM_buf[4] = command;
  ugear_cksum(COM_HEADER, COM_HEADER_SIZE, (uint8_t *)COM_buf, &cksum0, &cksum1 );
  COM_buf[5] = cksum0;
  COM_buf[6] = cksum1;
  send_buf(COM_MESSAGE_SIZE,COM_buf);
}

void ugearIMU(void) {
  uint8_t cksum0, cksum1;
  
  image_phi = (int16_t)( (float)ahrs.ltp_to_body_euler.phi*2.422866068);
  image_the = (int16_t)( (float)ahrs.ltp_to_body_euler.theta*2.422866068);
  image_psi = (int16_t)( (float)ahrs.ltp_to_body_euler.psi*2.422866068);

  IMU_buf[4] = 0xFF&image_phi;
  IMU_buf[5] = 0xFF&(image_phi>>8);
  IMU_buf[6] = 0xFF&image_the;
  IMU_buf[7] = 0xFF&(image_the>>8);
  IMU_buf[8] = 0xFF&image_psi;
  IMU_buf[9] = 0xFF&(image_psi>>8);

  ugear_cksum(IMU_HEADER, IMU_HEADER_SIZE, (uint8_t *)IMU_buf, &cksum0, &cksum1 );
  IMU_buf[10] = cksum0;
  IMU_buf[11] = cksum1;
  send_buf(IMU_MESSAGE_SIZE,IMU_buf);

}

void ugearGPS(void) {
  uint8_t cksum0, cksum1;
  image_lon = gps.lla_pos.lon;
  image_lat = gps.lla_pos.lat;
  image_alt = gps.lla_pos.alt; 
  image_vsp = gps.ned_vel.z;
  image_hea = .1*DegOfRad(gps.course*.1);
  image_pac = gps.pacc;
  image_sac = gps.sacc;
  image_gsp = gps.gspeed;
  image_tow = gps.tow;
  image_pdp = gps.pdop;
  image_nSV = gps.tow;
  image_bbb = 0x44;

  for (uint8_t i = 0;i<4;i++){
	GPS_buf[4+i] = 0xFF&(image_lon>>(i*8));
  }
  for (uint8_t i = 0;i<4;i++){
	GPS_buf[8+i] = 0xFF&(image_lat>>(i*8));
  }
  for (uint8_t i = 0;i<4;i++){
	GPS_buf[12+i] = 0xFF&(image_alt>>(i*8));
  }
  for (uint8_t i = 0;i<4;i++){
	GPS_buf[16+i] = 0xFF&(image_vsp>>(i*8));
  }
  for (uint8_t i = 0;i<4;i++){
	GPS_buf[20+i] = 0xFF&(image_hea>>(i*8));
  }
  for (uint8_t i = 0;i<4;i++){
	GPS_buf[24+i] = 0xFF&(image_pac>>(i*8));
  }
  for (uint8_t i = 0;i<4;i++){
	GPS_buf[28+i] = 0xFF&(image_sac>>(i*8));
  }
  for (uint8_t i = 0;i<4;i++){
	GPS_buf[32+i] = 0xFF&(image_gsp>>(i*8));
  }
  for (uint8_t i = 0;i<4;i++){
	GPS_buf[36+i] = 0xFF&(image_tow>>(i*8));
  }
  for (uint8_t i = 0;i<2;i++){
	GPS_buf[40+i] = 0xFF&(image_pdp>>(i*8));
  }
  GPS_buf[42] = image_nSV;
  GPS_buf[43] = image_bbb;

  ugear_cksum(GPS_HEADER, GPS_HEADER_SIZE, (uint8_t *)GPS_buf, &cksum0, &cksum1 );
  GPS_buf[44] = cksum0;
  GPS_buf[45] = cksum1;
  send_buf(GPS_MESSAGE_SIZE,GPS_buf);
  

}

void send_buf(uint8_t size, uint8_t *_buf){
  for (uint8_t i = 0;i<size;i++){
  	AGCLink(Transmit(_buf[i]));
  }
}

void ugear_cksum( uint8_t hdr1, uint8_t hdr2, uint8_t *buf, uint8_t *cksum0, uint8_t *cksum1 ) {
  uint8_t c0 = 0;
  uint8_t c1 = 0;
  uint8_t i = 0;
  uint8_t size = hdr2;

  c0 += hdr1;
  c1 += c0;
  
  c0 += hdr2;
  c1 += c0;

  for ( i = 4; i < (size+4); i++ ) {
  	c0 += (uint8_t)buf[i];
    	c1 += c0;
  }

  
  *cksum0 = c0;
  *cksum1 = c1;
  

}




