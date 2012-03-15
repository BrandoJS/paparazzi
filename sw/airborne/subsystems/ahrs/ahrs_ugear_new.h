/*
 * Released under Creative Commons License
 *
 * 2010 The Paparazzi Team
 *
 *
 *
 */

/** \file ahrs_gx3.h
 *  \brief Use the GX3 as AHRS
 *
 */

#ifndef AHRS_UGEAR_H
#define AHRS_UGEAR_H

#include <inttypes.h>
#include "math/pprz_algebra_float.h"
#include "generated/airframe.h"
#include "mcu_periph/uart.h"

#define UNINIT      0
#define GOT_SYNC1   1
#define GOT_SYNC2   2
#define GOT_ID          3
#define GOT_LEN         4
#define GOT_PAYS     5
#define GOT_CHECKSUM1   6
#define GOT_CHECKSUM2   7

#define UGEAR_SYNC1 0x93
#define UGEAR_SYNC2 0xE0
#define UGEAR_MAX_PAYLOAD 44
#define IMU_PACKET_SIZE 12
#define RAD2DEG 57.3
#define WrapUp(x) (x < 0 ? x+3600 : x)

#define __UGEARLink(dev, _x) dev##_x
#define _UGEARLink(dev, _x)  __UGEARLink(dev, _x)
#define UGEARLink(_x) _UGEARLink(UGEAR_LINK, _x)

#define UGEARBuffer() UGEARLink(ChAvailable())


//Pull data out of message buffer
#define UGEAR_NAV_SOL_GPSfix(_ubx_payload) (uint8_t)(*((uint8_t*)_ubx_payload))
#define UGEAR_NAV_POSLLH_LON(_ubx_payload) (int32_t)(*((uint8_t*)_ubx_payload+1)|*((uint8_t*)_ubx_payload+1+1)<<8|((int32_t)*((uint8_t*)_ubx_payload+1+2))<<16|((int32_t)*((uint8_t*)_ubx_payload+1+3))<<24)
#define UGEAR_NAV_POSLLH_LAT(_ubx_payload) (int32_t)(*((uint8_t*)_ubx_payload+1+4)|*((uint8_t*)_ubx_payload+1+1+4)<<8|((int32_t)*((uint8_t*)_ubx_payload+1+2+4))<<16|((int32_t)*((uint8_t*)_ubx_payload+1+3+4))<<24)
#define UGEAR_NAV_POSLLH_HEIGHT(_ubx_payload) (int32_t)(*((uint8_t*)_ubx_payload+1+8)|*((uint8_t*)_ubx_payload+1+1+8)<<8|((int32_t)*((uint8_t*)_ubx_payload+1+2+8))<<16|((int32_t)*((uint8_t*)_ubx_payload+1+3+8))<<24)
#define UGEAR_NAV_POSLLH_VD(_ubx_payload) (int32_t)(*((uint8_t*)_ubx_payload+1+12)|*((uint8_t*)_ubx_payload+1+1+12)<<8|((int32_t)*((uint8_t*)_ubx_payload+1+2+12))<<16|((int32_t)*((uint8_t*)_ubx_payload+1+3+12))<<24)
#define UGEAR_NAV_VELNED_Heading(_ubx_payload) (int32_t)(*((uint8_t*)_ubx_payload+1+16)|*((uint8_t*)_ubx_payload+1+1+16)<<8|((int32_t)*((uint8_t*)_ubx_payload+1+2+16))<<16|((int32_t)*((uint8_t*)_ubx_payload+1+3+16))<<24)
#define UGEAR_NAV_SOL_Pacc(_ubx_payload) (uint32_t)(*((uint8_t*)_ubx_payload+1+20)|*((uint8_t*)_ubx_payload+1+1+20)<<8|((uint32_t)*((uint8_t*)_ubx_payload+1+2+20))<<16|((uint32_t)*((uint8_t*)_ubx_payload+1+3+20))<<24)
#define UGEAR_NAV_SOL_Sacc(_ubx_payload) (uint32_t)(*((uint8_t*)_ubx_payload+1+24)|*((uint8_t*)_ubx_payload+1+1+24)<<8|((uint32_t)*((uint8_t*)_ubx_payload+1+2+24))<<16|((uint32_t)*((uint8_t*)_ubx_payload+1+3+24))<<24)
#define UGEAR_NAV_VELNED_GSpeed(_ubx_payload) (uint32_t)(*((uint8_t*)_ubx_payload+1+28)|*((uint8_t*)_ubx_payload+1+1+28)<<8|((uint32_t)*((uint8_t*)_ubx_payload+1+2+28))<<16|((uint32_t)*((uint8_t*)_ubx_payload+1+3+28))<<24)
#define UGEAR_NAV_VELNED_ITOW(_ubx_payload) (uint32_t)(*((uint8_t*)_ubx_payload+1+32)|*((uint8_t*)_ubx_payload+1+1+32)<<8|((uint32_t)*((uint8_t*)_ubx_payload+1+2+32))<<16|((uint32_t)*((uint8_t*)_ubx_payload+1+3+32))<<24)
#define UGEAR_NAV_SOL_PDOP(_ubx_payload) (uint16_t)(*((uint8_t*)_ubx_payload+1+36)|*((uint8_t*)_ubx_payload+1+1+36)<<8)
#define UGEAR_NAV_SOL_numSV(_ubx_payload) (uint8_t)(*((uint8_t*)_ubx_payload+1+38))

#define UGEAR_IMU_PHI(_ubx_payload) (uint32_t)((uint32_t)(*((uint8_t*)_ubx_payload+0))|(uint32_t)(*((uint8_t*)_ubx_payload+1))<<8|(uint32_t)(*((uint8_t*)_ubx_payload+2))<<16|(uint32_t)(*((uint8_t*)_ubx_payload+3))<<24)
#define UGEAR_IMU_THE(_ubx_payload) (uint32_t)((uint32_t)(*((uint8_t*)_ubx_payload+4))|(uint32_t)(*((uint8_t*)_ubx_payload+5))<<8|(uint32_t)(*((uint8_t*)_ubx_payload+6))<<16|(uint32_t)(*((uint8_t*)_ubx_payload+7))<<24)
#define UGEAR_IMU_PSI(_ubx_payload) (uint32_t)((uint32_t)(*((uint8_t*)_ubx_payload+8))|(uint32_t)(*((uint8_t*)_ubx_payload+9))<<8|(uint32_t)(*((uint8_t*)_ubx_payload+10))<<16|(uint32_t)(*((uint8_t*)_ubx_payload+11))<<24)

#define UGEAR_ERROR(_ubx_payload) (uint8_t)(*((uint8_t*)_ubx_payload))

extern void UGEAR_packet_read_message(void);
extern void UGEAR_packet_parse(uint8_t c);

extern float ins_roll_neutral;
extern float ins_pitch_neutral;

extern uint8_t ugear_error;

/*define the following varables for communication with ugear by haiyang 20080508*/
extern float ugear_phi;
extern float ugear_psi;
extern float ugear_theta; 

void ahrs_update_fw_estimator(void);


struct UGEAR_packet {
  bool_t  msg_available;
  uint8_t ugear_msg_buf[UGEAR_MAX_PAYLOAD] __attribute__ ((aligned));

  uint8_t  status;
  uint8_t  msg_idx;
  uint8_t  len;
  uint8_t  type;
  uint8_t  ck_a, ck_b;
};

extern struct UGEAR_packet UGEAR_packet;


#define AhrsEvent(_sol_available_callback) {				\
    if (UGEARBuffer()) {							\
      ReadUGEARBuffer();							\
    }									\
    if (UGEAR_packet.msg_available) {				\
      UGEAR_packet_read_message();					\
      _sol_available_callback();					\
      UGEAR_packet.msg_available = FALSE;				\
    }									\
}

#define ReadUGEARBuffer() {					\
    while (UGEARLink(ChAvailable())&&!UGEAR_packet.msg_available)	\
      UGEAR_packet_parse(UGEARLink(Getch()));			\
  }
#endif


