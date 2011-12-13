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

#ifndef AHRS_GX3_H
#define AHRS_GX3_H

#include <inttypes.h>
#include "math/pprz_algebra_float.h"
#include "generated/airframe.h"
#include "mcu_periph/uart.h"

#define GX3_MAX_PAYLOAD 128
#define GX3_MSG_LEN 67

#define __GX3Link(dev, _x) dev##_x
#define _GX3Link(dev, _x)  __GX3Link(dev, _x)
#define GX3Link(_x) _GX3Link(GX3_LINK, _x)

#define GX3Buffer() GX3Link(ChAvailable())

struct GX3 {
  struct FloatVect3 	G_acc;    /* GX3 accel		*/
  struct FloatRates	G_gyr;    /* GX3 rates		*/
  struct FloatRMat	G_rmat;    /* GX3 rmat		*/
  
};

extern struct GX3 gx3;

extern struct GX3_packet GX3_packet;

extern void GX3_packet_read_message(void);
extern void GX3_packet_parse(uint8_t c);

extern float ins_roll_neutral;
extern float ins_pitch_neutral;

extern float gps_estimator_psi;
extern float gx3_estimator_psi;


void ahrs_update_fw_estimator(void);

static inline float bef(volatile uint8_t *c) { //Big Endian to Float
    float f;
    int8_t * p;
    p = ((int8_t *)&f)+3;
    *p-- = *c++;
    *p-- = *c++;
    *p-- = *c++;
    *p = *c;
    return f;
}

static inline bool_t GX3_verify_chk(volatile uint8_t *buff_add) {
    uint16_t i,chk_calc;
    chk_calc = 0;
    for (i=0;i<GX3_MSG_LEN-2;i++) {
        chk_calc += (uint8_t)*buff_add++;
    }
    return (chk_calc == ( (((uint16_t)*buff_add)<<8) + (uint8_t)*(buff_add+1) ));
}




struct GX3_packet {
  bool_t  msg_available;
  uint8_t msg_buf[GX3_MAX_PAYLOAD] __attribute__ ((aligned));

  uint8_t  status;
  uint8_t  msg_idx;

};



#define AhrsEvent(_sol_available_callback) {				\
    if (GX3Buffer()) {							\
      ReadGX3Buffer();							\
    }									\
    if (GX3_packet.msg_available) {				\
      GX3_packet_read_message();					\
      _sol_available_callback();					\
      GX3_packet.msg_available = FALSE;				\
    }									\
}

#define ReadGX3Buffer() {					\
    while (GX3Link(ChAvailable())&&!GX3_packet.msg_available)	\
      GX3_packet_parse(GX3Link(Getch()));			\
  }
#endif


