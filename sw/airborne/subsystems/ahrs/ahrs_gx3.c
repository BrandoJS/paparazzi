/*
 * Released under Creative Commons License
 *
 * 2010 The Paparazzi Team
 *
 *
 * Based on Code by Jordi Munoz and William Premerlani, Supported by Chris Anderson (Wired) and Nathan Sindle (SparkFun).
 * Version 1.0 for flat board updated by Doug Weibel and Jose Julio
 *
 * Modified at Hochschule Bremen, Germany
 * 2010 Heinrich Warmers, Christoph Niemann, Oliver Riesener
 *
 */



#include "std.h"

#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_gx3.h"
#include "subsystems/imu.h"
#include "subsystems/gps.h"

#include "math/pprz_algebra_float.h"

#ifdef USE_GPS
#include "subsystems/gps.h"
#endif

#include <string.h>

// FIXME this is still needed for fixedwing integration
#include "estimator.h"
#include "led.h"

/* FIXME Debugging Only
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/downlink/downlink.h"
*/

#define GX3_HEADER 0xC8

/* parser status */
#define WAITING		0
#define READING		1
#define DONE		2


// remotely settable
float ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
float ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;

// for heading message
float gps_estimator_psi;
float gx3_estimator_psi;


// Axis definition: X axis pointing forward, Y axis pointing to the right and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise

static inline void compute_body_orientation_and_rates(void);

struct GX3 gx3;

struct GX3_packet GX3_packet;

/**************************************************/

void ahrs_update_fw_estimator( void )
{
  //GPS heading
  float course_f = (float)DegOfRad(gps.course / 1e7);
  if (course_f > 180.0) {
	course_f -= 360.0;
  }
  gps_estimator_psi = (float)RadOfDeg(course_f);
  

  compute_body_orientation_and_rates();

  // export results to estimator (output in rad)
  estimator_phi   = (float)RadOfDeg((ahrs.ltp_to_body_euler.phi * .0139882) - ins_roll_neutral);
  estimator_theta = (float)RadOfDeg((ahrs.ltp_to_body_euler.theta * .0139882) - ins_pitch_neutral);
  gx3_estimator_psi   = (float)RadOfDeg(ahrs.ltp_to_body_euler.psi * .0139882  - ahrs_mag_offset) ;  //ahrs_mag_offset in degs

  estimator_psi = gps_estimator_psi;

  estimator_p = RATE_FLOAT_OF_BFP(ahrs.body_rate.p);
  estimator_q = RATE_FLOAT_OF_BFP(ahrs.body_rate.q);

}


void ahrs_init(void) {
  
  INT_EULERS_ZERO(ahrs.ltp_to_body_euler);
  INT_EULERS_ZERO(ahrs.ltp_to_imu_euler);
  INT32_QUAT_ZERO(ahrs.ltp_to_body_quat);
  INT32_QUAT_ZERO(ahrs.ltp_to_imu_quat);
  INT_RATES_ZERO(ahrs.body_rate);
  INT_RATES_ZERO(ahrs.imu_rate);
  
  #ifdef IMU_MAG_OFFSET
    ahrs_mag_offset = IMU_MAG_OFFSET;
  #else
    ahrs_mag_offset = 0.;
  #endif
  
  //Needed to set orientations
  imu_init();
  ahrs.status = AHRS_RUNNING;
  
}

void imu_impl_init(void) {
     //4 byte command for Continous Mode
   GX3Link(Transmit(0xc4));
   GX3Link(Transmit(0xc1));
   GX3Link(Transmit(0x29));
   GX3Link(Transmit(0xc8));

   
      
   GX3_packet.status = 0;
   GX3_packet.msg_idx = 0;
}


void ahrs_propagate(void)  {
}

void ahrs_update_gps(void) {

}

void ahrs_update_accel(void) {
}


void ahrs_update_mag(void) {
}

/*
 * Compute body orientation and rates from imu orientation and rates
 */
static inline void compute_body_orientation_and_rates(void) {
  //All in BFP
  INT32_QUAT_COMP_INV(ahrs.ltp_to_body_quat, ahrs.ltp_to_imu_quat, imu.body_to_imu_quat);
  INT32_RMAT_COMP_INV(ahrs.ltp_to_body_rmat, ahrs.ltp_to_imu_rmat, imu.body_to_imu_rmat);
  INT32_EULERS_OF_RMAT(ahrs.ltp_to_body_euler, ahrs.ltp_to_body_rmat);
  INT32_RMAT_TRANSP_RATEMULT(ahrs.body_rate, imu.body_to_imu_rmat, ahrs.imu_rate);

}

void GX3_packet_read_message(void) {
    
	#ifdef AHRS_CPU_LED
    	LED_ON(AHRS_CPU_LED);
	#endif

    struct FloatVect3 GX3_accel;
    struct FloatRates GX3_rate;
    struct FloatRMat  GX3_rmat;



        GX3_accel.x 	= bef(&GX3_packet.msg_buf[1]);
        GX3_accel.y 	= bef(&GX3_packet.msg_buf[5]);
        GX3_accel.z 	= bef(&GX3_packet.msg_buf[9]);
        GX3_rate.p  	= bef(&GX3_packet.msg_buf[13]);
        GX3_rate.q  	= bef(&GX3_packet.msg_buf[17]);
        GX3_rate.r  	= bef(&GX3_packet.msg_buf[21]);
        GX3_rmat.m[0] 	= bef(&GX3_packet.msg_buf[25]);
        GX3_rmat.m[1] 	= bef(&GX3_packet.msg_buf[29]);
        GX3_rmat.m[2] 	= bef(&GX3_packet.msg_buf[33]);
        GX3_rmat.m[3] 	= bef(&GX3_packet.msg_buf[37]);
        GX3_rmat.m[4] 	= bef(&GX3_packet.msg_buf[41]);
        GX3_rmat.m[5] 	= bef(&GX3_packet.msg_buf[45]);
        GX3_rmat.m[6] 	= bef(&GX3_packet.msg_buf[49]);
        GX3_rmat.m[7] 	= bef(&GX3_packet.msg_buf[53]);
        GX3_rmat.m[8] 	= bef(&GX3_packet.msg_buf[57]);

        /* IMU accel */
	// GX provides g for accel, rad/s for gyro
        VECT3_SMUL(GX3_accel,GX3_accel, 9.80665); //Convert g into m/s2
        ACCELS_BFP_OF_REAL(imu.accel, GX3_accel); //


        /* IMU rate */
        RATES_BFP_OF_REAL(imu.gyro, GX3_rate);
        RATES_BFP_OF_REAL(ahrs.imu_rate, GX3_rate);

        /* LTP to IMU rotation matrix to Eulers to Quat*/
        RMAT_BFP_OF_REAL(ahrs.ltp_to_imu_rmat, GX3_rmat);
        INT32_EULERS_OF_RMAT(ahrs.ltp_to_imu_euler, ahrs.ltp_to_imu_rmat);
        
	/* Add Mag Offset */
	//ahrs.ltp_to_body_euler.psi += -ANGLE_BFP_OF_REAL(ahrs_mag_offset); 
        
	#ifdef AHRS_CPU_LED
    	LED_OFF(AHRS_CPU_LED);
	#endif
        
        INT32_QUAT_OF_EULERS(ahrs.ltp_to_imu_quat, ahrs.ltp_to_imu_euler);

}

/* GX3 Packet Collection */

void GX3_packet_parse( uint8_t c ) {

  switch (GX3_packet.status) {
  case WAITING:
    GX3_packet.msg_idx = 0;
    if (c == GX3_HEADER) {
      GX3_packet.status++;
      GX3_packet.msg_buf[GX3_packet.msg_idx] = c;
      GX3_packet.msg_idx++;
    }
    break;
  case READING:
    GX3_packet.msg_buf[GX3_packet.msg_idx] =  c;
    GX3_packet.msg_idx++;
    if (GX3_packet.msg_idx >= GX3_MSG_LEN) {
      GX3_packet.status++;
    }
    break;
  case DONE:
    GX3_packet.msg_available = GX3_verify_chk(GX3_packet.msg_buf);
    GX3_packet.status = 0;
    break;
  default:
    GX3_packet.status = 0;
    break;
  }
}
