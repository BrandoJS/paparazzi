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


#include <math.h>
#include "std.h"

#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_ugear.h"
#include "subsystems/gps.h"

#include "math/pprz_algebra_float.h"

#include "subsystems/nav.h"
#include "math/pprz_geodetic_float.h"
#include "mcu_periph/sys_time.h"

#include "xsens_protocol.h" //in var/include
#include <string.h>

// FIXME this is still needed for fixedwing integration
#include "estimator.h"
#include "led.h"



// remotely settable
float ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
float ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;

// for heading message
float gps_estimator_psi;


// Axis definition: X axis pointing forward, Y axis pointing to the right and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise


uint8_t ugear_error;

struct UGEAR_packet UGEAR_packet;

/**************************************************/

void ahrs_update_fw_estimator( void )
{
  //GPS heading
  float course_f = (float)DegOfRad(gps.course / 1e7);
  if (course_f > 180.0) {
	course_f -= 360.0;
  }
  gps_estimator_psi = (float)RadOfDeg(course_f);
  

  // export results to estimator (output in rad)
  estimator_phi   = ahrs_float.ltp_to_body_euler.phi;
  estimator_theta = ahrs_float.ltp_to_body_euler.theta;
  estimator_psi = gps_estimator_psi;

  
}


void ahrs_init(void) {
  
  /* set ltp_to_body to zero */
  FLOAT_QUAT_ZERO(ahrs_float.ltp_to_body_quat);
  FLOAT_EULERS_ZERO(ahrs_float.ltp_to_body_euler);
  FLOAT_RMAT_ZERO(ahrs_float.ltp_to_body_rmat);
  FLOAT_RATES_ZERO(ahrs_float.body_rate);
  
  ahrs.status = AHRS_RUNNING;
  
}


/* None of the following functions are needed, but exist in main code*/
void ahrs_propagate(void)  {
}

void ahrs_update_gps(void) {

}

void ahrs_update_accel(void) {
}


void ahrs_update_mag(void) {
}



void UGEAR_packet_read_message(void) {
   
   int16_t ugear_phi, ugear_psi, ugear_theta;

    switch (UGEAR_packet.type){
        case 0:  /*gps*/
            gps.tow = UGEAR_NAV_VELNED_ITOW(UGEAR_packet.ugear_msg_buf);
            gps.week = 0; // FIXME
	    gps.fix = 3;  // FIXME currently forced into 3d fix
 
	    gps.ecef_pos.x = 0; // FIXME
	    gps.ecef_pos.y = 0; // FIXME
	    gps.ecef_pos.z = 0; // FIXME

	    gps.pacc = UGEAR_NAV_SOL_Pacc(UGEAR_packet.ugear_msg_buf);

	    gps.ecef_vel.x = 0; // FIXME
            gps.ecef_vel.y = 0; // FIXME
            gps.ecef_vel.z = 0; // FIXME 

	    gps.sacc = UGEAR_NAV_SOL_Sacc(UGEAR_packet.ugear_msg_buf);
            gps.pdop = UGEAR_NAV_SOL_PDOP(UGEAR_packet.ugear_msg_buf);
            gps.num_sv = UGEAR_NAV_SOL_numSV(UGEAR_packet.ugear_msg_buf);

            gps.lla_pos.lat = RadOfDeg(UGEAR_NAV_POSLLH_LAT(UGEAR_packet.ugear_msg_buf));
            gps.lla_pos.lon = RadOfDeg(UGEAR_NAV_POSLLH_LON(UGEAR_packet.ugear_msg_buf));
            gps.lla_pos.alt = UGEAR_NAV_POSLLH_HEIGHT(UGEAR_packet.ugear_msg_buf);
            gps.hmsl = UGEAR_NAV_POSLLH_HEIGHT(UGEAR_packet.ugear_msg_buf)*10; // UGEAR sends in cm, struct gps likes mm

            /* Computes from (lat, long) in the referenced UTM zone */
	    struct LlaCoor_f lla_f;
	    lla_f.lat = ((float) gps.lla_pos.lat) / 1e7;
	    lla_f.lon = ((float) gps.lla_pos.lon) / 1e7;
	    struct UtmCoor_f utm_f;
	    utm_f.zone = nav_utm_zone0;
	    /* convert to utm */
	    utm_of_lla_f(&utm_f, &lla_f);
	    /* copy results of utm conversion */
	    gps.utm_pos.east = utm_f.east*100;
	    gps.utm_pos.north = utm_f.north*100;
	    gps.utm_pos.alt = utm_f.alt*1000;
            gps.utm_pos.zone = nav_utm_zone0;

            gps.speed_3d = 0; // FIXME
            gps.gspeed = UGEAR_NAV_VELNED_GSpeed(UGEAR_packet.ugear_msg_buf);
            gps.ned_vel.x = 0; // FIXME
            gps.ned_vel.y = 0; // FIXME
            gps.ned_vel.z = UGEAR_NAV_POSLLH_VD(UGEAR_packet.ugear_msg_buf);
            gps.course = RadOfDeg(UGEAR_NAV_VELNED_Heading(UGEAR_packet.ugear_msg_buf)*10)*10; /*in decdegree */
            gps_available = TRUE; // Let AP know a GPS message has been received
	    gps.last_fix_time = cpu_time_sec; // record last GPS message
	    #ifdef GPS_LED
      	    LED_TOGGLE(GPS_LED);
	    #endif
            break;
        case 1:  /*IMU*/
            ugear_phi = UGEAR_IMU_PHI(UGEAR_packet.ugear_msg_buf);
            ugear_psi = UGEAR_IMU_PSI(UGEAR_packet.ugear_msg_buf);
            ugear_theta = UGEAR_IMU_THE(UGEAR_packet.ugear_msg_buf);
	    ahrs_float.ltp_to_body_euler.phi  = ((float)ugear_phi/10000 - ins_roll_neutral); //ugear outputs degrees
            ahrs_float.ltp_to_body_euler.psi = 0;
            ahrs_float.ltp_to_body_euler.theta  = ((float)ugear_theta/10000 - ins_pitch_neutral);

            break;
        case 2:  /*Error Messages*/
            ugear_error = UGEAR_ERROR(UGEAR_packet.ugear_msg_buf);
            break;
	default: break;

    }

}

/* UGEAR Packet Collection */

void UGEAR_packet_parse( uint8_t c ) {

/*checksum go first*/
  if (UGEAR_packet.status < GOT_PAYS) {
    UGEAR_packet.ck_a += c;
    UGEAR_packet.ck_b += UGEAR_packet.ck_a;
  }
  switch (UGEAR_packet.status) {
  case UNINIT:
    if (c == UGEAR_SYNC1)
      UGEAR_packet.status++;
    break;
  case GOT_SYNC1:
    if (c == UGEAR_SYNC2){
      UGEAR_packet.ck_a = 0;
      UGEAR_packet.ck_b = 0;
      UGEAR_packet.status++;
    } else {
    UGEAR_packet.status = UNINIT; // If sync2 doesn't match, packet got bad :(
    }
    
    break;
  case GOT_SYNC2:
    UGEAR_packet.type = c;
    UGEAR_packet.status++;
    if (UGEAR_packet.type > 2) {
    	UGEAR_packet.status = UNINIT; // no ugear packets with type > 2
    }
    break;
  case GOT_ID: //How big is this packet goes into *.len
    UGEAR_packet.len = c;
    UGEAR_packet.msg_idx = 0;
    UGEAR_packet.status++;
    break;
  case GOT_LEN: //Fill message buffer with c
    UGEAR_packet.ugear_msg_buf[UGEAR_packet.msg_idx] = c;
    UGEAR_packet.msg_idx++;
    if (UGEAR_packet.msg_idx >= UGEAR_packet.len) { //until we've filled buffer
      UGEAR_packet.status++;
    }
    break;
  case GOT_PAYS:
    if (c == UGEAR_packet.ck_a) {
       UGEAR_packet.status++;
    } else {
       UGEAR_packet.status = UNINIT; // If checksum a doesn't match, packet got bad :( 
    }
    break;
  case GOT_CHECKSUM1:
    if (c == UGEAR_packet.ck_b) {
        UGEAR_packet.msg_available = TRUE; // If checksum a doesn't match, packet got bad :( 
    } 
    UGEAR_packet.status = UNINIT; //either way, start over
    break;
  default: UGEAR_packet.status = UNINIT;
  }
}
