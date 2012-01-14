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
#include "subsystems/ahrs/ahrs_ugear.h"
#include "subsystems/gps.h"

#include "math/pprz_algebra_float.h"

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


struct UGEAR ugear;

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
   
   float ins_phi, ins_psi, ins_theta;
   int16_t ugear_phi, ugear_psi, ugear_theta;

    switch (UGEAR_packet.type){
        case 0:  /*gps*/
            gps.tow = UGEAR_NAV_VELNED_ITOW(ugear_msg_buf);
            gps.week = 0; // FIXME
	    gps.fix = 3;  // FIXME currently forced into 3d fix
 
	    gps.ecef_pos.x = 0; // FIXME
	    gps.ecef_pos.y = 0; // FIXME
	    gps.ecef_pos.z = 0; // FIXME

	    gps.pacc = UGEAR_NAV_SOL_Pacc(ugear_msg_buf);

	    gps.ecef_vel.x = 0; // FIXME
            gps.ecef_vel.y = 0; // FIXME
            gps.ecef_vel.z = 0; // FIXME 

	    gps.sacc = UGEAR_NAV_SOL_Sacc(ugear_msg_buf);
            gps.pdop = UGEAR_NAV_SOL_PDOP(ugear_msg_buf);
            gps.num_sv = UGEAR_NAV_SOL_numSV(ugear_msg_buf);

            gps.lla_pos.lat = UGEAR_NAV_POSLLH_LAT(ugear_msg_buf);
            gps.lla_pos.lon = UGEAR_NAV_POSLLH_LON(ugear_msg_buf);
            gps.lla_pos.alt = UGEAR_NAV_POSLLH_HEIGHT(ugear_msg_buf);

            gps.hmsl = 0; // FIXME

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
            gps.gspeed = UGEAR_NAV_VELNED_GSpeed(ugear_msg_buf);
            gps.ned_vel.x = 0; // FIXME
            gps.ned_vel.y = 0; // FIXME
            gps.ned_vel.z = - UGEAR_NAV_POSLLH_VD(ugear_msg_buf);
            gps.course = UGEAR_NAV_VELNED_Heading(ugear_msg_buf)/10000; /*in decdegree */
            
            break;
        case 1:  /*IMU*/
            ugear_phi = UGEAR_IMU_PHI(ugear_msg_buf);
            ugear_psi = UGEAR_IMU_PSI(ugear_msg_buf);
            ugear_theta = UGEAR_IMU_THE(ugear_msg_buf);
            ahrs_float.ltp_to_body_euler.phi  = RadofDeg((float)ugear_phi/10000 - ins_roll_neutral); //ugear outputs degrees
            ahrs_float.ltp_to_body_euler.psi = 0;
            ahrs_float.ltp_to_body_euler.theta  = RadofDeg((float)ugear_theta/10000 - ins_pitch_neutral);

            break;
        case 2:  /*GPS status*/
            gps.nb_channels = XSENS_GPSStatus_nch(ugear_msg_buf);
            uint8_t is;
            for(is = 0; is < gps.nb_channels; is++) {
                gps.svinfos[ch].svid = XSENS_GPSStatus_svid(ugear_msg_buf, is);
                gps.svinfos[ch].flags = XSENS_GPSStatus_bitmask(ugear_msg_buf, is);
                gps.svinfos[ch].qi = XSENS_GPSStatus_qi(ugear_msg_buf, is);
                gps.svinfos[ch].cno = XSENS_GPSStatus_cnr(ugear_msg_buf, is);
                gps.svinfos[ch].elev = 0;
                gps.svinfos[ch].azim = 0;
            }
            break;
	default: break;

    }

}

/* UGEAR Packet Collection */

void UGEAR_packet_parse( uint8_t c ) {

/*checksum go first*/
  if (UGEAR_packet.status < GOT_PAYLOAD) {
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
    UGEAR_packet.msg_buf[ugear_msg_idx] = c;
    UGEAR_packet.msg_idx++;
    if (UGEAR_packet.msg_idx >= UGEAR_packet.len) { //until we've filled buffer
      UGEAR_packet.status++;
    }
    break;
  case GOT_PAYLOAD:
    if (c == UGEAR_packet.ck_a) {
       UGEAR_packet.status++;
    } else {
       UGEAR_packet.status = UNINIT; // If checksum a doesn't match, packet got bad :( 
    }
    break;
  case GOT_CHECKSUM1:
    if (c == UGEAR_packet.ck_b) {
        UGEAR_packet.msg_received = TRUE; // If checksum a doesn't match, packet got bad :( 
    } 
    UGEAR_packet.status = UNINIT; //either way, start over
    break;
  default: UGEAR_packet.status = UNINIT;
  }
}
