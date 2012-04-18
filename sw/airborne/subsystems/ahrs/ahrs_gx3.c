/*
 * Released under Creative Commons License
 *
 * 2010 The Paparazzi Team
 *
 *
 *
 *
 */



#include "std.h"

#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_gx3.h"
#include "subsystems/imu.h"
#include "subsystems/gps.h"
#include "led.h"
#include "mcu_periph/sys_time.h"

#include "math/pprz_algebra_float.h"

#ifdef USE_GPS
#include "subsystems/gps.h"
#endif

#include <string.h>



#define GX3_HEADER 0xC8

/* parser status */
#define WAITING		0
#define READING		1
#define DONE		2


#define F_UPDATE 512

// Axis definition: X axis pointing forward, Y axis pointing to the right and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise

static inline void compute_body_orientation_and_rates(void);

struct GX3_packet GX3_packet;

uint32_t gx3_delay_time;
bool_t gx3_delay_done;
uint32_t GX3_time;
uint32_t GX3_ltime;
uint16_t GX3_chksm;
uint16_t GX3_calcsm;
uint32_t gx3_stop_time;

float AHRS_freq;
float GX3_freq;


#ifdef AHRS_UPDATE_FW_ESTIMATOR

// remotely settable
float ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
float ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;

// for heading message
float gps_estimator_psi;
float gx3_estimator_psi;

int32_t gx3_psi;

// FIXME this is still needed for fixedwing integration
#include "estimator.h"
#include "led.h"
/**************************************************/

void ahrs_update_fw_estimator( void )
{
  //GPS heading
  float course_f = (float)DegOfRad(gps.course / 1e7);
  if (course_f > 180.0) {
	course_f -= 360.0;
  }
  gps_estimator_psi = (float)RadOfDeg(course_f);
  gx3_estimator_psi = (float)RadOfDeg((ahrs.ltp_to_body_euler.psi * .0139882));

  compute_body_orientation_and_rates();

  // export results to estimator (output in rad)
  estimator_phi   = (float)RadOfDeg((ahrs.ltp_to_body_euler.phi * .0139882) - ins_roll_neutral);
  estimator_theta = (float)RadOfDeg((ahrs.ltp_to_body_euler.theta * .0139882) - ins_pitch_neutral);
  //gx3_estimator_psi   = (float)RadOfDeg(ahrs.ltp_to_body_euler.psi * .0139882  - ahrs_mag_offset) ;  //ahrs_mag_offset in degs

  estimator_psi = gps_estimator_psi;

  estimator_p = RATE_FLOAT_OF_BFP(ahrs.body_rate.p);
  estimator_q = RATE_FLOAT_OF_BFP(ahrs.body_rate.q);

}
#endif

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

  GX3_freq = 0;
  GX3_ltime = 0;
  AHRS_freq = 0;

  
  //Needed to set orientations
  ahrs.status = AHRS_RUNNING;
#ifdef AHRS_ALIGNER_LED
      LED_ON(AHRS_ALIGNER_LED);
#endif
  
}

void imu_impl_init(void) {
  SysTimeTimerStart(gx3_delay_time);  
  gx3_delay_done = FALSE;
  /*Busy wait
  while (!gx3_delay_done) {
    if (SysTimeTimer(gx3_delay_time) > USEC_OF_SEC(1))
        gx3_delay_done = TRUE;
  }*/
   #ifdef USE_GX3
  /* IF THIS IS NEEDED SOME PERHIPHERAL THEN PLEASE MOVE IT THERE */
  for (uint32_t startup_counter=0; startup_counter<2000000; startup_counter++){
    __asm("nop");
  }
#endif


   //4 byte command for Continous Mode
   GX3Link(Transmit(0xc4));
   GX3Link(Transmit(0xc1));
   GX3Link(Transmit(0x29));
   GX3Link(Transmit(0xc8)); // accel,gyro,R

   GX3_packet.status = 0;
   GX3_packet.msg_idx = 0;

   //Start again for loop timing
   SysTimeTimerStart(gx3_delay_time); 
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
	GX3_time 	= GX3_TIME(GX3_packet.msg_buf);
	GX3_chksm	= GX3_CHKSM(GX3_packet.msg_buf);

        GX3_calcsm = 0;
        

        GX3_freq = ((GX3_time - GX3_ltime))/16000000.0;
	GX3_freq = 1.0/GX3_freq;
        GX3_ltime = GX3_time;

	AHRS_freq = (gx3_delay_time-gx3_stop_time)/1000000.0;
	AHRS_freq = 1.0/AHRS_freq;
	gx3_stop_time = gx3_delay_time;

        SysTimeTimerStart(gx3_delay_time); 


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
        
        INT32_QUAT_OF_EULERS(ahrs.ltp_to_imu_quat, ahrs.ltp_to_imu_euler);
        compute_body_orientation_and_rates();

	/* Add Mag Offset */
	ahrs.ltp_to_body_euler.psi += -ANGLE_BFP_OF_REAL(ahrs_mag_offset); 

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
    } else {
      GX3_packet.hdr_error++;
    }
    break;
  case READING: 
    GX3_packet.msg_buf[GX3_packet.msg_idx] =  c;
    GX3_packet.msg_idx++;
    if (GX3_packet.msg_idx == GX3_MSG_LEN) {
      if (GX3_verify_chk(GX3_packet.msg_buf)) {
        GX3_packet.msg_available = TRUE;
      } else {
        GX3_packet.msg_available = FALSE;
        GX3_packet.chksm_error++;
      }
    GX3_packet.status = 0;
    }
    break;
  case DONE: 
  default:
    GX3_packet.status = 0;
    GX3_packet.msg_idx = 0;
    break;
  }
}

