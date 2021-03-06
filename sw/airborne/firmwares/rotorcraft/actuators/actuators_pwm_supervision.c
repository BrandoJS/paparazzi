/*
 * Copyright (C) 2010-2012 The Paparazzi Team
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
 */

/** @file actuators_pwm_supervision.h
 *  PWM actuators with supervision.
 */

#include "generated/airframe.h"
#include "firmwares/rotorcraft/actuators.h"
#include "firmwares/rotorcraft/commands.h"
#include "subsystems/radio_control.h"

/* warn if SUPERVISION_STOP_MOTOR is not define in the airframe file */
#ifndef SUPERVISION_STOP_MOTOR
#warning "STOP_MOTOR is not defined in the SUPERVISION section, are you sure you want to use the default of 0?"
#endif

#include "actuators_pwm_supervision.h"

/* let's start butchery now and use the actuators_pwm arch functions */
#include "firmwares/rotorcraft/actuators/actuators_pwm.h"

#define actuators actuators_pwm_values
#define Actuator(_x) actuators_pwm_values[_x]
#define ActuatorsCommit() {}

/** actuator PWM values in usec. */
int32_t actuators_pwm_values[ACTUATORS_PWM_NB];

void actuators_init(void)
{
  supervision_init();
  actuators_pwm_arch_init();
}

void actuators_set(bool_t motors_on) {

  int32_t pwm_commands[COMMANDS_NB];
  int32_t pwm_commands_pprz[COMMANDS_NB];
  //int32_t booz2_commands[COMMANDS_NB];

  pwm_commands[COMMAND_ROLL] = commands[COMMAND_ROLL] * PWM_GAIN_SCALE;
  pwm_commands[COMMAND_PITCH] = commands[COMMAND_PITCH] * PWM_GAIN_SCALE;
  pwm_commands[COMMAND_YAW] = commands[COMMAND_YAW] * PWM_GAIN_SCALE;
  pwm_commands[COMMAND_THRUST] = (commands[COMMAND_THRUST] * ((SUPERVISION_MAX_MOTOR - SUPERVISION_MIN_MOTOR) / 200)) + SUPERVISION_MIN_MOTOR;

  pwm_commands_pprz[COMMAND_ROLL] = commands[COMMAND_ROLL] * (MAX_PPRZ / 100);
  pwm_commands_pprz[COMMAND_PITCH] = commands[COMMAND_PITCH] * (MAX_PPRZ / 100);
  pwm_commands_pprz[COMMAND_YAW] = commands[COMMAND_YAW] * (MAX_PPRZ / 100);


  /* set normal control surface actuators, i.e. servos */
  SetActuatorsFromCommands(commands);

  /* run supervision for actuators (motor controllers) that need mixing */
  supervision_run(motors_on, FALSE, commands);

  for (int i = 0; i < SUPERVISION_NB_MOTOR; i++) {
    actuators_pwm_values[i] = supervision.commands[i];
  }
  
  actuators_pwm_commit();

}

